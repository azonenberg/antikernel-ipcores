`default_nettype none
`timescale 1ns/1ps

/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

`include "GmiiBus.svh"

/**
	@brief Bridge from SGMII to GMII

	Initial "quick and dirty" implementation has no CDR and requires RX clock.
 */
module SGMIIToGMIIBridge(

	input wire			clk_125mhz,

	//SGMII interface (connect directly to top-level pads)
	input wire			sgmii_rx_clk_p,		//625 MHz RX clock
	input wire			sgmii_rx_clk_n,

	input wire			sgmii_rx_data_p,	//1250 Mbps DDR RX data, aligned to sgmii_rx_clk (8b10b coded)
	input wire			sgmii_rx_data_n,

	output wire			sgmii_tx_data_p,
	output wire			sgmii_tx_data_n,

	output wire			gmii_rx_clk,

	output wire			link_up,
	output lspeed_t		link_speed,

	output GmiiBus		rx_gmii_bus
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Differential I/O buffers

	wire	rx_clk;

	DifferentialInputBuffer #(
		.WIDTH(1),
		.IOSTANDARD("LVDS"),
		.ODT(1),
		.OPTIMIZE("SPEED")
	) ibuf_clk (
		.pad_in_p(sgmii_rx_clk_p),
		.pad_in_n(sgmii_rx_clk_n),
		.fabric_out(rx_clk)
	);

	wire	rx_data_serial;

	DifferentialInputBuffer #(
		.WIDTH(1),
		.IOSTANDARD("LVDS"),
		.ODT(1),
		.OPTIMIZE("SPEED")
	) ibuf_data (
		.pad_in_p(sgmii_rx_data_p),
		.pad_in_n(sgmii_rx_data_n),
		.fabric_out(rx_data_serial)
	);

	wire	tx_data_serial;

	DifferentialOutputBuffer #(
		.WIDTH(1),
		.IOSTANDARD("LVDS")
	) obuf_data (
		.pad_out_p(sgmii_tx_data_p),
		.pad_out_n(sgmii_tx_data_n),
		.fabric_in(tx_data_serial)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX PLL (locked to the RX clock)

	wire 	pll_locked;

	wire	serdes_clk_raw;
	wire	gmii_rx_clk_raw;

	//Stick a BUFG into the clock feedback path so that we can compensate for buffer delays
	wire	clkfb_bufg;
	BUFG feedback_buf(
		.I(clkfb),
		.O(clkfb_bufg)
	);

	//TODO: abstraction for this
	wire clkfb;
	MMCME4_BASE #(
		.BANDWIDTH("OPTIMIZED"),

		.CLKIN1_PERIOD(1.600),		//625 MHz Fin
		.DIVCLK_DIVIDE(2),			//312.5 MHz to the PFD
		.CLKFBOUT_MULT_F(4),		//1.25 GHz Fvco
		.CLKFBOUT_PHASE(0),

		.STARTUP_WAIT("FALSE"),		//Don't wait for PLL to lock at startup

		.CLKOUT1_DIVIDE(2),			//625 MHz clock for SERDES
		.CLKOUT2_DIVIDE(5),			//125 MHz clock for GMII subsystem
		.CLKOUT3_DIVIDE(1),
		.CLKOUT4_DIVIDE(1),
		.CLKOUT5_DIVIDE(1),
		.CLKOUT6_DIVIDE(1),

		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5),
		.CLKOUT6_DUTY_CYCLE(0.5),

		.CLKOUT1_PHASE(0),
		.CLKOUT2_PHASE(0),
		.CLKOUT3_PHASE(0),
		.CLKOUT4_PHASE(0),
		.CLKOUT5_PHASE(0),
		.CLKOUT6_PHASE(0)

	) pll (
		.CLKFBIN(clkfb_bufg),
		.CLKFBOUT(clkfb),

		.CLKIN1(rx_clk),

		.CLKOUT1(serdes_clk_raw),
		.CLKOUT2(gmii_rx_clk_raw),
		.CLKOUT3(),
		.CLKOUT4(),
		.CLKOUT5(),
		.CLKOUT6(),

		.LOCKED(pll_locked),

		.PWRDWN(0),

		.RST(0)
	);

	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("YES")
	) bufg_gmii_clk (
		.clkin(gmii_rx_clk_raw),
		.clkout(gmii_rx_clk),
		.ce(pll_locked)
	);

	//156.25 MHz symbol clock
	wire	symbol_clk;

	BUFGCE_DIV #(
		.BUFGCE_DIVIDE(4)
	) bufg_symbol_clk(
		.I(serdes_clk_raw),
		.CLR(1'b0),
		.CE(1'b1),
		.O(symbol_clk)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Deserialize the incoming RX data

	//Raw 8b/10b symbols (need to be gearboxed) coming off the ISERDES
	wire[7:0] rx_serdes_data;

	logic	serdes_reset	= 1;
	always_ff @(posedge symbol_clk) begin
		serdes_reset	<= 0;
	end

	ISERDESE3 #(
		.DATA_WIDTH(8),
		.FIFO_ENABLE("FALSE"),
		.FIFO_SYNC_MODE("FALSE"),
		.IS_CLK_INVERTED(0),
		.IS_CLK_B_INVERTED(1),
		.IS_RST_INVERTED(0),
		.SIM_DEVICE("ULTRASCALE_PLUS")
	) rx_serdes (
		.CLK(serdes_clk_raw),
		.CLK_B(serdes_clk_raw),
		.CLKDIV(symbol_clk),
		.D(rx_data_serial),
		.Q(rx_serdes_data),
		.RST(serdes_reset),
		.FIFO_RD_CLK(1'b0),
		.FIFO_RD_EN(1'b0),
		.FIFO_EMPTY(),
		.INTERNAL_DIVCLK()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Gearbox the 8-bit SERDES data out to 10-bit symbols (need to bitslip to get alignment)

	//convert to "printer friendly" bit ordering for display
	wire[7:0]	rx_serdes_data_mirrored =
	{
		rx_serdes_data[0],
		rx_serdes_data[1],
		rx_serdes_data[2],
		rx_serdes_data[3],
		rx_serdes_data[4],
		rx_serdes_data[5],
		rx_serdes_data[6],
		rx_serdes_data[7]
	};

	wire		rx_symbol_valid;
	wire[9:0]	rx_symbol;
	wire		rx_bitslip;

	Gearbox8To10 rx_gearbox(
		.clk(symbol_clk),

		.din_valid(1'b1),
		.din(rx_serdes_data_mirrored),
		.bitslip(rx_bitslip),

		.dout_valid(rx_symbol_valid),
		.dout(rx_symbol)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decode the 8b/10b symbols

	wire		rx_data_valid;
	wire[7:0]	rx_data;
	wire		rx_data_is_ctl;

	wire		rx_disparity_err;
	wire		rx_symbol_err;
	wire		rx_locked;

	Decode8b10b rx_decoder(
		.clk(symbol_clk),

		.codeword_valid(rx_symbol_valid),
		.codeword_in(rx_symbol),

		.data_valid(rx_data_valid),
		.data(rx_data),
		.data_is_ctl(rx_data_is_ctl),

		.disparity_err(rx_disparity_err),
		.symbol_err(rx_symbol_err),
		.bitslip(rx_bitslip),
		.locked(rx_locked)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit 8b/10b coder

	wire		tx_clk;
	wire		tx_data_is_ctl;
	wire[7:0]	tx_data;

	wire[9:0]	tx_codeword;

	wire		tx_force_disparity_negative;

	Encode8b10b tx_encoder(
		.clk(tx_clk),

		.data_is_ctl(tx_data_is_ctl),
		.data(tx_data),
		.force_disparity_negative(tx_force_disparity_negative),

		.codeword(tx_codeword)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit cross-clock FIFO

	wire		tx_fifo_rd;
	wire[9:0]	tx_fifo_symbol;
	wire[5:0]	tx_fifo_rsize;

	//Push at 125 MHz, pop at 156.25
	CrossClockFifo #(
		.WIDTH(10),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) tx_fifo (
		.wr_clk(tx_clk),
		.wr_en(1'b1),
		.wr_data(tx_codeword),
		.wr_size(),
		.wr_full(),
		.wr_overflow(),

		.rd_clk(symbol_clk),
		.rd_en(tx_fifo_rd),
		.rd_data(tx_fifo_symbol),
		.rd_size(tx_fifo_rsize),
		.rd_empty(),
		.rd_underflow()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit gearbox

	wire[7:0]	serdes_tx_data;

	Gearbox10To8 tx_gearbox (
		.clk(symbol_clk),
		.fifo_ready(tx_fifo_rsize > 8),
		.fifo_rd_en(tx_fifo_rd),
		.fifo_rd_data(tx_fifo_symbol),

		.dout(serdes_tx_data)
	);

	wire[7:0] serdes_tx_data_mirrored =
	{
		serdes_tx_data[0],
		serdes_tx_data[1],
		serdes_tx_data[2],
		serdes_tx_data[3],
		serdes_tx_data[4],
		serdes_tx_data[5],
		serdes_tx_data[6],
		serdes_tx_data[7]
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Serialize the outbound TX data

	OSERDESE3 #(
		.DATA_WIDTH(8),
		.INIT(0),
		.ODDR_MODE("FALSE"),
		.OSERDES_D_BYPASS("FALSE"),
		.OSERDES_T_BYPASS("FALSE"),
		.IS_CLK_INVERTED(0),
		.IS_CLKDIV_INVERTED(0),
		.IS_RST_INVERTED(0),
		.SIM_DEVICE("ULTRASCALE_PLUS")
	) tx_serdes (
		.CLK(serdes_clk_raw),
		.CLKDIV(symbol_clk),
		.D(serdes_tx_data_mirrored),
		.OQ(tx_data_serial),
		.RST(serdes_reset),
		.T_OUT(),
		.T(8'h0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 1000base-X / SGMII PCS

	GigBaseXPCS pcs(
		.clk_125mhz(clk_125mhz),

		.sgmii_mode(1'b1),

		.rx_clk(symbol_clk),
		.rx_data_valid(rx_data_valid),
		.rx_data_is_ctl(rx_data_is_ctl),
		.rx_data(rx_data),

		.link_up(link_up),
		.link_speed(link_speed),

		.rx_gmii_bus(rx_gmii_bus),

		.tx_clk(tx_clk),
		.tx_data_is_ctl(tx_data_is_ctl),
		.tx_data(tx_data),
		.tx_force_disparity_negative(tx_force_disparity_negative)
	);

endmodule
