`default_nettype none
`timescale 1ns/1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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
 */
module SGMIIToGMIIBridge #(
	parameter RX_INVERT = 0
)(

	//Main clock
	input wire			clk_312p5mhz,

	//Oversampling clocks for receiver
	input wire			clk_625mhz_fabric,
	input wire			clk_625mhz_io_0,
	input wire			clk_625mhz_io_90,

	input wire			sgmii_rx_data_p,
	input wire			sgmii_rx_data_n,
	/*
	output wire			sgmii_tx_data_p,
	output wire			sgmii_tx_data_n,

	output wire			gmii_rx_clk,
	output GmiiBus		gmii_rx_bus,

	input wire			gmii_tx_clk,
	input GmiiBus		gmii_tx_bus,*/

	output wire			link_up,
	output lspeed_t		link_speed
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Oversampling clock/data recovery block

	//This module converts the raw differential pair into a stream of 4 bits (average) per clock at 312.5 MHz.

	wire[4:0]	rx_data;
	wire[2:0]	rx_data_count;

	OversamplingCDR #(
		.INVERT(RX_INVERT)
	) cdr (
		.clk_625mhz_io_0(clk_625mhz_io_0),
		.clk_625mhz_io_90(clk_625mhz_io_90),
		.clk_625mhz_fabric(clk_625mhz_fabric),
		.clk_312p5mhz(clk_312p5mhz),

		.din_p(sgmii_rx_data_p),
		.din_n(sgmii_rx_data_n),

		.rx_data(rx_data),
		.rx_data_count(rx_data_count)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Convert the variable rate output to a stream of 10-bit line code symbols plus a clock enable

	//TODO: put this in its own module
	logic[9:0]	rx_symbol = 0;
	logic		rx_symbol_valid = 0;

	wire		rx_bitslip;

	logic[14:0]	rx_symbol_buf = 0;
	logic[3:0]	rx_symbol_count = 0;
	logic[2:0]	rx_data_count_fwd;
	always_ff @(posedge clk_312p5mhz) begin

		rx_symbol_valid <= 0;

		//Skip one bit if we're bitslipping
		if(rx_bitslip)
			rx_data_count_fwd = rx_data_count - 1;
		else
			rx_data_count_fwd = rx_data_count;

		//Extend the symbol buffer from the right (oldest bit at left)
		case(rx_data_count_fwd)
			2: rx_symbol_buf = { rx_symbol_buf[12:0], rx_data[1:0] };
			3: rx_symbol_buf = { rx_symbol_buf[11:0], rx_data[2:0] };
			4: rx_symbol_buf = { rx_symbol_buf[10:0], rx_data[3:0] };
			5: rx_symbol_buf = { rx_symbol_buf[9:0], rx_data[4:0] };
			default: begin
			end
		endcase
		rx_symbol_count = rx_symbol_count + rx_data_count_fwd;

		//Once we have at least 10 valid bits, output them and shift left
		if(rx_symbol_count >= 10) begin
			case(rx_symbol_count)

				10: rx_symbol <= rx_symbol_buf[0 +: 10];
				11: rx_symbol <= rx_symbol_buf[1 +: 10];
				12: rx_symbol <= rx_symbol_buf[2 +: 10];
				13: rx_symbol <= rx_symbol_buf[3 +: 10];
				14: rx_symbol <= rx_symbol_buf[4 +: 10];
				15: rx_symbol <= rx_symbol_buf[5 +: 10];	//shouldn't be possible, but just in case

				//nothing left to do
				default: begin
				end
			endcase

			rx_symbol_count = rx_symbol_count - 10;;
			rx_symbol_valid	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decode the 8b/10b symbols

	wire		rx_8b_data_valid;
	wire[7:0]	rx_8b_data;
	wire		rx_8b_data_is_ctl;

	wire		rx_disparity_err;
	wire		rx_symbol_err;
	wire		rx_locked;

	Decode8b10b rx_decoder(
		.clk(clk_312p5mhz),

		.codeword_valid(rx_symbol_valid),
		.codeword_in(rx_symbol),

		.data_valid(rx_8b_data_valid),
		.data(rx_8b_data),
		.data_is_ctl(rx_8b_data_is_ctl),

		.disparity_err(rx_disparity_err),
		.symbol_err(rx_symbol_err),
		.bitslip(rx_bitslip),
		.locked(rx_locked)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	wire	rx_fault;
	assign rx_fault = rx_locked && (rx_symbol_err || rx_disparity_err);

	ila_0 ila(
		.clk(clk_312p5mhz),
		.probe0(rx_8b_data_valid),
		.probe1(rx_8b_data),
		.probe2(rx_8b_data_is_ctl),
		.probe3(rx_disparity_err),
		.probe4(rx_symbol_err),
		.probe5(rx_bitslip),
		.probe6(rx_locked),
		.probe7(rx_fault)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Differential I/O buffers

	/*
	wire	tx_data_serial;

	DifferentialOutputBuffer #(
		.WIDTH(1),
		.IOSTANDARD("LVDS")
	) obuf_data (
		.pad_out_p(sgmii_tx_data_p),
		.pad_out_n(sgmii_tx_data_n),
		.fabric_in(tx_data_serial)
	);
	*/

	/*

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit 8b/10b coder

	wire		tx_clk;
	wire		tx_data_is_ctl;
	wire[7:0]	tx_data;

	wire[9:0]	tx_codeword;

	wire		tx_force_disparity_negative;
	wire		tx_disparity_negative;

	Encode8b10b tx_encoder(
		.clk(tx_clk),

		.data_is_ctl(tx_data_is_ctl),
		.data(tx_data),
		.force_disparity_negative(tx_force_disparity_negative),

		.codeword(tx_codeword),
		.tx_disparity_negative(tx_disparity_negative)
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
		.clk_125mhz(gmii_rx_clk_raw),

		.sgmii_mode(1'b1),

		.rx_clk(symbol_clk),
		.rx_data_valid(rx_data_valid),
		.rx_data_is_ctl(rx_data_is_ctl),
		.rx_data(rx_data),

		.link_up(link_up),
		.link_speed(link_speed),

		.gmii_rx_bus(gmii_rx_bus),
		.gmii_tx_clk(gmii_tx_clk),
		.gmii_tx_bus(gmii_tx_bus),

		.tx_clk(tx_clk),
		.tx_data_is_ctl(tx_data_is_ctl),
		.tx_data(tx_data),
		.tx_force_disparity_negative(tx_force_disparity_negative),
		.tx_disparity_negative(tx_disparity_negative)
	);
	*/

endmodule
