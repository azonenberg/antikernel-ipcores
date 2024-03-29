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
`include "SGMIIToGMIIBridge.svh"

/**
	@brief Bridge from SGMII to GMII
 */
module SGMIIToGMIIBridge #(
	parameter RX_INVERT = 0,
	parameter TX_INVERT = 0
)(

	//Main clocks
	input wire			clk_125mhz,
	input wire			clk_312p5mhz,

	//Oversampling clocks for receiver
	input wire			clk_625mhz_0,
	input wire			clk_625mhz_90,

	input wire			sgmii_rx_data_p,
	input wire			sgmii_rx_data_n,

	output wire			sgmii_tx_data_p,
	output wire			sgmii_tx_data_n,

	//clk_125mhz domain
	output GmiiBus		gmii_rx_bus,
	input GmiiBus		gmii_tx_bus,

	output wire			link_up,
	output lspeed_t		link_speed,

	input wire			rst_stat,
	output SGMIIPerformanceCounters	perf,

	output wire			rx_error
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Oversampling clock/data recovery block

	//This module converts the raw differential pair into a stream of 4 bits (average) per clock at 312.5 MHz.

	wire[4:0]	rx_data;
	wire[2:0]	rx_data_count;

	OversamplingCDR #(
		.INVERT(RX_INVERT)
	) cdr (
		.clk_625mhz_0(clk_625mhz_0),
		.clk_625mhz_90(clk_625mhz_90),
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

	assign rx_error = rx_disparity_err | rx_symbol_err;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output buffer for TX side

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

	wire[9:0] serdes_tx_data;
	assign serdes_tx_data = TX_INVERT ? ~tx_codeword : tx_codeword;

	wire[9:0] serdes_tx_data_mirrored =
	{
		serdes_tx_data[0],
		serdes_tx_data[1],
		serdes_tx_data[2],
		serdes_tx_data[3],
		serdes_tx_data[4],
		serdes_tx_data[5],
		serdes_tx_data[6],
		serdes_tx_data[7],
		serdes_tx_data[8],
		serdes_tx_data[9]
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Serialize the outbound TX data

	logic		serdes_reset = 1;
	logic[7:0]	serdes_reset_count = 1;
	always_ff @(posedge tx_clk) begin
		if(serdes_reset_count == 0)
			serdes_reset		<= 0;
		else
			serdes_reset_count <= serdes_reset_count + 1;
	end

	wire	serdes_cascade1;
	wire	serdes_cascade2;
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"),
		.DATA_RATE_TQ("BUF"),
		.DATA_WIDTH(10),
		.SERDES_MODE("MASTER"),
		.TRISTATE_WIDTH(1),
		.TBYTE_CTL("FALSE"),
		.TBYTE_SRC("FALSE")
	) tx_serdes_master (
		.OQ(tx_data_serial),
		.OFB(),
		.TQ(),
		.TFB(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.CLK(clk_625mhz_0),
		.CLKDIV(tx_clk),
		.D1(serdes_tx_data_mirrored[0]),
		.D2(serdes_tx_data_mirrored[1]),
		.D3(serdes_tx_data_mirrored[2]),
		.D4(serdes_tx_data_mirrored[3]),
		.D5(serdes_tx_data_mirrored[4]),
		.D6(serdes_tx_data_mirrored[5]),
		.D7(serdes_tx_data_mirrored[6]),
		.D8(serdes_tx_data_mirrored[7]),
		.TCE(1'b1),
		.OCE(1'b1),
		.TBYTEIN(1'b0),
		.TBYTEOUT(),
		.RST(serdes_reset),
		.SHIFTIN1(serdes_cascade1),
		.SHIFTIN2(serdes_cascade2),
		.T1(1'b0),
		.T2(1'b0),
		.T3(1'b0),
		.T4(1'b0)
	);

	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"),
		.DATA_RATE_TQ("BUF"),
		.DATA_WIDTH(10),
		.SERDES_MODE("SLAVE"),
		.TRISTATE_WIDTH(1),
		.TBYTE_CTL("FALSE"),
		.TBYTE_SRC("FALSE")
	) tx_serdes_slave (
		.OQ(),
		.OFB(),
		.TQ(),
		.TFB(),
		.SHIFTOUT1(serdes_cascade1),
		.SHIFTOUT2(serdes_cascade2),
		.CLK(clk_625mhz_0),
		.CLKDIV(tx_clk),
		.D1(1'b0),
		.D2(1'b0),
		.D3(serdes_tx_data_mirrored[8]),
		.D4(serdes_tx_data_mirrored[9]),
		.D5(1'b0),
		.D6(1'b0),
		.D7(serdes_tx_data_mirrored[8]),	//UG471 says to use D3/D4
		.D8(serdes_tx_data_mirrored[9]),	//but it seems to only work if we use D7/D8??
		.TCE(1'b1),
		.OCE(1'b1),
		.TBYTEIN(1'b0),
		.TBYTEOUT(),
		.RST(serdes_reset),
		.SHIFTIN1(),
		.SHIFTIN2(),
		.T1(1'b0),
		.T2(1'b0),
		.T3(1'b0),
		.T4(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 1000base-X / SGMII PCS

	GigBaseXPCS pcs(
		.clk_125mhz(clk_125mhz),

		.sgmii_mode(1'b1),

		.rx_clk(clk_312p5mhz),
		.rx_data_valid(rx_8b_data_valid),
		.rx_data_is_ctl(rx_8b_data_is_ctl),
		.rx_data(rx_8b_data),
		.rx_disparity_err(rx_disparity_err),
		.rx_symbol_err(rx_symbol_err),

		.link_up(link_up),
		.link_speed(link_speed),

		.gmii_rx_bus(gmii_rx_bus),
		.gmii_tx_bus(gmii_tx_bus),

		.tx_clk(tx_clk),
		.tx_data_is_ctl(tx_data_is_ctl),
		.tx_data(tx_data),
		.tx_force_disparity_negative(tx_force_disparity_negative),
		.tx_disparity_negative(tx_disparity_negative)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Performance counters in fast clock domain

	SGMIIPerformanceCounters	fastperf	= {$bits(SGMIIPerformanceCounters){1'b0}};

	logic	first 	= 1;

	wire	rst_stat_sync;
	ThreeStageSynchronizer sync_rst_stat(
		.clk_in(clk_125mhz),
		.din(rst_stat),
		.clk_out(clk_312p5mhz),
		.dout(rst_stat_sync)
	);

	always_ff @(posedge clk_312p5mhz) begin
		if(rx_disparity_err)
			fastperf.rx_disparity_errs	<= fastperf.rx_disparity_errs + 1'h1;
		if(rx_symbol_err)
			fastperf.rx_symbol_errs		<= fastperf.rx_symbol_errs + 1'h1;
		if(rx_bitslip)
			fastperf.rx_bitslips		<= fastperf.rx_bitslips + 1'h1;
		if(rx_8b_data_valid)
			fastperf.rx_symbols			<= fastperf.rx_symbols + 1'h1;

		if(rst_stat_sync)
			fastperf					<= {$bits(SGMIIPerformanceCounters){1'b0}};

		first	<= 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize counters into slow clock domain

	wire	sync_ack;
	RegisterSynchronizer #(
		.WIDTH($bits(SGMIIPerformanceCounters)),
		.INIT(0)
	) sync_perf (
		.clk_a(clk_312p5mhz),
		.en_a(first || sync_ack),
		.ack_a(sync_ack),
		.reg_a(fastperf),

		.clk_b(clk_125mhz),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(perf)
	);

endmodule
