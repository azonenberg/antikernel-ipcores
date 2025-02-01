`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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

/**
	@brief A bidirectional bridge from APB to a GTY
 */
module GTY_APBBridge #(
	parameter TX_INVERT	= 0,
	parameter RX_INVERT = 0
) (
	//Clocks
	input wire		sysclk,		//Management APB clock

	//GTY quad reference clocks
	input wire[1:0]	clk_ref,

	//Top level GTY pins
	input wire	rx_p,
	input wire	rx_n,

	output wire tx_p,
	output wire tx_n,

	//QPLL clocks and control signals
	input wire[1:0]		qpll_clkout,
	input wire[1:0]		qpll_refout,
	input wire[1:0]		qpll_lock
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset logic

	logic tx_reset = 1;
	logic rx_reset = 1;
	logic[7:0] count = 1;

	always_ff @(posedge sysclk) begin
		if(count == 0) begin
			tx_reset	<= 0;
			rx_reset	<= 0;
		end
		else
			count <= count + 1;
	end

	logic	rxuserrdy = 0;
	logic	txuserrdy = 0;

	wire	rxoutclk;
	wire	txoutclk;

	//TODO: move reset logic internal
	logic[7:0] rxcount = 1;
	always_ff@(posedge rxoutclk) begin
		if(rxcount != 0)
			rxcount <= rxcount + 1;
		else
			rxuserrdy	<= 1;

		if(rx_reset) begin
			rxuserrdy	<= 0;
			rxcount		<= 1;
		end
	end

	logic[7:0] txcount = 1;
	always_ff@(posedge txoutclk) begin
		if(txcount != 0)
			txcount <= txcount + 1;
		else
			txuserrdy	<= 1;

		if(tx_reset) begin
			txuserrdy	<= 0;
			txcount		<= 1;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The transceiver lane

	//TODO: make the apb do something
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(10), .USER_WIDTH(0)) serdes_apb();
	assign serdes_apb.pclk = sysclk;
	assign serdes_apb.preset_n = 1'b0;

	wire[39:0]	rx_data;
	wire[39:0]	tx_data;
	wire		rx_comma_is_aligned;
	wire[15:0]	rx_char_is_k;
	wire[15:0]	rx_char_is_comma;
	wire[15:0]	tx_char_is_k;

	GTYLane_UltraScale #(
		.ROUGH_RATE_GBPS(5),	//TODO parameterize
		.DATA_WIDTH(40),
		.RX_COMMA_ALIGN(1)
	) lane (
		.apb(serdes_apb),

		.rx_p(rx_p),
		.rx_n(rx_n),

		.tx_p(tx_p),
		.tx_n(tx_n),

		.rx_reset(rx_reset),
		.tx_reset(tx_reset),

		.tx_data(tx_data),
		.rx_data(rx_data),

		.clk_ref_north(2'b0),
		.clk_ref_south(2'b0),
		.clk_ref(clk_ref),
		.clk_lockdet(sysclk),

		.rxoutclk(rxoutclk),
		.rxusrclk(rxoutclk),
		.rxusrclk2(rxoutclk),
		.rxuserrdy(rxuserrdy),

		.txoutclk(txoutclk),
		.txusrclk(txoutclk),
		.txusrclk2(txoutclk),
		.txuserrdy(txuserrdy),

		.rxpllclksel(2'b10),			//QPLL1 hard coded for now
		.txpllclksel(2'b10),

		.qpll_clk(qpll_clkout),
		.qpll_refclk(qpll_refout),
		.qpll_lock(qpll_lock),

		.cpll_pd(1'b1),					//Not using CPLL for now
		.cpll_fblost(),
		.cpll_reflost(),
		.cpll_lock(),
		.cpll_refclk_sel(3'd1),			//set to 1 when only using one clock source even if it's not GTREFCLK0??

		.tx_rate(3'b011),				//divide by 4 (5 Gbps)
		.rx_rate(3'b011),

		.rx_ctle_en(1'b1),

		.txdiffctrl(5'h8),
		.txpostcursor(5'h0),
		.txprecursor(5'h8),
		.tx_invert(TX_INVERT),
		.rx_invert(RX_INVERT),

		.rxprbssel(4'b0),
		.txprbssel(4'b0),

		.rx_8b10b_decode(1'b1),
		.rx_comma_is_aligned(rx_comma_is_aligned),
		.rx_char_is_k(rx_char_is_k),
		.rx_char_is_comma(rx_char_is_comma),

		.tx_8b10b_encode(1'b1),
		.tx_char_is_k(tx_char_is_k)
	);

	//comma in LSB position
	assign tx_char_is_k = 16'b0001;

	//K28.5 D21.5 D0.0 D0.0
	//Least significant lane sent first on the wire; extra 8 bits are for encoder and not used
	assign tx_data = 40'h00_00_00_b5_bc;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_0 ila(
		.clk(rxoutclk),
		.probe0(rx_data),
		.probe1(rx_comma_is_aligned),
		.probe2(rx_char_is_k),
		.probe3(rx_char_is_comma)
	);

endmodule
