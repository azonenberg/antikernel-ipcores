`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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
	@file
	@author Andrew D. Zonenberg
	@brief Controller for quad SPI (or single SPI) NOR Flash memory
 */
module QuadSPIFlashController(

	//The main system clock
	input wire				clk,

	//The SPI bus (connected to top-level ports)
	//TODO: have separate tristate enables for each pin?
	output wire				spi_sck,
	inout wire[3:0] 		spi_dq,
	output reg				spi_cs_n 		= 1,

	//Divider for the SPI bus
	input wire[15:0]		clkdiv,

	//Control signals
	input wire				chip_erase_en,
	output reg				busy			= 1,

	//DEBUG
	input wire				uart_rxd,
	output wire				uart_txd
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration

	//This should be default, probably wont need to get changed unless the memory is weird
	parameter SAMPLE_EDGE = "RISING";
	parameter LOCAL_EDGE = "INVERTED";

	//Set to 0 (default) if the quad pins are connected to the flash on the PCB.
	//If this value is 1, we won't attempt to use quad mode even if the SFDP says it's supported.
	parameter QUAD_DISABLE = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The SPI controller

	assign		spi_dq[1] = 1'bz;	//tri-state MISO

	assign		spi_dq[2] = 1'b1;	//drive high (inactive) for now
	assign		spi_dq[3] = 1'b1;	//drive high (inactive) for now

	reg			shift_en		= 0;
	wire		shift_done;

	reg[7:0]	spi_tx_data		= 0;
	wire[7:0]	spi_rx_data;

	SPITransceiver #(
		.SAMPLE_EDGE("RISING"),
		.LOCAL_EDGE("INVERTED")
	) txvr (
		.clk(clk),
		.clkdiv(clkdiv),

		.spi_sck(spi_sck),
		.spi_mosi(spi_dq[0]),
		.spi_miso(spi_dq[1]),

		.shift_en(shift_en),
		.shift_done(shift_done),
		.tx_data(spi_tx_data),
		.rx_data(spi_rx_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The SFDP parser

    wire		sfdp_shift_en;
    wire[7:0]	sfdp_tx_data;
    wire		sfdp_cs_n;

	//The parser
    SFDPParser parser(
		.clk(clk),

		//.scan_start(la_ready),
		.scan_done(),

		.shift_en(sfdp_shift_en),
		.shift_done(shift_done),
		.spi_tx_data(sfdp_tx_data),
		.spi_rx_data(spi_rx_data),
		.spi_cs_n(sfdp_cs_n),

		.uart_rxd(uart_rxd),
		.uart_txd(uart_txd)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SPI bus arbitration

	reg		last_sender_was_sfdp	= 0;

	always @(*) begin

		//Mux CS
		if(last_sender_was_sfdp)
			spi_cs_n		<= sfdp_cs_n;
		else
			spi_cs_n		<= 1;

	end

    always @(posedge clk) begin
		shift_en					<= 0;

		//Send a shift request if the SFDP core wants it
		if(sfdp_shift_en) begin
			last_sender_was_sfdp	<= 1;
			shift_en				<= 1;
			spi_tx_data				<= sfdp_tx_data;
		end

    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The actual flash controller

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Our logic analyzer

endmodule
