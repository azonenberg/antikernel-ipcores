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
	@brief UART wrapper for RED TIN logic analyzer

	SIGNAL_ROM format:
		{ 16384'h0 },					//Padding to ensure ROM is always 16Kbits in size
		{ "DEBUGROM" }	,				//Magic header to indicate start of ROM
		{ "uart_tx_en\0", 8'h1, 8'h0 },	//name, width, format TBD (reserved zero)
		{ "uart_txd\0", 8'h8, 8'h0 },
 */
module #(
		parameter WIDTH = 128,
		parameter DEPTH = 512,
		parameter SYMBOL_ROM = 16384'h0,
		parameter UART_CLKDIV = 16'd868			//115200 baud @ 100 MHz
		)
	RedTinUartWrapper(

		//Internal clock, not necessarily used for capturing
		input wire				clk,

		//Data being sniffed
		input wire				capture_clk,
		input wire[WIDTH-1:0]	din,

		//Bus to host PC
		input wire				uart_rx,
		output wire				uart_tx
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The UART

	//8N1 configuration, no others supported
	reg			uart_tx_en		= 0;
	reg[7:0]	uart_tx_data	= 0;
	wire		uart_tx_active;
	wire		uart_rx_en;
	wire[7:0]	uart_rx_data;
	UART uart(
		.clk(clk),
		.clkdiv(UART_CLKDIV[15:0]),

		.tx(uart_txd),
		.txin(uart_tx_data),
		.txrdy(uart_tx_en),
		.txactive(uart_tx_active),

		.rx(uart_rxd),
		.rxout(uart_rx_data),
		.rxrdy(uart_rx_en),
		.rxactive(),

		.overflow()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reshuffle the signal ROM until it fits in a 2Kx8 ROM, and swap bytes so we're ordered properly

	reg[7:0] symbols[2047:0];

	integer i;
	initial begin

		for(i=0; i<2048; i=i+1)
			symbols[i] <= SYMBOL_ROM[(2047 - i)*8 +: 8];

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual LA

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// UART command engine

endmodule
