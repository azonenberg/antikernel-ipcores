`default_nettype none
`timescale 1ns / 1ps
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
	@brief The main crossbar of the RPCv3 router
 */
module RPCv3RouterCrossbar
#(
	//Data width (must be one of 16, 32, 64, 128).
	parameter CORE_DATA_WIDTH 				= 32,

	//Configuration of this router's child port(s)
	//1: one child interface with 256 addresses, goes to a debug bridge / soft CPU etc
	//0: up to 256 child interfaces with one address each, goes to IP cores
	parameter CHILD_IS_TRUNK				= 1'b0,

	//Number of child ports (must be 1 for CHILD_IS_TRUNK = 1)
	//Must be <= 256.
	parameter CHILD_COUNT 					= 4,

	//Width of the bus going to each child node.
	//8 bits per link, must be 16/32/64/128.
	parameter CHILD_DATA_WIDTH				= 32'h20202020,

	//Bit indicating if we have a neighbor in each direction. The transceiver is optimized out.
	//TODO: Return RPC_TYPE_HOST_UNREACH to any traffic sent in that direction.
	//For now, packets sent there just get silently dropped.
	//Concatenated {north, south, east, west}
	parameter NEIGHBOR_PRESENT 				= {4'b1111},

	//Width of the bus going to each router, or zero if no router in that direction.
	//8 bits per link, must be 16/32/64/128.
	//Concatenated {north, south, east, west}
	parameter NEIGHBOR_DATA_WIDTH 			= 32'h20202020,

	//Coordinates of this router in the grid.
	//The base address of this router is {X_POS, Y_POS, 8'h00}.
	parameter X_POS 						= 4'h0,
	parameter Y_POS 						= 4'h0
)
(
	//Internal clock (also used for all links, for now)
	input wire												clk,

	//RX-side FIFO interface
	//Packed {children, neighbors}
	output reg[(CHILD_COUNT+4)-1 : 0]						rx_fifo_rd		= 0,
	input wire[(CHILD_COUNT+4)-1 : 0]						rx_fifo_empty,
	input wire[(CHILD_COUNT+4)*CORE_DATA_WIDTH-1 : 0]		rx_fifo_dout,
	input wire[(CHILD_COUNT+4)*6-1 : 0]						rx_fifo_rsize,

	//Debug LEDs
	output reg[3:0]											leds			= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration of the crossbar

	//We can have up to 260 ports total (256 children + 4 neighbors) in a worst-case scenario.
	//Use a 9-bit integer as the port ID (high bits will get optimized out as needed).
	//ID mapping:
	//5 = child address 1
	//4 = child address 0
	//3 = north neighbor
	//2 = south neighbor
	//1 = east neighbor
	//0 = west neighbor

	//Figure out how many ports we have in this crossbar (including unused neighbors)
	localparam TOTAL_PORTS		= 4 + CHILD_COUNT;

	//Number of clocks it takes us to process a message
	localparam MESSAGE_WORDS 	= 128 / CORE_DATA_WIDTH;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Figure out where each port wants to send.

	reg[TOTAL_PORTS-1:0]	fifo_dout_valid		= 0;

	integer i;
	always @(posedge clk) begin

		//If there is at least one full message in the RX FIFO, but we don't have a header ready yet,
		//we can pop the address header word and go from there.

	end


endmodule
