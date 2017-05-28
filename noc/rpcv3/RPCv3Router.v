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
	@brief Router for RPC network, protocol version 3

	High level architecture: grid of stars.

	Each router peers with other routers to the north/south/east/west, and
	has up to 256 IP cores attached to its crossbar.

	All networks must a router at (0,0).
	The network must be convex (no internal cutouts allowed).
 */
module RPCv3Router
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

	//Bit indicating if we have a neighbor in each direction. The transceiver is optimized out, and we return
	//RPC_TYPE_HOST_UNREACH to any traffic sent in that direction.
	//Concatenated {north, south, east, west}
	parameter NEIGHBOR_PRESENT 				= {4'b0000},

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
	input wire								clk,

	//Interfaces to neighboring routers. Concatenated {north, south, east, west}
	//Declare all links 128 bits wide and let unused bits get optimized out
	output wire[3:0]						neighbor_tx_en,
	output wire[511:0]						neighbor_tx_data,
	input wire[3:0]							neighbor_tx_ready,

	input wire[3:0]							neighbor_rx_en,
	input wire[511:0]						neighbor_rx_data,
	output wire[3:0]						neighbor_rx_ready,

	//Interfaces to child nodes.
	//Declare all links 128 bits wide and let unused bits get optimized out
	output wire[CHILD_COUNT-1 : 0]			child_tx_en,
	output wire[CHILD_COUNT*128 - 1 : 0]	child_tx_data,
	input wire[CHILD_COUNT-1 : 0]			child_tx_ready,

	input wire[CHILD_COUNT-1 : 0]			child_rx_en,
	input wire[CHILD_COUNT*128 - 1 : 0]		child_rx_data,
	output wire[CHILD_COUNT-1 : 0]			child_rx_ready
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synthesis-time sanity checking

	initial begin

		//Must have exactly 1 child if we're a trunk
		if(CHILD_IS_TRUNK) begin
			if(CHILD_COUNT != 1) begin
				$display("ERROR: RPCv3Router: must have only one child if CHILD_IS_TRUNK is set");
				$finish;
			end
		end

		//Must have <256 children otherwise
		else if(CHILD_COUNT > 256) begin
			$display("ERROR: RPCv3Router: must have <256 children");
			$finish;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receivers for neighbor ports

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receiver FIFOs for neighbor ports

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receivers for child ports

	wire[CHILD_COUNT-1:0]					rpc_fab_rx_space_available;
	wire[CHILD_COUNT-1:0]					rpc_fab_rx_packet_start;
	wire[CHILD_COUNT-1:0]					rpc_fab_rx_data_valid;
	wire[CHILD_COUNT*CORE_DATA_WIDTH-1:0]	rpc_fab_rx_data;
	wire[CHILD_COUNT-1:0]					rpc_fab_rx_packet_done;

	genvar i;
	generate
		for(i=0; i<CHILD_COUNT; i=i+1) begin : rxvrs

			RPCv3RouterReceiver #(
				.IN_DATA_WIDTH(CHILD_DATA_WIDTH[i*8 +: 8]),
				.OUT_DATA_WIDTH(CORE_DATA_WIDTH)
			) rxvr (
				.clk(clk),
				.rpc_rx_en(child_rx_en[i]),
				.rpc_rx_data(child_rx_data[i*128 +: CHILD_DATA_WIDTH[i*8 +: 8] ] ),
				.rpc_rx_ready(child_rx_ready[i]),

				.rpc_fab_rx_space_available(rpc_fab_rx_space_available[i]),
				.rpc_fab_rx_packet_start(rpc_fab_rx_packet_start[i]),
				.rpc_fab_rx_data_valid(rpc_fab_rx_data_valid[i]),
				.rpc_fab_rx_data(rpc_fab_rx_data[i*CORE_DATA_WIDTH +: CORE_DATA_WIDTH]),
				.rpc_fab_rx_packet_done(rpc_fab_rx_packet_done[i])
			);

		end
	endgenerate

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receiver FIFOs for child ports

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmitters for neighbor ports

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmitters for child ports

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual switch crossbar

endmodule


