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

	//Bit indicating if we have a neighbor in each direction. The transceiver is optimized out.
	//TODO: Return RPC_TYPE_HOST_UNREACH to any traffic sent in that direction
	//Concatenated {north, south, east, west}
	parameter NEIGHBOR_PRESENT 				= {4'b1111},

	//Width of the bus going to each router, or zero if no router in that direction.
	//8 bits per link, must be 16/32/64/128.
	//Concatenated {north, south, east, west}
	parameter NEIGHBOR_DATA_WIDTH 			= 32'h20202020,

	//Coordinates of this router in the grid.
	//The base address of this router is {X_POS, Y_POS, 8'h00}.
	//North = positive Y
	//East = positive X
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
	// Compute a few useful constants

	//Number of clocks in one message through the core
	localparam CORE_WORD_COUNT = 128 / CORE_DATA_WIDTH;

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
	// Receivers and FIFOs for neighbor ports

	wire[3:0]						neighbor_fifo_rd;
	wire[3:0]						neighbor_fifo_empty;
	wire[CORE_DATA_WIDTH*4 - 1:0]	neighbor_fifo_dout;
	wire[23:0]						neighbor_fifo_rsize;

	genvar i;
	generate
		for(i=0; i<4; i=i+1) begin : neighbor_rxs

			if(NEIGHBOR_PRESENT[i]) begin

				//Bus from receiver to FIFO
				wire						fifo_space_available;
				wire[5:0]					fifo_wr_size;
				wire						fifo_wr_en;
				wire[CORE_DATA_WIDTH-1:0]	fifo_wr_data;

				//True if there is enough room in the FIFO for one entire packet
				assign fifo_space_available	= (fifo_wr_size >= CORE_WORD_COUNT);

				//Receiver pushes data directly to FIFO.
				//Ignore packet start/done signals, we only care about the data bus.
				//Receiver is proven to never send partial packets, so we can't lose sync!
				RPCv3RouterReceiver #(
					.IN_DATA_WIDTH(NEIGHBOR_DATA_WIDTH[i*8 +: 8]),
					.OUT_DATA_WIDTH(CORE_DATA_WIDTH)
				) rxvr (
					.clk(clk),
					.rpc_rx_en(neighbor_rx_en[i]),
					.rpc_rx_data(neighbor_rx_data[i*128 +: NEIGHBOR_DATA_WIDTH[i*8 +: 8] ] ),
					.rpc_rx_ready(neighbor_rx_ready[i]),

					.rpc_fab_rx_space_available(fifo_space_available),
					.rpc_fab_rx_packet_start(),
					.rpc_fab_rx_data_valid(fifo_wr_en),
					.rpc_fab_rx_data(fifo_wr_data),
					.rpc_fab_rx_packet_done()
				);

				SingleClockShiftRegisterFifo #(
					.WIDTH(CORE_DATA_WIDTH),
					.DEPTH(32),
					.OUT_REG(1)
				) rx_fifo (

					.clk(clk),

					.wr(fifo_wr_en),
					.din(fifo_wr_data),

					.rd(neighbor_fifo_rd[i]),
					.dout(neighbor_fifo_dout[i*CORE_DATA_WIDTH +: CORE_DATA_WIDTH]),
					.overflow(),			//ignored, can never under/overflow b/c of receiver flow control
					.underflow(),
					.empty(neighbor_fifo_empty[i]),
					.full(),
					.rsize(neighbor_fifo_rsize[6*i +: 6]),
					.wsize(fifo_wr_size),

					.reset(1'b0)		//never reset the fifo
				);


			end

			//No neighbor? Tie everything off to zero
			else begin
				assign	neighbor_fifo_empty[i] = 1;
				assign	neighbor_fifo_rsize[i*6 +: 6] = 0;
				assign	neighbor_fifo_dout[i*CORE_DATA_WIDTH +: CORE_DATA_WIDTH] = 0;
			end

		end
	endgenerate

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receivers and FIFOs for child ports

	wire[CHILD_COUNT-1:0]						child_fifo_rd;
	wire[CHILD_COUNT-1:0]						child_fifo_empty;
	wire[CHILD_COUNT*CORE_DATA_WIDTH - 1:0]		child_fifo_dout;
	wire[CHILD_COUNT*6-1:0]						child_fifo_rsize;

	generate
		for(i=0; i<CHILD_COUNT; i=i+1) begin : child_rxs

			//Bus from receiver to FIFO
			wire						fifo_space_available;
			wire[5:0]					fifo_wr_size;
			wire						fifo_wr_en;
			wire[CORE_DATA_WIDTH-1:0]	fifo_wr_data;

			//True if there is enough room in the FIFO for one entire packet
			assign fifo_space_available	= (fifo_wr_size >= CHILD_DATA_WIDTH[i*8 +: 8]);

			//Receiver pushes data directly to FIFO.
			//Ignore packet start/done signals, we only care about the data bus.
			//Receiver is proven to never send partial packets, so we can't lose sync!
			RPCv3RouterReceiver #(
				.IN_DATA_WIDTH(CHILD_DATA_WIDTH[i*8 +: 8]),
				.OUT_DATA_WIDTH(CORE_DATA_WIDTH)
			) rxvr (
				.clk(clk),
				.rpc_rx_en(child_rx_en[i]),
				.rpc_rx_data(child_rx_data[i*128 +: CHILD_DATA_WIDTH[i*8 +: 8] ] ),
				.rpc_rx_ready(child_rx_ready[i]),

				.rpc_fab_rx_space_available(fifo_space_available),
				.rpc_fab_rx_packet_start(),
				.rpc_fab_rx_data_valid(fifo_wr_en),
				.rpc_fab_rx_data(fifo_wr_data),
				.rpc_fab_rx_packet_done()
			);

			SingleClockShiftRegisterFifo #(
				.WIDTH(CORE_DATA_WIDTH),
				.DEPTH(32),
				.OUT_REG(1)
			) rx_fifo (

				.clk(clk),

				.wr(fifo_wr_en),
				.din(fifo_wr_data),

				.rd(child_fifo_rd[i]),
				.dout(child_fifo_dout[i*CORE_DATA_WIDTH +: CORE_DATA_WIDTH]),
				.overflow(),			//ignored, can never under/overflow b/c of receiver flow control
				.underflow(),
				.empty(child_fifo_empty[i]),
				.full(),
				.rsize(child_fifo_rsize[6*i +: 6]),
				.wsize(fifo_wr_size),

				.reset(1'b0)			//never reset the fifo
			);


		end

	endgenerate

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmitters for neighbor ports

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmitters for child ports

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual switch crossbar

	RPCv3RouterCrossbar #(
		.CORE_DATA_WIDTH(CORE_DATA_WIDTH),
		.CHILD_IS_TRUNK(CHILD_IS_TRUNK),
		.CHILD_COUNT(CHILD_COUNT),
		.CHILD_DATA_WIDTH(CHILD_DATA_WIDTH),
		.NEIGHBOR_PRESENT(NEIGHBOR_PRESENT),
		.NEIGHBOR_DATA_WIDTH(NEIGHBOR_DATA_WIDTH),
		.X_POS(X_POS),
		.Y_POS(Y_POS)
	) crossbar (
		.clk(clk),

		.rx_fifo_rd({child_fifo_rd, neighbor_fifo_rd}),
		.rx_fifo_empty({child_fifo_empty, neighbor_fifo_empty}),
		.rx_fifo_dout({child_fifo_dout, neighbor_fifo_dout}),
		.rx_fifo_rsize({child_fifo_rsize, neighbor_fifo_rsize})
	);

endmodule


