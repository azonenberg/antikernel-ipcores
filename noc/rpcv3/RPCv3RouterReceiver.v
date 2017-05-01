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
	@brief Receiver for RPC network, protocol version 3

	Network-side interface is standard RPCv3

	Router-side interface is a FIFO.
		space_available					Asserted by router if it has at least one *packet* worth of buffer space.
		packet_start					Asserted by transceiver for one clock at start of message.
										Asserted before first assertion of data_valid.
		data_valid						Asserted by transceiver if data should be processed.
										Will be asserted for one clock every (OUT_DATA_WIDTH / IN_DATA_WIDTH) clocks.
		data							One word of message data.
		packet_done						Asserted by transceiver for one clock at end of message.
										Concurrent with last assertion of data_valid.

	RESOURCE USAGE (XST A7 rough estimate)
		Width				FF			LUT			Slice
		16 -> 16			22			7			7
		16 -> 32			38			12			14
		16 -> 64			70			10			21
		16 -> 128			133			5			37
		32 -> 16			40			76			43
		32 -> 32			37			5			10
		32 -> 64			69			4			20
		32 -> 128			132			48			24
		64 -> 16			72			154			73
		64 -> 32			71			89			39
		64 -> 64			68			3			20
		64 -> 128			131			27			30
		128 -> 16			136			104			36
		128 -> 32			134			102			49
		128 -> 64			132			164			68
		128 -> 128			131			30			28
 */
module RPCv3RouterReceiver
#(
	parameter OUT_DATA_WIDTH = 32,
	parameter IN_DATA_WIDTH = 16
)
(
	//Interface clock
	input wire clk,

	//Network interface, inbound side
	input wire						rpc_rx_en,
	input wire[IN_DATA_WIDTH-1:0]	rpc_rx_data,
	output wire						rpc_rx_ready,

	//Router interface, outbound side
	input wire						rpc_fab_rx_space_available,
	output wire						rpc_fab_rx_packet_start,
	output wire						rpc_fab_rx_data_valid,
	output wire[OUT_DATA_WIDTH-1:0]	rpc_fab_rx_data,
	output wire						rpc_fab_rx_packet_done
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Create the right receiver depending on link widths

	localparam EXPANDING = (IN_DATA_WIDTH < OUT_DATA_WIDTH);
	localparam COLLAPSING = (IN_DATA_WIDTH > OUT_DATA_WIDTH);
	localparam BUFFERING = (IN_DATA_WIDTH == OUT_DATA_WIDTH);

	generate

		if(EXPANDING) begin
			RPCv3RouterReceiver_expanding #(
				.IN_DATA_WIDTH(IN_DATA_WIDTH),
				.OUT_DATA_WIDTH(OUT_DATA_WIDTH)
			) rxvr (
				.clk(clk),

				.rpc_rx_en(rpc_rx_en),
				.rpc_rx_data(rpc_rx_data),
				.rpc_rx_ready(rpc_rx_ready),

				.rpc_fab_rx_space_available(rpc_fab_rx_ready),
				.rpc_fab_rx_packet_start(rpc_fab_rx_packet_start),
				.rpc_fab_rx_data_valid(rpc_fab_rx_data_valid),
				.rpc_fab_rx_data(rpc_fab_rx_data),
				.rpc_fab_rx_packet_done(rpc_fab_rx_packet_done)
			);
		end

		else if(COLLAPSING) begin
			RPCv3RouterReceiver_collapsing #(
				.IN_DATA_WIDTH(IN_DATA_WIDTH),
				.OUT_DATA_WIDTH(OUT_DATA_WIDTH)
			) rxvr (
				.clk(clk),

				.rpc_rx_en(rpc_rx_en),
				.rpc_rx_data(rpc_rx_data),
				.rpc_rx_ready(rpc_rx_ready),

				.rpc_fab_rx_space_available(rpc_fab_rx_ready),
				.rpc_fab_rx_packet_start(rpc_fab_rx_packet_start),
				.rpc_fab_rx_data_valid(rpc_fab_rx_data_valid),
				.rpc_fab_rx_data(rpc_fab_rx_data),
				.rpc_fab_rx_packet_done(rpc_fab_rx_packet_done)
			);
		end

		else begin
			RPCv3RouterReceiver_buffering #(
				.IN_DATA_WIDTH(IN_DATA_WIDTH),
				.OUT_DATA_WIDTH(OUT_DATA_WIDTH)
			) rxvr (
				.clk(clk),

				.rpc_rx_en(rpc_rx_en),
				.rpc_rx_data(rpc_rx_data),
				.rpc_rx_ready(rpc_rx_ready),

				.rpc_fab_rx_space_available(rpc_fab_rx_ready),
				.rpc_fab_rx_packet_start(rpc_fab_rx_packet_start),
				.rpc_fab_rx_data_valid(rpc_fab_rx_data_valid),
				.rpc_fab_rx_data(rpc_fab_rx_data),
				.rpc_fab_rx_packet_done(rpc_fab_rx_packet_done)
			);
		end

	endgenerate

endmodule
