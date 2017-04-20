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

	This module expects OUT_DATA_WIDTH to be greater than IN_DATA_WIDTH.

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
 */
module RPCv3RouterReceiver_expanding
#(
	//Data width (must be one of 16, 32, 64, 128).
	parameter OUT_DATA_WIDTH = 32,
	parameter IN_DATA_WIDTH = 16,
)
(
	//Interface clock
	input wire clk;

	//Network interface, inbound side
	input wire						rpc_rx_en,
	input wire[IN_DATA_WIDTH-1:0]	rpc_rx_data,
	output reg						rpc_rx_ready,

	//Router interface, outbound side
	input wire						rpc_fab_rx_space_available,
	output reg						rpc_fab_rx_packet_start = 0,
	output reg						rpc_fab_rx_data_valid	= 0,
	output reg[OUT_DATA_WIDTH-1:0]	rpc_fab_rx_data			= 0,
	output reg						rpc_fab_rx_packet_done	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synthesis-time sanity checking

	initial begin

		case(IN_DATA_WIDTH)
			16: begin
			end

			32: begin
			end

			64: begin
			end

			default: begin
				$display("ERROR: RPCv3RouterReceiver_expanding IN_DATA_WIDTH must be 16/32/64");
				$finish;
			end

		endcase

		case(OUT_DATA_WIDTH)

			32: begin
			end

			64: begin
			end

			128: begin
			end

			default: begin
				$display("ERROR: RPCv3RouterReceiver_expanding OUT_DATA_WIDTH must be 32/64/128");
				$finish;
			end

		endcase

		if(IN_DATA_WIDTH >= OUT_DATA_WIDTH) begin
			$display("ERROR: RPCv3RouterReceiver_expanding IN_DATA_WIDTH must be less than OUT_DATA_WIDTH");
			$finish;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Compute some useful values

	//Number of clocks it takes to receive a message
	localparam MESSAGE_CYCLES = 128 / IN_DATA_WIDTH;

	//Number of bits we need in the cycle counter
	`include "../../synth_helpers/clog2.vh"
	localparam CYCLE_BITS = clog2(MESSAGE_CYCLES);
	localparam CYCLE_MAX = CYCLE_BITS ? CYCLE_BITS-1 : 0;

	//Calculate the expansion ratio (number of input words per output word)
	//Always 2, 4, or 8
	localparam EXPANSION_RATIO = OUT_DATA_WIDTH / IN_DATA_WIDTH;
	localparam PHASE_BITS = clog2(EXPANSION_RATIO);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main RX logic

	always @(posedge clk) begin



	end

endmodule


