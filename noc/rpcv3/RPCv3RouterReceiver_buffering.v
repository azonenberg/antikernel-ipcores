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

	This module expects OUT_DATA_WIDTH to be equal to IN_DATA_WIDTH.

	Network-side interface is standard RPCv3

	Router-side interface is a FIFO.
		space_available					Asserted by router if it has at least one *packet* worth of buffer space.
		packet_start					Asserted by transceiver for one clock at start of message.
										Asserted concurrently with first assertion of data_valid.
		data_valid						Asserted by transceiver if data should be processed.
										Will be asserted constantly between packet_start and packet_done.
		data							One word of message data.
		packet_done						Asserted by transceiver for one clock at end of message.
										Concurrent with last assertion of data_valid.

	RESOURCE USAGE (XST A7 rough estimate)
		Width				FF			LUT			Slice
		16					22			7			7
		32					37			5			10
		64					68			3			20
		128					131			30			28
 */
module RPCv3RouterReceiver_buffering
#(
	//Data width (must be one of 16, 32, 64, 128).
	parameter OUT_DATA_WIDTH = 16,
	parameter IN_DATA_WIDTH = 16
)
(
	//Interface clock
	input wire clk,

	//Network interface, inbound side
	input wire						rpc_rx_en,
	input wire[IN_DATA_WIDTH-1:0]	rpc_rx_data,
	output reg						rpc_rx_ready = 0,

	//Router interface, outbound side
	input wire						rpc_fab_rx_space_available,
	output wire						rpc_fab_rx_packet_start,
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

			128: begin
			end

			default: begin
				$display("ERROR: RPCv3RouterReceiver_buffering IN_DATA_WIDTH must be 16/32/64/128");
				$finish;
			end

		endcase

		case(OUT_DATA_WIDTH)

			16: begin
			end

			32: begin
			end

			64: begin
			end

			128: begin
			end

			default: begin
				$display("ERROR: RPCv3RouterReceiver_buffering OUT_DATA_WIDTH must be 16/32/64/128");
				$finish;
			end

		endcase

		if(IN_DATA_WIDTH != OUT_DATA_WIDTH) begin
			$display("ERROR: RPCv3RouterReceiver_buffering IN_DATA_WIDTH must be equal to OUT_DATA_WIDTH");
			$finish;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Compute some useful values

	//Number of clocks it takes to receive a message
	localparam MESSAGE_CYCLES = 128 / IN_DATA_WIDTH;
	localparam MESSAGE_MAX = MESSAGE_CYCLES - 1;

	//Number of bits we need in the cycle counter
	`include "../../synth_helpers/clog2.vh"
	localparam CYCLE_BITS = clog2(MESSAGE_CYCLES);
	localparam CYCLE_MAX = CYCLE_BITS ? CYCLE_BITS-1 : 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main RX logic

	//True if we are in the first cycle of an incoming message
	wire					rx_starting				= (rpc_rx_en && rpc_rx_ready);
	assign					rpc_fab_rx_packet_start	= rx_starting;

	//Position within the message (in IN_DATA_WIDTH-bit units)
	reg[2:0]				rx_count		= 0;

	//True if a receive is in progress
	wire					rx_active		= (rx_count != 0) || rx_starting;

	//Register the output because our formal testbench doesn't know how to handle a zero-cycle-delay receiver.
	//TODO: improve testbench to handle this better and cut our area down by a few FFs
	always @(posedge clk) begin
		rpc_fab_rx_data_valid			<= rx_active;
		rpc_fab_rx_packet_done			<= (rx_count == MESSAGE_MAX);

		rpc_fab_rx_data					<= rpc_rx_data;
	end

	always @(posedge clk) begin

		//Process incoming data words
		if(rx_active) begin

			//Update word count as we move through the message
			if(rx_starting)
				rx_count				<= 1;
			else
				rx_count				<= rx_count + 1'h1;

			//When we hit the end of the message, stop
			if(rx_count == MESSAGE_MAX)
				rx_count				<= 0;

		end

	end

	//Ready to receive if the fabric side is ready.
	//Once we go ready, go un-ready when a message comes in.
	reg		rpc_rx_ready_ff	= 0;
	always @(posedge clk) begin
		if(rpc_rx_en)
			rpc_rx_ready_ff		<= 0;
		if(rpc_fab_rx_space_available && !rx_active)
			rpc_rx_ready_ff		<= 1;
	end

	always @(*) begin
		rpc_rx_ready			<= rpc_rx_ready_ff;
	end

endmodule
