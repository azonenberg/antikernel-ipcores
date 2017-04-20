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

	RESOURCE USAGE (XST A7 rough estimate)
		Width				FF			LUT			Slice
		16 -> 32			38			12			14
		16 -> 64			70			10			21
		16 -> 128			133			5			37
		32 -> 64			69			4			20
		32 -> 128			132			48			24
		64 -> 128			131			27			30
 */
module RPCv3RouterReceiver_expanding
#(
	//Data width (must be one of 16, 32, 64, 128).
	parameter OUT_DATA_WIDTH = 32,
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

	//True if we are in the first cycle of an incoming message
	wire					rx_starting				= (rpc_rx_en && rpc_rx_ready);
	assign					rpc_fab_rx_packet_start	= rx_starting;

	//Position within the message (in IN_DATA_WIDTH-bit units)
	reg[2:0]				rx_count		= 0;

	//True if a receive is in progress
	wire					rx_active		= (rx_count != 0) || rx_starting;

	always @(posedge clk) begin

		rpc_fab_rx_data_valid			<= 0;
		rpc_fab_rx_packet_done			<= 0;

		//Process incoming data words
		if(rx_active) begin

			//Shift new data into the output register.
			//Leftmost word comes in first, so we shift from right to left
			rpc_fab_rx_data				<= { rpc_fab_rx_data[OUT_DATA_WIDTH-IN_DATA_WIDTH : 0], rpc_rx_data };

			//Update word count as we move through the message
			if(rx_starting)
				rx_count				<= 1;
			else
				rx_count				<= rx_count + 1'h1;

			//If we're at the final phase of this output word, report it
			if(rx_count[PHASE_BITS-1:0] == {PHASE_BITS{1'b1}})
				rpc_fab_rx_data_valid	<= 1'b1;

			//Stop at end of message
			//IN_DATA_WIDTH = 128 is illegal
			if( (IN_DATA_WIDTH == 64) && (rx_count == 1) ) begin
				rpc_fab_rx_packet_done	<= 1;
				rx_count				<= 0;
			end
			else if( (IN_DATA_WIDTH == 32) && (rx_count == 3) ) begin
				rpc_fab_rx_packet_done	<= 1;
				rx_count				<= 0;
			end
			else if( (IN_DATA_WIDTH == 16) && (rx_count == 7) ) begin
				rpc_fab_rx_packet_done	<= 1;
				rx_count				<= 0;
			end

		end

	end

	//Ready to receive if the fabric side is ready.
	//Once we go ready, go un-ready when a message comes in.
	reg		rpc_rx_ready_ff	= 0;
	always @(posedge clk) begin
		if(rpc_rx_en)
			rpc_rx_ready_ff		<= 0;
		if(rpc_fab_rx_space_available)
			rpc_rx_ready_ff		<= 1;
	end

	always @(*) begin
		rpc_rx_ready			<= rpc_rx_ready_ff;
	end

endmodule
