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

	This module expects OUT_DATA_WIDTH to be less than IN_DATA_WIDTH.

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
		32 -> 16
		64 -> 16
		128 -> 16
		64 -> 32
		128 -> 32
		128 -> 64
 */
module RPCv3RouterReceiver_collapsing
#(
	//Data width (must be one of 16, 32, 64, 128).
	parameter OUT_DATA_WIDTH = 16,
	parameter IN_DATA_WIDTH = 32
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

			32: begin
			end

			64: begin
			end

			128: begin
			end

			default: begin
				$display("ERROR: RPCv3RouterReceiver_collapsing IN_DATA_WIDTH must be 16/32/64/128");
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

			default: begin
				$display("ERROR: RPCv3RouterReceiver_collapsing OUT_DATA_WIDTH must be 16/32/64");
				$finish;
			end

		endcase

		if(IN_DATA_WIDTH <= OUT_DATA_WIDTH) begin
			$display("ERROR: RPCv3RouterReceiver_collapsing IN_DATA_WIDTH must be greater than OUT_DATA_WIDTH");
			$finish;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Compute some useful values

	//Number of clocks it takes to receive a message
	localparam MESSAGE_CYCLES = 128 / IN_DATA_WIDTH;
	localparam MESSAGE_MAX = MESSAGE_CYCLES - 1;

	//Number of clocks it takes to re-send a message
	localparam OUT_CYCLES = 128 / OUT_DATA_WIDTH;

	//Number of bits we need in the cycle counter
	`include "../../synth_helpers/clog2.vh"
	localparam CYCLE_BITS = clog2(MESSAGE_CYCLES);
	localparam CYCLE_MAX = CYCLE_BITS ? CYCLE_BITS-1 : 0;

	localparam OUT_CYCLE_BITS = clog2(OUT_CYCLES);
	localparam OUT_CYCLE_MAX = OUT_CYCLE_BITS ? OUT_CYCLE_BITS-1 : 0;

	//Calculate the collapsing ratio (number of output words per input word)
	//Always 2, 4, or 8
	localparam COLLAPSE_RATIO = IN_DATA_WIDTH / OUT_DATA_WIDTH;
	localparam COLLAPSE_MAX = COLLAPSE_RATIO - 1;
	localparam COLLAPSE_BITS = clog2(COLLAPSE_RATIO);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO of data being received

	wire						fifo_wr;
	reg							fifo_rd		= 0;

	wire						fifo_empty;

	wire[IN_DATA_WIDTH-1:0]		fifo_dout;

	wire						unused_full;
	wire						unused_underflow;
	wire						unused_overflow;
	wire[CYCLE_BITS:0]			unused_rsize;
	wire[CYCLE_BITS:0]			unused_wsize;

	SingleClockShiftRegisterFifo #(
		.WIDTH(IN_DATA_WIDTH),
		.DEPTH(MESSAGE_CYCLES),
		.OUT_REG(1)
	) rx_fifo (
		.clk(clk),
		.wr(fifo_wr),
		.din(rpc_rx_data),

		.rd(fifo_rd),
		.dout(fifo_dout),
		.overflow(unused_overflow),
		.underflow(unused_underflow),
		.empty(fifo_empty),
		.full(unused_full),
		.rsize(unused_rsize),
		.wsize(unused_wsize),

		.reset(1'b0)		//never reset the fifo
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main RX logic

	//True if we are in the first cycle of an incoming message
	wire					rx_starting				= (rpc_rx_en && rpc_rx_ready);
	assign					rpc_fab_rx_packet_start	= rx_starting;

	//Position within the message (in IN_DATA_WIDTH-bit units)
	reg[2:0]				rx_count		= 0;

	//True if a receive is in progress
	wire					rx_active		= (rx_count != 0) || rx_starting;
	assign 					fifo_wr			= rx_active;

	//True if fifo_dout contains a valid data word
	reg						fifo_dout_valid	= 0;

	//Position within the output word (in OUT_DATA_WIDTH-bit units)
	reg[COLLAPSE_BITS-1:0]	out_pos		= 0;

	//True if a transmit (to our host) is active
	wire					tx_active		= (fifo_dout_valid || !fifo_empty);

	//True if we are currently at the last word in fifo_dout
	wire					last_word_in_buffer = (out_pos == COLLAPSE_MAX);

	//If we have data in the FIFO, and our current word is missing or done, go read another one
	always @(*) begin
		fifo_rd							<= !fifo_empty && (!fifo_dout_valid || last_word_in_buffer);
	end

	//Combinatorial muxing of the output to save a bit of time
	always @(*) begin
		rpc_fab_rx_data_valid			<= fifo_dout_valid;
		rpc_fab_rx_packet_done			<= fifo_empty && last_word_in_buffer;

		for(i=0; i<COLLAPSE_RATIO; i=i+1) begin
			if(i == out_pos)
				rpc_fab_rx_data		<= fifo_dout[OUT_DATA_WIDTH*(COLLAPSE_MAX - i) +: OUT_DATA_WIDTH];
		end
	end

	integer i;
	always @(posedge clk) begin

		//Update status flags as we read data from the FIFO
		if(fifo_rd)
			fifo_dout_valid				<= 1;

		//Keep track of position in the output word
		if(fifo_dout_valid)
			out_pos						<= out_pos + 1'h1;

		//Clear state on the last word
		if(rpc_fab_rx_packet_done) begin
			rx_count					<= 0;
			fifo_dout_valid				<= 0;
			out_pos						<= 0;
		end

		//Process incoming data words
		if(rx_active) begin

			//Clear some status flags at the start of a new message
			if(rx_starting) begin
				out_pos					<= 0;
				fifo_dout_valid			<= 0;
			end

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
		if(rpc_fab_rx_space_available && !rx_active && !tx_active)
			rpc_rx_ready_ff		<= 1;
	end

	always @(*) begin
		rpc_rx_ready			<= rpc_rx_ready_ff;
	end

endmodule
