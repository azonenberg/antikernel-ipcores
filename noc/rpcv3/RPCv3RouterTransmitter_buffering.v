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
	@brief Transmitter for RPC network, protocol version 3

	This module expects OUT_DATA_WIDTH to be equal to IN_DATA_WIDTH.

	Network-side interface is standard RPCv3.

	Router-side interface is a FIFO.
		fifo_size			Number of IN_DATA_WIDTH-bit words left in transmit FIFO.
							FIFO is a fixed depth of 32 words regardless of data width, so capacity ranges from
							4 to 32 messages depending on data width.
		packet_start		Asserted by router concurrently with first wr_en of a message
		wr_en				Asserted by router for (128 / IN_DATA_WIDTH) consecutive cycles
							to indicate wr_data is valid
		wr_data				Data to be sent
		packet_done			Asserted by transmitter as the last cycle of the message is sent

	RESOURCE USAGE (XST A7 rough estimate)
		Width				FF			LUT			Slice
		16
		32
		64
		128
 */
module RPCv3RouterTransmitter_buffering
#(
	//Data width (must be one of 16, 32, 64, 128).
	parameter OUT_DATA_WIDTH = 32,
	parameter IN_DATA_WIDTH = 32
)
(
	//Interface clock
	input wire clk,

	//Network interface, outbound side
	output reg						rpc_tx_en				= 0,
	output reg[OUT_DATA_WIDTH-1:0]	rpc_tx_data				= 0,
	input wire						rpc_tx_ready,

	//Router interface, inbound side
	output wire[5:0]				rpc_fab_tx_fifo_size,
	input wire						rpc_fab_tx_packet_start,
	input wire						rpc_fab_tx_wr_en,
	input wire[IN_DATA_WIDTH-1:0]	rpc_fab_tx_wr_data,
	output reg						rpc_fab_tx_packet_done	= 0
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
				$display("ERROR: RPCv3RouterTransmitter_buffering IN_DATA_WIDTH must be 16/32/64/128");
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
				$display("ERROR: RPCv3RouterTransmitter_buffering OUT_DATA_WIDTH must be 16/32/64/128");
				$finish;
			end

		endcase

		if(IN_DATA_WIDTH != OUT_DATA_WIDTH) begin
			$display("ERROR: RPCv3RouterTransmitter_buffering IN_DATA_WIDTH must be equal to OUT_DATA_WIDTH");
			$finish;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Compute some useful values

	//Number of clocks it takes to send/receive a message
	localparam MESSAGE_CYCLES = 128 / IN_DATA_WIDTH;
	localparam MESSAGE_MAX = MESSAGE_CYCLES - 1;

	//Number of bits we need in the cycle counter
	`include "../../synth_helpers/clog2.vh"
	localparam CYCLE_BITS = clog2(MESSAGE_CYCLES);
	localparam CYCLE_MAX = CYCLE_BITS ? CYCLE_BITS-1 : 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual outbound data FIFO

	reg						fifo_rdata_valid		= 0;
	reg						fifo_rd					= 0;
	reg						fifo_rd_ff				= 0;
	wire					fifo_empty;

	wire					fifo_rdata_packet_start;
	wire[IN_DATA_WIDTH-1:0]	fifo_rdata_data;

	wire					unused_overflow;
	wire					unused_underflow;
	wire					unused_full;
	wire[5:0]				unused_rsize;

	SingleClockShiftRegisterFifo #(
		.WIDTH(IN_DATA_WIDTH + 1),
		.DEPTH(32),
		.OUT_REG(1)
	) tx_fifo (
		.clk(clk),
		.wr(rpc_fab_tx_wr_en),
		.din({rpc_fab_tx_packet_start, rpc_fab_tx_wr_data}),

		.rd(fifo_rd),
		.dout({fifo_rdata_packet_start, fifo_rdata_data}),
		.overflow(unused_overflow),
		.underflow(unused_underflow),
		.empty(fifo_empty),
		.full(unused_full),
		.rsize(unused_rsize),
		.wsize(rpc_fab_tx_fifo_size),

		.reset(1'b0)		//never reset the fifo
	);

	always @(posedge clk) begin
		fifo_rd_ff		<= fifo_rd;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	//If we couldn't send the first cycle, remember that we have a send queued up
	//reg			tx_pending			= 0;

	//True if we want to send (either current or queued transmit request)
	//wire		tx_request			= tx_pending || rpc_fab_tx_en;

	//True if we're starting a transmit this cycle (whether queued or fresh)
	//wire		tx_starting			= tx_request && rpc_tx_ready;

	//Position within the message (in DATA_WIDTH-bit units)
	reg[CYCLE_MAX:0] tx_count		= 0;

	//True if a transmit is in progress
	//wire		tx_active			= (tx_count != 0) || tx_starting;

	//If we have data ready to read, and the bus isn't otherwise occupied, read it
	always @(*) begin
		fifo_rd	<= (!fifo_empty && rpc_tx_ready);
	end

	always @(posedge clk) begin

		//Since input and output data width are equal, we can begin sending as soon as there's any data in the fifo

		/*
		//One little bit of stateful logic, though :)
		always @(posedge clk) begin

			//Clear pending messages once sent
			if(tx_starting)
				tx_pending	<= 0;

			//If we try to send when rx isn't ready, save it until they are
			//Send requests during a transmit cycle are ignored.
			if(rpc_fab_tx_en && !rpc_tx_ready && !tx_active)
				tx_pending	<= 1;

		end
		*/

	end

endmodule
