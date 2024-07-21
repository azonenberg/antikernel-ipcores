`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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
	@brief A cross-clock FIFO with registered outputs intended for storing packetized data.

	Packets are pushed in one word at a time but can be read random-access at the other side and popped as a unit.

	Note that the FIFO does not actually store any metadata about how large the packets are. This information must be
	transmitted in-band or using a separate FIFO.

	Committing the same cycle as the final write of a packet is legal.
 */
module CrossClockPacketFifo #(
	parameter WIDTH 	= 32,
	parameter DEPTH		= 11'd1024,

	parameter ADDR_BITS = $clog2(DEPTH),

	parameter USE_BLOCK	= 1,
	parameter OUT_REG	= 1
)(
	//WRITE port (all signals in wr_clk domain)
	input wire					wr_clk,			//Clock for write port
	input wire					wr_en,			//Assert wr_en and put data on wr_data to push
	input wire[WIDTH-1:0]		wr_data,
	input wire					wr_reset,		//Reset write side of the FIFO
	output wire[ADDR_BITS:0]	wr_size,		//needs to be one bigger than pointers to hold fully empty size
	input wire					wr_commit,		//Assert this to say "everything we've written is legal to read"
	input wire					wr_rollback,	//Assert this to discard everything pushed since the last commit

	//READ port (all signals in rd_clk domain)
	input wire					rd_clk,
	input wire					rd_en,			//Read a word
	input wire[ADDR_BITS-1:0]	rd_offset,		//Offset from packet start for random access reads
	input wire					rd_pop_single,	//Pop one word (exclusive with rd_pop_packet)
	input wire					rd_pop_packet,	//Pop an entire packet (exclusive with rd_pop_single)
	input wire[ADDR_BITS:0]		rd_packet_size,	//Size of the packet to pop
	output wire[WIDTH-1:0]		rd_data,
	output wire[ADDR_BITS:0]	rd_size,
	input wire					rd_reset
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO pointers

	logic[ADDR_BITS:0] 	data_wptr		= 0;							//extra bit for empty/full detect
	wire[ADDR_BITS-1:0] data_wptr_low	= data_wptr[ADDR_BITS-1:0];		//actual pointer

	logic[ADDR_BITS:0] data_rptr		= 0;							//extra bit for empty/full detect
	wire[ADDR_BITS-1:0] data_rptr_off	= data_rptr[ADDR_BITS-1:0] + rd_offset;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual memory block

	MemoryMacro #(
		.WIDTH(WIDTH),
		.DEPTH(DEPTH),
		.USE_BLOCK(USE_BLOCK),
		.OUT_REG(OUT_REG),
		.DUAL_PORT(1),
		.TRUE_DUAL(0),
		.PORTA_WRONLY(1),	//remove read logic on port A
		.INIT_VALUE({WIDTH{1'h0}})
	) data_mem (
		.porta_clk(wr_clk),
		.porta_en(wr_en),
		.porta_addr(data_wptr_low[ADDR_BITS-1:0]),
		.porta_we(wr_en),
		.porta_din(wr_data),
		.porta_dout(),

		.portb_clk(rd_clk),
		.portb_en(rd_en),
		.portb_addr(data_rptr_off),
		.portb_we(1'b0),
		.portb_din({WIDTH{1'h0}}),
		.portb_dout(rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write logic (input clock domain)

	logic[ADDR_BITS:0]	data_wptr_committed	= 0;

	always_ff @(posedge wr_clk) begin

		if(wr_en)
			data_wptr 			= data_wptr + 1'h1;

		//commit/rollback have higher precedence than writes
		//but lower than resets
		if(wr_commit)
			data_wptr_committed	<= data_wptr;
		if(wr_rollback)
			data_wptr			= data_wptr_committed;

		if(wr_reset)
			data_wptr 			= 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read logic (output clock domain)

	logic	rptr_updated	= 0;

	//Pointer manipulation
	always_ff @(posedge rd_clk) begin

		rptr_updated		<= 0;

		if(rd_pop_single) begin
			if(rd_size != 0) begin
				data_rptr		<= data_rptr + 1'h1;
				rptr_updated	<= 1;
			end
		end
		else if(rd_pop_packet) begin
			data_rptr 		<= data_rptr + rd_packet_size;
			rptr_updated	<= 1;
		end

		if(rd_reset) begin
			data_rptr 		<= 0;
			rptr_updated	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tail pointer synchronization from input to output clock domain

	//Tail pointer
	//This is the lowest address to which writes are guaranteed to have committed.
	wire[ADDR_BITS:0] data_tail;
	assign rd_size = data_tail - data_rptr;

	RegisterSynchronizer #(
		.WIDTH(ADDR_BITS+1),
		.INIT({ADDR_BITS+1{1'h0}})
	) sync_tail (
		.clk_a(wr_clk),
		.en_a(wr_commit),
		.ack_a(),
		.reg_a(data_wptr),

		.clk_b(rd_clk),
		.updated_b(),
		.reg_b(data_tail),
		.reset_b(rd_reset)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Head pointer synchronization from output to input clock domain

	//Head pointer
	//This is the highest address from which reads are guaranteed to have committed
	wire[ADDR_BITS:0] data_head;
	assign wr_size = DEPTH + data_head - data_wptr;

	RegisterSynchronizer #(
		.WIDTH(ADDR_BITS+1),
		.INIT({ADDR_BITS+1{1'h0}})
	) sync_head (
		.clk_a(rd_clk),
		.en_a(rptr_updated),
		.ack_a(),
		.reg_a(data_rptr),

		.clk_b(wr_clk),
		.updated_b(),
		.reg_b(data_head),
		.reset_b(rd_reset)
	);

endmodule
