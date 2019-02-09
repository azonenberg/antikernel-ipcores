`default_nettype none
`timescale 1ns/1ps

/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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
	@author	Andrew D. Zonenberg
	@brief	FIFO buffer using handshaking of pointers across clock domains.

	TODO: maybe replace with gray code version for lower latency?
	Getting size values is harder that way though.
 */
module CrossClockFifo #(
	parameter WIDTH			= 16,
	parameter DEPTH			= 16,
	parameter USE_BLOCK		= 0,
	parameter OUT_REG		= 1,

	localparam ADDR_BITS	= $clog2(DEPTH)
) (
	input wire					wr_clk,
	input wire					wr_en,
	input wire[WIDTH-1:0]		wr_data,
	output wire[ADDR_BITS:0]	wr_size,
	output wire					wr_full,
	output logic				wr_overflow		= 0,

	input wire					rd_clk,
	input wire					rd_en,
	output wire[WIDTH-1:0]		rd_data,
	output wire[ADDR_BITS:0]	rd_size,
	output wire					rd_empty,
	output logic				rd_underflow	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The memory itself

	wire[ADDR_BITS-1:0]	wr_memptr;
	wire[ADDR_BITS-1:0]	rd_memptr;

	MemoryMacro #(
		.WIDTH(WIDTH),
		.DEPTH(DEPTH),
		.USE_BLOCK(USE_BLOCK),
		.OUT_REG(OUT_REG),
		.DUAL_PORT(1),
		.TRUE_DUAL(0)
	) fifomem (
		.porta_clk(wr_clk),
		.porta_en(wr_en),
		.porta_addr(wr_memptr),
		.porta_we(wr_en),
		.porta_din(wr_data),
		.porta_dout(),

		.portb_clk(rd_clk),
		.portb_en(rd_en),
		.portb_addr(rd_memptr),
		.portb_we(1'b0),
		.portb_din({WIDTH{1'b0}}),
		.portb_dout(rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pointer synchronization

	logic[ADDR_BITS:0]		wr_ptr	= 0;
	assign					wr_memptr = wr_ptr[ADDR_BITS-1:0];

	logic[ADDR_BITS:0]		rd_ptr	= 0;
	assign					rd_memptr = rd_ptr[ADDR_BITS-1:0];

	wire[ADDR_BITS:0]		rd_wr_ptr;	//write pointer as seen in the read domain

	logic					wr_ptr_update	= 0;
	wire					wr_ptr_ack;

	RegisterSynchronizer #(
		.WIDTH(ADDR_BITS+1)
	) sync_wr_ptr (
		.clk_a(wr_clk),
		.en_a(wr_ptr_update),
		.ack_a(wr_ptr_ack),
		.reg_a(wr_ptr),
		.reset_a(1'b0),

		.clk_b(rd_clk),
		.updated_b(),
		.reg_b(rd_wr_ptr)
	);

	wire[ADDR_BITS:0]		wr_rd_ptr;	//read pointer as seen in the write domain

	logic					rd_ptr_update	= 0;
	wire					rd_ptr_ack;

	RegisterSynchronizer #(
		.WIDTH(ADDR_BITS+1)
	) sync_rd_ptr (
		.clk_a(rd_clk),
		.en_a(rd_ptr_update),
		.ack_a(rd_ptr_ack),
		.reg_a(rd_ptr),
		.reset_a(1'b0),

		.clk_b(wr_clk),
		.updated_b(),
		.reg_b(wr_rd_ptr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write logic

	assign	wr_full = (wr_rd_ptr + DEPTH) == wr_ptr;
	assign	wr_size = DEPTH + wr_rd_ptr - wr_ptr;

	logic	wr_sync_busy	= 0;

	always_ff @(posedge wr_clk) begin

		wr_ptr_update	<= 0;

		//Check overflow flag
		wr_overflow		<= wr_full && wr_en;

		if(wr_en && !wr_full)
			wr_ptr		<= wr_ptr + 1'h1;

		//Push registers between clock domains
		if(!wr_sync_busy) begin
			wr_ptr_update	<= 1;
			wr_sync_busy	<= 1;
		end
		if(wr_ptr_ack)
			wr_sync_busy	<= 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read logic

	assign	rd_empty = (rd_wr_ptr == rd_ptr);
	assign	rd_size	= rd_wr_ptr - rd_ptr;

	logic	rd_sync_busy	= 0;

	always_ff @(posedge rd_clk) begin

		rd_ptr_update	<= 0;

		//Check overflow flag
		rd_underflow	<= rd_empty && rd_en;

		if(rd_en && !rd_empty)
			rd_ptr		<= rd_ptr + 1'h1;

		//Push registers between clock domains
		if(!rd_sync_busy) begin
			rd_ptr_update	<= 1;
			rd_sync_busy	<= 1;
		end
		if(rd_ptr_ack)
			rd_sync_busy	<= 0;

	end

endmodule
