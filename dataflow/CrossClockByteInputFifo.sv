`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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
	@brief CrossClockFifo wrapper for byte oriented input
 */
module CrossClockByteInputFifo #(
	parameter DEPTH			= 512,
	parameter USE_BLOCK		= 1,
	parameter OUT_REG		= 1,

	localparam ADDR_BITS	= $clog2(DEPTH)
)(
	input wire					wr_clk,
	input wire					wr_en,
	input wire[31:0]			wr_data,
	input wire[2:0]				wr_bytes_valid,	//1-4
	input wire					wr_flush,		//push din_temp into the fifo
												//(must not be same cycle as wr)
	input wire					rd_clk,
	input wire					rd_en,
	output wire[31:0]			rd_data,
	output wire[2:0]			rd_bytes_valid,
	output wire[ADDR_BITS:0]	rd_size,
	output wire					rd_empty,
	output wire					rd_underflow
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input stage

	wire		push_en;
	wire[2:0]	push_bytes_valid;
	wire[31:0]	push_data;

	ByteToWordConverter converter(
		.clk(wr_clk),
		.wr(wr_en),
		.din(wr_data),
		.bytes_valid(wr_bytes_valid),
		.flush(wr_flush),
		.reset(1'b0),

		.dout_valid(push_en),
		.dout_bytes_valid(push_bytes_valid),
		.dout(push_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual FIFO

	CrossClockFifo #(
		.WIDTH(35),
		.DEPTH(DEPTH),
		.USE_BLOCK(USE_BLOCK),
		.OUT_REG(OUT_REG)
	) fifo (
		.wr_clk(wr_clk),
		.wr_en(push_en),
		.wr_data({push_bytes_valid, push_data}),
		.wr_size(),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(1'b0),

		.rd_clk(rd_clk),
		.rd_en(rd_en),
		.rd_data({rd_bytes_valid, rd_data}),
		.rd_size(rd_size),
		.rd_empty(rd_empty),
		.rd_underflow(rd_underflow)
	);

endmodule
