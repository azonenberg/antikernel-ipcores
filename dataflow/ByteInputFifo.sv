`timescale 1ns / 1ps
`default_nettype none
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
	@author Andrew D. Zonenberg
	@brief Wrapper around SingleClockFifo for allowing arbitrary sized input (1-4 bytes per clock)
 */
module ByteInputFifo #(
	parameter DEPTH 		= 512,
	localparam ADDR_BITS 	= $clog2(DEPTH),
	parameter USE_BLOCK 	= 1,
	parameter OUT_REG 		= 1
)(
	input wire					clk,
	input wire					wr,
	input wire[31:0]			din,
	input wire[2:0]				bytes_valid,	//1-4
	input wire					flush,			//push din_temp into the fifo
												//(must not be same cycle as wr)

	input wire					rd,
	output wire[31:0]			dout,

	output logic				overflow = 0,
	output logic				underflow = 0,

	output wire					empty,
	output wire					full,

	output wire[ADDR_BITS:0]	rsize,
	output wire[ADDR_BITS:0]	wsize,

	input wire					reset
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input logic

	logic[23:0]					din_temp	= 0;
	logic[1:0]					temp_valid	= 0;

	logic[55:0]					din_merged;
	logic[2:0]					din_merged_valid;

	logic						fifo_wr		= 0;
	logic[31:0]					fifo_din	= 0;

	always_comb begin

		if(wr) begin

			din_merged_valid	= temp_valid + bytes_valid;

			case(temp_valid)
				0: din_merged	= { din, 24'h0 };
				1: din_merged	= { din_temp[23:16], din, 16'h0 };
				2: din_merged	= { din_temp[23:8], din, 8'h0 };
				3: din_merged	= { din_temp[23:0], din };
			endcase

		end

		else begin
			din_merged_valid	= temp_valid;

			case(temp_valid)
				0: din_merged	= { 56'h0 };
				1: din_merged	= { din_temp[23:16], 48'h0 };
				2: din_merged	= { din_temp[23:8], 40'h0 };
				3: din_merged	= { din_temp[23:0], 32'h0 };
			endcase
		end

	end

	always_ff @(posedge clk) begin
		fifo_wr			<= 0;

		//We have 4 bytes, push them into the fifo
		if(din_merged_valid >= 4) begin
			fifo_wr		<= 1;
			fifo_din	<= din_merged[55:24];

			temp_valid	<= din_merged_valid - 4;
			din_temp	<= din_merged[23:0];
		end

		//Not enough, just save the existing stuff
		else begin
			din_temp	<= din_merged[55:32];
			temp_valid	<= din_merged_valid;
		end

		//Handle flushing
		if(flush) begin
			fifo_wr		<= 1;
			fifo_din	<= { din_temp, 8'h0 };
			din_temp	<= 0;
			temp_valid	<= 0;
		end

		if(reset) begin
			din_temp	<= 0;
			temp_valid	<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual FIFO

	SingleClockFifo #(
		.WIDTH(32),
		.DEPTH(DEPTH),
		.USE_BLOCK(USE_BLOCK),
		.OUT_REG(OUT_REG)
	) fifo (
		.clk(clk),
		.wr(fifo_wr),
		.din(fifo_din),
		.rd(rd),
		.dout(dout),
		.underflow(underflow),
		.overflow(overflow),
		.empty(empty),
		.full(full),
		.rsize(rsize),
		.wsize(wsize),
		.reset(reset)
	);

endmodule
