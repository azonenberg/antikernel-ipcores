`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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

import Curve25519Registers::*;

//enable this to turn on KEEP_HIERARCHY for better area feedback during optimization
//turn off to enable flattening and improve performance/area
//`define FORCE_HIERARCHY

/**
	@file
	@author Andrew D. Zonenberg
	@brief X25519 multiplication

	Derived from mult() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)

	NOT pipelined. a/b must not change until out_valid is asserted.
 */
module X25519_Mult(
	input wire			clk,
	input wire			en,
	input wire[263:0]	a,
	input wire[263:0]	b,
	output wire			out_valid,
	output wire[263:0]	out
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 0: make bignums for the input values

	bignum_t	a_bignum;
	bignum_t	b_bignum;

	X25519_MakeBignum mbna(.din(a), .dout(a_bignum));
	X25519_MakeBignum mbnb(.din(b), .dout(b_bignum));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 1: run multpass to iteratively calculate one block of the output at a time

	wire		stage1_en;
	wire[4:0]	stage1_i;
	wire		pass_out_valid;
	wire[31:0]	pass_out;
	bignum_t	b_rotated;

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	X25519_MultMuxing mux(
		.clk(clk),
		.en(en),
		.pass_out_valid(pass_out_valid),
		.b_bignum(b_bignum),

		.stage1_en(stage1_en),
		.stage1_i(stage1_i),
		.b_rotated(b_rotated)
	);

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	X25519_MultPass pass(
		.clk(clk),
		.en(stage1_en),
		.i(stage1_i),
		.a(a_bignum),
		.b(b_rotated),
		.out_valid(pass_out_valid),
		.out(pass_out)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 2: reduce the multiplier output

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	X25519_StreamingSqueeze squeeze(
		.clk(clk),
		.en(en),
		.din_valid(pass_out_valid),
		.din(pass_out),
		.out_valid(out_valid),
		.out(out)
	);

endmodule

//TODO move to separate file?
module X25519_MultMuxing(
	input wire			clk,
	input wire			en,
	input wire			pass_out_valid,
	input bignum_t		b_bignum,

	output logic		stage1_en	= 0,
	output logic[4:0]	stage1_i	= 0,
	output bignum_t		b_rotated	= 0
);

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		stage1_en = 0;
		stage1_i = 0;
		b_rotated = 0;
	end
	`endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We need to rotate B by one block to the left each iteration

	always_ff @(posedge clk) begin

		stage1_en	= 0;

		//Start the first pass when we're ready
		if(en) begin
			stage1_en	= 1;
			stage1_i	<= 0;
		end

		//Start the next pass when the current one finishes
		if(pass_out_valid) begin
			stage1_i		<= stage1_i + 1'h1;
			if(stage1_i != 31)
				stage1_en	= 1;
		end

		//Input muxing
		if(stage1_en) begin

			//Least significant block
			if(en)
				b_rotated.blocks[0] 		<= b_bignum.blocks[0];
			else
				b_rotated.blocks[0]			<= b_rotated.blocks[31];

			//Other blocks
			for(integer i=0; i<31; i=i+1) begin
				if(en)
					b_rotated.blocks[i+1]	<= b_bignum.blocks[31-i];
				else
					b_rotated.blocks[i+1]	<= b_rotated.blocks[i];
			end

		end

	end

endmodule
