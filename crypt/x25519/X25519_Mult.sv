`timescale 1ns/1ps
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
	// Step 1: run multpass to iteratively calculate one block of the output at a time

	logic		stage1_en	= 0;
	logic[4:0]	stage1_i	= 0;
	wire		pass_out_valid;
	wire[31:0]	pass_out;

	//We need to rotate B by 8 bits to the left each iteration, but also reverse it at the start
	logic[255:0]	b_rotated	= 0;

	X25519_MultPass pass(
		.clk(clk),
		.en(stage1_en),
		.i(stage1_i),
		.a(a),
		.b(b_rotated),
		.out_valid(pass_out_valid),
		.out(pass_out)
	);

	always_ff @(posedge clk) begin

		stage1_en	<= 0;

		//Start the first pass when we're started
		if(en) begin
			stage1_en	<= 1;
			stage1_i	<= 0;
			for(integer i=0; i<32; i=i+1) begin
				if(i != 31)
					b_rotated[(i+1)*8 +: 8]	<= b[(31-i)*8 +: 8];
				else
					b_rotated[0*8 +: 8]	<= b[(31-i)*8 +: 8];
			end
		end

		//Start the next pass when the current one finishes
		if(pass_out_valid) begin
			stage1_i				<= stage1_i + 1'h1;
			if(stage1_i != 31) begin
				stage1_en			<= 1;
				b_rotated			<= { b_rotated[247:0], b_rotated[255:248] };
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 2: reduce the multiplier output

	X25519_StreamingSqueeze squeeze(
		.clk(clk),
		.en(en),
		.din_valid(pass_out_valid),
		.din(pass_out),
		.out_valid(out_valid),
		.out(out)
	);

endmodule
