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

	Derived from mult() + squeeze() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)

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

	X25519_MultPass pass(
		.clk(clk),
		.en(stage1_en),
		.i(stage1_i),
		.a(a),
		.b(b),
		.out_valid(pass_out_valid),
		.out(pass_out)
	);

	logic		stage2_en		= 0;

	always_ff @(posedge clk) begin

		stage1_en	<= 0;
		stage2_en	<= 0;

		//Start the first pass when we're started
		if(en) begin
			stage1_en	<= 1;
			stage1_i	<= 0;
		end

		//Start the next pass when the current one finishes
		if(pass_out_valid) begin
			stage1_i				<= stage1_i + 1'h1;
			if(stage1_i == 31)
				stage2_en			<= 1;
			else
				stage1_en			<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 2: carry propagation (part of the squeeze)

	logic[31:0]		stage2_din[31:0];

	logic[4:0]		stage2_i 		= 0;
	wire[4:0]		stage2_i_inc	= stage2_i + 1'h1;
	logic			stage2_active	= 0;

	wire[31:0]		stage2_cur		= stage2_din[stage2_i];

	logic[263:0]	stage3_din		= 0;
	logic			stage3_en		= 0;
	logic[263:0]	stage3_carryin	= 0;

	always_ff @(posedge clk) begin

		stage3_en	<= 0;

		//Save the intermediate values as the multpass block finishes up
		if(pass_out_valid)
			stage2_din[stage1_i]	<= pass_out;

		//Start the propagation process when the multpass'ing is done
		//TODO: can we run this in parallel, rippling as the multiplication happens?
		//That would likely save 30-ish cycles of latency!
		if(stage2_en) begin
			stage2_i		<= 0;
			stage2_active	<= 1;
		end

		//Ripple carry
		if(stage2_active) begin

			//Save the low 8 bits
			stage3_din[stage2_i*8 +: 8]		<= stage2_cur[7:0];

			//If we're at the end, save the carry-out and feed it into the squeezer
			if(stage2_i == 31) begin
				stage3_din[263:256]			<= 0;
				stage2_active				<= 0;
				stage3_en					<= 1;
				stage3_carryin				<= stage2_cur[31:7] * 19;

				//not quite sure why we need to truncate this bit but seems necessary to match the C ref implementation
				stage3_din[255]				<= 0;
			end

			//Otherwise propagate the carry bits into the next word.
			else begin
				stage2_din[stage2_i_inc]	<= stage2_cur[31:8] + stage2_din[stage2_i_inc];
				stage2_i					<= stage2_i + 1'h1;
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 3: add the carry

	X25519_Add squeeze(
		.clk(clk),
		.en(stage3_en),
		.a(stage3_din),
		.b(stage3_carryin),
		.out_valid(out_valid),
		.out(out)
	);

endmodule
