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
	@brief X25519 carry propagation and reduction

	Derived from squeeze() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)

 */
module X25519_StreamingSqueeze(
	input wire			clk,
	input wire			en,

	input wire			din_valid,
	input wire[4:0]		din_count,
	input wire[31:0]	din,

	output wire			out_valid,
	output wire[263:0]	out);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Carry propagation

	logic[31:0]		stage2_din_ff	= 0;

	logic[4:0]		stage2_i 		= 0;
	logic[4:0]		stage2_i_ff		= 0;

	logic[263:0]	stage3_din		= 0;
	logic			stage3_en		= 0;
	logic[263:0]	stage3_carryin	= 0;

	logic			finishing		= 0;

	always_ff @(posedge clk) begin

		stage3_en	<= 0;
		finishing	<= 0;

		//Reset counters when we start a new squeeze operation
		if(en) begin
			stage2_i		<= 0;
			stage2_i_ff		<= 0;
			stage2_din_ff	<= 0;
		end

		//As each block of data comes off the multiplier, do the ripple carry
		if(din_valid) begin

			stage2_i_ff						<= stage2_i;

			//Save the low 8 bits
			stage3_din[stage2_i_ff*8 +: 8]	<= stage2_din_ff[7:0];

			//Check if we're done
			if(stage2_i == 31)
				finishing					<= 1;

			//Propagate the carry bits into the next word.
			stage2_din_ff					<= stage2_din_ff[31:8] + din;
			stage2_i						<= stage2_i + 1'h1;

		end

		//If we're at the end, save the carry-out and feed it into the squeezer
		if(finishing) begin
			stage3_din[stage2_i_ff*8 +: 8]	<= stage2_din_ff[7:0];
			stage3_din[263:255]				<= 0;

			stage3_en						<= 1;
			stage3_carryin					<= stage2_din_ff[31:7] * 19;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Add in the overflow

	X25519_Add overflow_adder(
		.clk(clk),
		.en(stage3_en),
		.a(stage3_din),
		.b(stage3_carryin),
		.out_valid(out_valid),
		.out(out)
	);

endmodule
