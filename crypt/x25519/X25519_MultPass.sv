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
 */
module X25519_MultPass(
	input wire			clk,
	input wire			en,
	input wire[4:0]		i,
	input wire[263:0]	a,
	input wire[263:0]	b,
	output logic		out_valid	= 0,
	output logic[31:0]	out = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Index calculation

	logic[4:0]	a_index[31:0];
	logic[4:0]	b_index[31:0];

	always_comb begin

		for(integer j=0; j<32; j=j+1) begin

			//just go down the array
			a_index[j]	= j;

			//wrap index if we go off the end
			if(j <= i)
				b_index[j]	= i - j;
			else
				b_index[j] = i - j + 32;

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First stage of multiplication

	logic[15:0]	stage2_tmp[31:0];

	logic[31:0]	stage2_do38		= 0;
	logic		stage2_en		= 0;

	always_ff @(posedge clk) begin

		stage2_en	<= en;

		if(en) begin
			for(integer j=0; j<32; j=j+1) begin
				stage2_do38[j]	<= (j > i);
				stage2_tmp[j]	<= a[a_index[j]*8 +: 8] * b[b_index[j]*8 +: 8];
			end
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second stage of multiplication (only for one side of the diagonal)

	logic[31:0]	stage3_tmp[31:0];
	logic		stage3_en		= 0;

	always_ff @(posedge clk) begin
		stage3_en	<= stage2_en;

		if(stage2_en) begin
			for(integer j=0; j<32; j=j+1) begin
				if(stage2_do38[j])
					stage3_tmp[j]	<= stage2_tmp[j] * 38;
				else
					stage3_tmp[j]	<= stage2_tmp[j];
			end
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Adder tree for the 32 temporary values. Do fanin of 4 per pipeline stage.

	logic[31:0]	stage4_tmp[7:0];
	logic		stage4_en			= 0;

	logic[31:0]	stage5_tmp[1:0];
	logic		stage5_en			= 0;

	always_ff @(posedge clk) begin
		stage4_en	<= stage3_en;
		stage5_en	<= stage4_en;
		out_valid	<= stage5_en;

		if(stage3_en) begin
			for(integer j=0; j<8; j=j+1) begin
				stage4_tmp[j]	<=
					stage3_tmp[j*4 + 0] +
					stage3_tmp[j*4 + 1] +
					stage3_tmp[j*4 + 2] +
					stage3_tmp[j*4 + 3];
			end
		end

		if(stage4_en) begin
			for(integer j=0; j<2; j=j+1) begin
				stage5_tmp[j]	<=
					stage4_tmp[j*4 + 0] +
					stage4_tmp[j*4 + 1] +
					stage4_tmp[j*4 + 2] +
					stage4_tmp[j*4 + 3];
			end
		end

		if(stage5_en)
			out	<= stage5_tmp[0] + stage5_tmp[1];

	end


endmodule
