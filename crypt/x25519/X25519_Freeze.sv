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
	@brief X25519 reduction

	Derived from freeze() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)

	WTF does this do?
 */
module X25519_Freeze(
	input wire			clk,
	input wire			en,
	input wire[263:0]	a,
	output logic		out_valid	= 0,
	output logic[263:0]	out = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First stage: save input and add minus_p to it

	logic[263:0]	minus_p	= 264'h00_8000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0013;
	wire[263:0]		stage2_a;
	wire			stage2_en;

	X25519_Add adder(
		.clk(clk),
		.en(en),
		.a(a),
		.b(minus_p),
		.out_valid(stage2_en),
		.out(stage2_a)
	);

	logic[263:0]	stage2_a_orig = 0;

	always_ff @(posedge clk) begin
		if(en)
			stage2_a_orig	<= a;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second stage: big pile of XORs

	always_ff @(posedge clk) begin
		out_valid	<= stage2_en;

		if(stage2_a[255])
			out		<= stage2_a ^ stage2_a_orig;
		else
			out		<= stage2_a_orig;

	end

endmodule
