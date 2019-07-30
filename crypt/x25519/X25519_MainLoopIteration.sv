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

	Derived from mainloop() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)
 */
module X25519_MainLoopIteration(
	input wire			clk,
	input wire			en,

	input wire[511:0]	xzm1_in,
	input wire[511:0]	xzm_in,
	input wire			b,

	output wire			out_valid,
	output wire[511:0]	xzm_out,
	output wire[511:0]	xzm1_out
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// select(xzmb,xzm1b,xzm,xzm1,b);

	//note that the 512-bit vectors can never have carry-outs since mult() squeezes the results

	wire[511:0]	xzmb;
	wire[511:0]	xzm1b;
	wire		bsel_valid;

	X25519_Select l1_bsel(
		.clk(clk),
		.en(en),
		.p(xzmb),
		.q(xzm1b),
		.r(xzm_in),
		.s(xzm1_in),
		.b(b),
		.out_valid(bsel_valid));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// add(a0,xzmb,xzmb + 32);
	// sub(a0 + 32,xzmb,xzmb + 32);

	wire		a0_valid;
	wire[263:0]	a0_low;
	wire[263:0]	a0_high;

	X25519_Add l2_a0_add(
		.clk(clk),
		.en(bsel_valid),
		.a({8'h0, xzmb[255:0]}),
		.b({8'h0, xzmb[511:256]}),
		.out_valid(a0_valid),
		.out(a0_low)
	);

	X25519_Sub l2_a0_sub(
		.clk(clk),
		.en(bsel_valid),
		.a({8'h0, xzmb[255:0]}),
		.b({8'h0, xzmb[511:256]}),
		.out_valid(),
		.out(a0_high)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// add(a1,xzm1b,xzm1b + 32);
	// sub(a1 + 32,xzm1b,xzm1b + 32);

	wire		a1_valid;
	wire[263:0]	a1_low;
	wire[263:0]	a1_high;

	X25519_Add l3_a1_add(
		.clk(clk),
		.en(bsel_valid),
		.a({8'h0, xzm1b[255:0]}),
		.b({8'h0, xzm1b[511:256]}),
		.out_valid(a1_valid),
		.out(a1_low)
	);

	X25519_Sub l3_a1_sub(
		.clk(clk),
		.en(bsel_valid),
		.a({8'h0, xzm1b[255:0]}),
		.b({8'h0, xzm1b[511:256]}),
		.out_valid(),
		.out(a1_high)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// square(b0,a0);
	// square(b0 + 32,a0 + 32);

	wire		b0_valid;
	wire[263:0]	b0_low;
	wire[263:0]	b0_high;

	X25519_Mult l4_b0_sqlow(
		.clk(clk),
		.en(a0_valid),
		.a(a0_low),
		.b(a0_low),
		.out_valid(b0_valid),
		.out(b0_low));

	//THIS IS GIVING WRONG RESULTS!!
	X25519_Mult l4_b0_sqhi(
		.clk(clk),
		.en(a0_valid),
		.a(a0_high),
		.b(a0_high),
		.out_valid(),
		.out(b0_high));

endmodule
