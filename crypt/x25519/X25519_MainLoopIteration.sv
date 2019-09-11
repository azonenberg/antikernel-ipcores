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

	input wire[263:0]	work_low,

	output wire			out_valid,
	output wire[511:0]	xzm_out,
	output wire[511:0]	xzm1_out
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RESOURCE SHARING

	logic			share_add_en	= 0;
	logic[263:0]	share_add_a		= 0;
	logic[263:0]	share_add_b		= 0;
	wire			share_add_valid;
	wire[263:0]		share_add_out;

	X25519_Add share_add(
		.clk(clk),
		.en(share_add_en),
		.a(share_add_a),
		.b(share_add_b),
		.out_valid(share_add_valid),
		.out(share_add_out)
	);

	logic			share_sub_en	= 0;
	logic[263:0]	share_sub_a		= 0;
	logic[263:0]	share_sub_b		= 0;
	wire			share_sub_valid;
	wire[263:0]		share_sub_out;

	X25519_Sub share_sub(
		.clk(clk),
		.en(share_sub_en),
		.a(share_sub_a),
		.b(share_sub_b),
		.out_valid(share_sub_valid),
		.out(share_sub_out)
	);

	logic			share_select_en	= 0;
	logic[511:0]	share_select_r	= 0;
	logic[511:0]	share_select_s	= 0;
	wire[511:0]		share_select_p;
	wire[511:0]		share_select_q;
	wire			share_select_valid;

	X25519_Select share_select(
		.clk(clk),
		.en(share_select_en),
		.p(share_select_p),
		.q(share_select_q),
		.r(share_select_r),
		.s(share_select_s),
		.b(b),
		.out_valid(share_select_valid));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main loop

	enum logic[3:0]
	{
		STATE_IDLE			= 0,
		STATE_SELECT_INIT	= 1,
		STATE_A0			= 2,
		STATE_A1			= 3
	} state = STATE_IDLE;

	//Temporary variables
	logic[511:0]	xzmb	= 0;
	logic[511:0]	xzm1b	= 0;
	logic[263:0]	a0_low	= 0;
	logic[263:0]	a0_high	= 0;

	//Valid flags for temporary variables
	//TODO: remove when nothing uses them anymore
	logic			a0_valid	= 0;
	logic			bsel_valid	= 0;

	always_ff @(posedge clk) begin
		share_add_en	<= 0;
		share_sub_en	<= 0;
		share_select_en	<= 0;

		bsel_valid		<= 0;
		a0_valid		<= 0;

		case(state)

			//wait for an operation to start
			STATE_IDLE: begin
				if(en) begin

					//select(xzmb,xzm1b,xzm,xzm1,b);
					share_select_en	<= 1;
					share_select_r	<= xzm_in;
					share_select_s	<= xzm1_in;

					state			<= STATE_SELECT_INIT;
				end
			end	//end STATE_IDLE

			STATE_SELECT_INIT: begin
				if(share_select_valid) begin

					//Save results
					xzmb			<= share_select_p;
					xzm1b			<= share_select_q;
					bsel_valid		<= 1;

					//add(a0,xzmb,xzmb + 32);
					share_add_en	<= 1;
					share_add_a		<= { 8'h0, share_select_p[255:0] };
					share_add_b		<= { 8'h0, share_select_p[511:256] };

					//sub(a0 + 32,xzmb,xzmb + 32);
					share_sub_en	<= 1;
					share_sub_a		<= { 8'h0, share_select_p[255:0] };
					share_sub_b		<= { 8'h0, share_select_p[511:256] };

					state			<= STATE_A0;
				end
			end	//end STATE_SELECT_INIT


			STATE_A0: begin

				if(share_add_valid) begin

					//Save results
					a0_low			<= share_add_out;
					a0_high			<= share_sub_out;
					a0_valid		<= 1;

					//add(a1,xzm1b,xzm1b + 32);
					//sub(a1 + 32,xzm1b,xzm1b + 32);

					//FIXME
					state				<= STATE_A1;
				end

			end	//end STATE_A0

			STATE_A1: begin
				//FIXME
				state				<= STATE_IDLE;
			end	//end STATE_A1

		endcase
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

	X25519_Mult l4_b0_sqhi(
		.clk(clk),
		.en(a0_valid),
		.a(a0_high),
		.b(a0_high),
		.out_valid(),
		.out(b0_high));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// mult(b1,a1,a0 + 32);
	// mult(b1 + 32,a1 + 32,a0);

	wire		b1_valid;
	wire[263:0]	b1_low;
	wire[263:0]	b1_high;

	X25519_Mult l5_b1_low(
		.clk(clk),
		.en(a0_valid),
		.a(a1_low),
		.b(a0_high),
		.out_valid(b1_valid),
		.out(b1_low));

	X25519_Mult l5_b1_high(
		.clk(clk),
		.en(a0_valid),
		.a(a1_high),
		.b(a0_low),
		.out_valid(),
		.out(b1_high));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// add(c1,b1,b1 + 32);
	// sub(c1 + 32,b1,b1 + 32);

	wire		c1_valid;
	wire[263:0]	c1_low;
	wire[263:0]	c1_high;

	X25519_Add l6_c1_add(
		.clk(clk),
		.en(b1_valid),
		.a(b1_low),
		.b(b1_high),
		.out_valid(c1_valid),
		.out(c1_low)
	);

	X25519_Sub l6_c1_sub(
		.clk(clk),
		.en(b1_valid),
		.a(b1_low),
		.b(b1_high),
		.out_valid(),
		.out(c1_high)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// square(r,c1 + 32);
	// sub(s,b0,b0 + 32);

	wire		r_valid;
	wire[263:0]	r;

	wire		s_valid;
	wire[263:0]	s;

	X25519_Mult l7_r(
		.clk(clk),
		.en(c1_valid),
		.a(c1_high),
		.b(c1_high),
		.out_valid(r_valid),
		.out(r));

	X25519_Sub l7_s(
		.clk(clk),
		.en(b0_valid),
		.a(b0_low),
		.b(b0_high),
		.out_valid(s_valid),
		.out(s)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// mult121665(t,s);

	wire		t_valid;
	wire[263:0]	t;

	X25519_Mult121665 l8_t(
		.clk(clk),
		.en(s_valid),
		.a(s),
		.out_valid(t_valid),
		.out(t));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// add(u,t,b0);

	wire		u_valid;
	wire[263:0]	u;

	X25519_Add l9_u(
		.clk(clk),
		.en(t_valid),
		.a(t),
		.b(b0_low),
		.out_valid(u_valid),
		.out(u)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// mult(xznb,b0,b0 + 32);
	// mult(xznb + 32,s,u);

	wire		xznb_low_valid;
	wire[263:0]	xznb_low;
	wire		xznb_high_valid;
	wire[263:0]	xznb_high;

	X25519_Mult l10_xznb_low(
		.clk(clk),
		.en(b0_valid),
		.a(b0_low),
		.b(b0_high),
		.out_valid(xznb_low_valid),
		.out(xznb_low));

	X25519_Mult l10_xznb_high(
		.clk(clk),
		.en(u_valid),
		.a(s),
		.b(u),
		.out_valid(xznb_high_valid),
		.out(xznb_high));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// square(xzn1b,c1);
	// mult(xzn1b + 32,r,work);

	wire		xzn1b_low_valid;
	wire[263:0]	xzn1b_low;
	wire		xzn1b_high_valid;
	wire[263:0]	xzn1b_high;

	X25519_Mult l11_xzn1b_low(
		.clk(clk),
		.en(c1_valid),
		.a(c1_low),
		.b(c1_low),
		.out_valid(xzn1b_low_valid),
		.out(xzn1b_low));

	X25519_Mult l11_xzn1b_lhigh(
		.clk(clk),
		.en(r_valid),
		.a(r),
		.b(work_low),
		.out_valid(xzn1b_high_valid),
		.out(xzn1b_high));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// select(xzm,xzm1,xznb,xzn1b,b);

	X25519_Select l12_select(
		.clk(clk),
		.en(xznb_high_valid),
		.b(b),
		.r({xznb_high[255:0], xznb_low[255:0]}),
		.s({xzn1b_high[255:0], xzn1b_low[255:0]}),
		.p(xzm_out),
		.q(xzm1_out),
		.out_valid(out_valid)
	);

endmodule
