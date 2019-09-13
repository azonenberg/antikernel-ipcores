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

	output logic		out_valid	= 0,
	output logic[511:0]	xzm_out		= 0,
	output logic[511:0]	xzm1_out	= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RESOURCE SHARING

	logic			share_add_en	= 0;
	logic[263:0]	share_addsub_a	= 0;
	logic[263:0]	share_addsub_b	= 0;
	wire			share_add_valid;
	wire[263:0]		share_add_out;

	X25519_Add share_add(
		.clk(clk),
		.en(share_add_en),
		.a(share_addsub_a),
		.b(share_addsub_b),
		.out_valid(share_add_valid),
		.out(share_add_out)
	);

	logic			share_sub_en	= 0;
	wire			share_sub_valid;
	wire[263:0]		share_sub_out;

	X25519_Sub share_sub(
		.clk(clk),
		.en(share_sub_en),
		.a(share_addsub_a),
		.b(share_addsub_b),
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

	logic			share_mult_en	= 0;
	logic[263:0]	share_mult_a	= 0;
	logic[263:0]	share_mult_b	= 0;
	wire[263:0]		share_mult_out;
	wire			share_mult_valid;

	X25519_Mult share_mult(
		.clk(clk),
		.en(share_mult_en),
		.a(share_mult_a),
		.b(share_mult_b),
		.out_valid(share_mult_valid),
		.out(share_mult_out));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main loop

	enum logic[3:0]
	{
		STATE_IDLE			= 4'h0,
		STATE_SELECT_INIT	= 4'h1,
		STATE_A0			= 4'h2,
		STATE_B0_LOW		= 4'h4,
		STATE_B0_HIGH		= 4'h5,
		STATE_B1_LOW		= 4'h6,
		STATE_B1_HIGH		= 4'h7,
		STATE_C1			= 4'h8,
		STATE_R				= 4'h9,
		STATE_T				= 4'ha,
		STATE_XB_LOW		= 4'hb,
		STATE_XB_HIGH		= 4'hc,
		STATE_XN_LOW		= 4'hd,
		STATE_XN_HIGH		= 4'he,
		STATE_FINISH		= 4'hf
	} state = STATE_IDLE;

	//Temporary variables
	logic[511:0]	xzmb	= 0;
	logic[511:0]	xzm1b	= 0;
	logic[263:0]	a0_low	= 0;
	logic[263:0]	a0_high	= 0;
	logic[263:0]	a1_low	= 0;
	logic[263:0]	a1_high	= 0;
	logic[263:0]	b0_low	= 0;
	logic[263:0]	b0_high	= 0;
	logic[263:0]	b1_low	= 0;
	logic[263:0]	c1_low	= 0;
	logic[263:0]	r		= 0;
	logic[263:0]	s		= 0;
	logic[263:0]	xznb_low	= 0;
	logic[263:0]	xznb_high	= 0;
	logic[263:0]	xzn1b_low	= 0;

	always_ff @(posedge clk) begin
		share_add_en	<= 0;
		share_sub_en	<= 0;
		share_select_en	<= 0;
		share_mult_en	<= 0;

		out_valid		<= 0;

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

					//add(a0,xzmb,xzmb + 32);
					share_add_en	<= 1;
					share_addsub_a	<= { 8'h0, share_select_p[255:0] };
					share_addsub_b	<= { 8'h0, share_select_p[511:256] };

					//sub(a0 + 32,xzmb,xzmb + 32);
					share_sub_en	<= 1;

					state			<= STATE_A0;
				end
			end	//end STATE_SELECT_INIT


			STATE_A0: begin

				if(share_add_valid) begin

					//Save results
					a0_low			<= share_add_out;
					a0_high			<= share_sub_out;

					//add(a1,xzm1b,xzm1b + 32);
					share_add_en	<= 1;
					share_addsub_a	<= { 8'h0, xzm1b[255:0] };
					share_addsub_b	<= { 8'h0, xzm1b[511:256] };

					//sub(a1 + 32,xzm1b,xzm1b + 32);
					share_sub_en	<= 1;

					//square(b0,a0);
					share_mult_en	<= 1;
					share_mult_a	<= share_add_out;
					share_mult_b	<= share_add_out;

					state			<= STATE_B0_LOW;
				end

			end	//end STATE_A0

			STATE_B0_LOW: begin

				//Save add results before multiply finishes
				if(share_add_valid) begin
					a1_low			<= share_add_out;
					a1_high			<= share_sub_out;
				end

				if(share_mult_valid) begin

					//Save results
					b0_low			<= share_mult_out;

					//square(b0 + 32,a0 + 32);
					share_mult_en	<= 1;
					share_mult_a	<= a0_high;
					share_mult_b	<= a0_high;

					state			<= STATE_B0_HIGH;

				end

			end	//end STATE_B0_LOW

			STATE_B0_HIGH: begin
				if(share_mult_valid) begin

					//Save results
					b0_high			<= share_mult_out;

					//mult(b1,a1,a0 + 32);
					share_mult_en	<= 1;
					share_mult_a	<= a1_low;
					share_mult_b	<= a0_high;

					state			<= STATE_B1_LOW;

				end
			end	//end STATE_B0_HIGH

			STATE_B1_LOW: begin
				if(share_mult_valid) begin

					//Save results
					b1_low			<= share_mult_out;

					//mult(b1 + 32,a1 + 32,a0);
					share_mult_en	<= 1;
					share_mult_a	<= a1_high;
					share_mult_b	<= a0_low;

					state			<= STATE_B1_HIGH;

				end
			end	//end STATE_B1_LOW

			STATE_B1_HIGH: begin
				if(share_mult_valid) begin

					//add(c1,b1,b1 + 32);
					share_add_en	<= 1;
					share_addsub_a	<= b1_low;
					share_addsub_b	<= share_mult_out;

					//sub(c1 + 32,b1,b1 + 32);
					share_sub_en	<= 1;

					state			<= STATE_C1;
				end
			end	//end STATE_B1_HIGH

			STATE_C1: begin
				if(share_add_valid) begin

					//Save results
					c1_low			<= share_add_out;

					//square(r,c1 + 32);
					share_mult_en	<= 1;
					share_mult_a	<= share_sub_out;
					share_mult_b	<= share_sub_out;

					//sub(s,b0,b0 + 32);
					share_sub_en	<= 1;
					share_addsub_a	<= b0_low;
					share_addsub_b	<= b0_high;

					state			<= STATE_R;

				end
			end	//end STATE_C1

			STATE_R: begin

				if(share_sub_valid)
					s				<= share_sub_out;

				if(share_mult_valid) begin

					//Save results
					r				<= share_mult_out;

					//mult121665(t,s);
					share_mult_en	<= 1;
					share_mult_a	<= s;
					share_mult_b	<= 264'd121665;

					state			<= STATE_T;
				end

			end	//end STATE_R

			STATE_T: begin
				if(share_mult_valid) begin

					//add(u,t,b0);
					share_add_en	<= 1;
					share_addsub_a	<= share_mult_out;
					share_addsub_b	<= b0_low;

					//mult(xznb,b0,b0 + 32);
					share_mult_en	<= 1;
					share_mult_a	<= b0_low;
					share_mult_b	<= b0_high;

					state			<= STATE_XB_LOW;
				end
			end	//end STATE_T

			STATE_XB_LOW: begin

				//share_add_valid will be asserted before share_mult_valid
				//so we can just use share_add_out as u

				if(share_mult_valid) begin

					//Save results
					xznb_low		<= share_mult_out;

					//mult(xznb + 32,s,u);
					share_mult_en	<= 1;
					share_mult_a	<= s;
					share_mult_b	<= share_add_out;
					state			<= STATE_XB_HIGH;

				end

			end	//end STATE_XB_LOW

			STATE_XB_HIGH: begin
				if(share_mult_valid) begin
					xznb_high		<= share_mult_out;

					//square(xzn1b,c1);
					share_mult_en	<= 1;
					share_mult_a	<= c1_low;
					share_mult_b	<= c1_low;

					state			<= STATE_XN_LOW;
				end
			end	//end STATE_XB_HIGH

			STATE_XN_LOW: begin
				if(share_mult_valid) begin
					xzn1b_low		<= share_mult_out;

					//mult(xzn1b + 32,r,work);
					share_mult_en	<= 1;
					share_mult_a	<= r;
					share_mult_b	<= work_low;

					state			<= STATE_XN_HIGH;

				end
			end	//end STATE_XN_LOW

			STATE_XN_HIGH: begin
				if(share_mult_valid) begin

					//select(xzm,xzm1,xznb,xzn1b,b);
					share_select_en	<= 1;
					share_select_r	<= {xznb_high[255:0], xznb_low[255:0]};
					share_select_s	<= {share_mult_out[255:0], xzn1b_low[255:0]};

					state			<= STATE_FINISH;
				end
			end	//end STATE_XN_HIGH

			STATE_FINISH: begin
				if(share_select_valid) begin
					xzm_out			<= share_select_p;
					xzm1_out		<= share_select_q;
					out_valid		<= 1;
					state			<= STATE_IDLE;
				end
			end

		endcase
	end

endmodule
