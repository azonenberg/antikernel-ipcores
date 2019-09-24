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

	Derived from mainloop() and crypto_scalarmult() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)
 */
module X25519_MainLoop(
	input wire			clk,
	input wire			en,
	input wire[255:0]	work_in,
	input wire[255:0]	e,
	output logic		out_valid	= 0,
	output logic[511:0]	work_out	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loop contents

	logic			iter_en		= 0;
	logic			iter_first	= 0;
	logic			b			= 0;

	logic			iter_out_valid	= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RESOURCE SHARING

	logic			share_addsub_en	= 0;
	logic[263:0]	share_addsub_a	= 0;
	logic[263:0]	share_addsub_b	= 0;
	wire			share_add_valid;
	wire[263:0]		share_add_out;

	X25519_Add share_add(
		.clk(clk),
		.en(share_addsub_en),
		.a(share_addsub_a),
		.b(share_addsub_b),
		.out_valid(share_add_valid),
		.out(share_add_out)
	);

	wire			share_sub_valid;
	wire[263:0]		share_sub_out;

	X25519_Sub share_sub(
		.clk(clk),
		.en(share_addsub_en),
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

	typedef enum logic[5:0]
	{
		//mainloop()
		STATE_IDLE			= 6'h00,
		STATE_SELECT_DONE	= 6'h01,
		STATE_A0			= 6'h02,
		STATE_B0_LOW		= 6'h03,
		STATE_B0_HIGH		= 6'h04,
		STATE_B1_LOW		= 6'h05,
		STATE_B1_HIGH		= 6'h06,
		STATE_C1			= 6'h07,
		STATE_R				= 6'h08,
		STATE_T				= 6'h09,
		STATE_XB_LOW		= 6'h0a,
		STATE_XB_HIGH		= 6'h0b,
		STATE_XN_LOW		= 6'h0c,
		STATE_XN_HIGH		= 6'h0d,

		//recip()
		STATE_RECIP			= 6'h0e,
		STATE_R4			= 6'h0f,
		STATE_R8			= 6'h10,
		STATE_R9			= 6'h11,
		STATE_R11			= 6'h12,
		STATE_R22			= 6'h13,
		STATE_R31			= 6'h14,
		STATE_R61			= 6'h15,
		STATE_R72			= 6'h16,
		STATE_R83			= 6'h17,
		STATE_R94			= 6'h18,
		STATE_R105			= 6'h19,
		STATE_R100			= 6'h1a,
		STATE_R111			= 6'h1b,
		STATE_R122			= 6'h1c,
		STATE_R2010_LOOP	= 6'h1d,
		STATE_R2010			= 6'h1e,
		STATE_R200			= 6'h1f,
		STATE_R211			= 6'h20,
		STATE_R4020_LOOP	= 6'h21,
		STATE_R4020			= 6'h22,
		STATE_R400			= 6'h23,
		STATE_R411			= 6'h24,
		STATE_R5010_LOOP	= 6'h25,
		STATE_R5010			= 6'h26,
		STATE_R500			= 6'h27,
		STATE_R511			= 6'h28,
		STATE_R522			= 6'h29,
		STATE_R10050_LOOP	= 6'h2a,
		STATE_R10050		= 6'h2b,
		STATE_R1000			= 6'h2c,
		STATE_R1011			= 6'h2d,
		STATE_R200100_LOOP	= 6'h2e,
		STATE_R200100		= 6'h2f,
		STATE_R2000			= 6'h30,
		STATE_R2011			= 6'h31,
		STATE_R25050_LOOP	= 6'h32,
		STATE_R25050		= 6'h33,

		STATE_MAX
	} state_t;

	state_t state = STATE_IDLE;

	//Memory for temporary variables
	typedef enum logic[3:0]
	{
		//General purpose registers, writable and usable everywhere
		REG_TEMP_0		= 4'h00,
		REG_TEMP_1		= 4'h01,
		REG_TEMP_2		= 4'h02,
		REG_TEMP_3		= 4'h03,
		REG_TEMP_4		= 4'h04,
		REG_TEMP_5		= 4'h05,

		//Special registers (named, but not always usable in every operation)
		REG_SELP_LO		= 4'h06,	//4x select outputs
		REG_SELP_HI		= 4'h07,
		REG_SELQ_LO		= 4'h08,
		REG_SELQ_HI		= 4'h09,
		REG_121665		= 4'h0a,	//always 121665
		REG_ZERO		= 4'h0b,	//throw away unused values
		REG_WORK_LOW	= 4'h0c,
		REG_WORK_HI		= 4'h0d		//must be last
	} regid_t;
	logic[263:0]	temp_regs[REG_TEMP_5:0];

	//Initialize registers, including constants
	initial begin

		for(integer i=0; i<=REG_TEMP_5; i++)
			temp_regs[i]		<= 0;

	end

	//Microcode definitions
	typedef struct packed
	{
		logic		select_en;
		logic		addsub_en;
		logic		mult_en;
		regid_t		addsub_a;
		regid_t		addsub_b;
		regid_t		mult_a;
		regid_t		mult_b;

		/////

		regid_t		add_out;
		regid_t		sub_out;
		regid_t		mult_out;

		logic		next_on_add;
		logic		next_on_mult;
		logic		next_on_select;
		state_t		next;

		logic		loop;				//if true, this state is a loop
										//All loops start at i=2 and count by 1
		logic[6:0]	loop_max;
	} microcode_t;

	microcode_t[STATE_MAX-1:0] ucode;
	initial begin

		//Filler
		for(integer i=0; i<STATE_MAX; i++)
			ucode[i] = {$bits(microcode_t){1'b0}};

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// MAINLOOP

		//Idle
		ucode[STATE_IDLE] = { 3'b000, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SELECT_DONE, 1'b0, 7'd0 };

		//add(a0,xzmb,xzmb + 32);
		//sub(a0 + 32,xzmb,xzmb + 32);
		ucode[STATE_SELECT_DONE] = { 3'b010, REG_SELP_LO, REG_SELP_HI, REG_ZERO, REG_ZERO,	//TEMP_0 is now A0_LO
			REG_TEMP_0, REG_TEMP_1, REG_ZERO, 3'b100, STATE_A0, 1'b0, 7'd0 };				//TEMP_1 is now A0_HI

		//add(a1,xzm1b,xzm1b + 32);
		//sub(a1 + 32,xzm1b,xzm1b + 32);
		//square(b0,a0);
		ucode[STATE_A0] = {3'b011, REG_SELQ_LO, REG_SELQ_HI, REG_TEMP_0, REG_TEMP_0,		//TEMP_2 is now A1_LO
			REG_TEMP_2, REG_TEMP_3, REG_TEMP_4, 3'b010, STATE_B0_LOW, 1'b0, 7'd0 };			//TEMP_3 is now A1_HI
																							//TEMP_4 is now B0_LO

		//square(b0 + 32,a0 + 32);
		ucode[STATE_B0_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,
			REG_ZERO, REG_ZERO, REG_TEMP_5, 3'b010, STATE_B0_HIGH, 1'b0, 7'd0 };

		//mult(b1,a1,a0 + 32);
		ucode[STATE_B0_HIGH] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_1,	//last use of TEMP_1 as A0_HI
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_B1_LOW, 1'b0, 7'd0 };		//TEMP_1 is now B1_LO

		//mult(b1 + 32,a1 + 32,a0);
		ucode[STATE_B1_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_0,
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_B1_HIGH, 1'b0, 7'd0 };	//TEMP_0 is now B1_HI

		//add(c1,b1,b1 + 32);
		//sub(c1 + 32,b1,b1 + 32);
		ucode[STATE_B1_HIGH] = {3'b010, REG_TEMP_1, REG_TEMP_0, REG_ZERO, REG_ZERO, //TEMP_0 is now C1_LO
			REG_TEMP_0, REG_TEMP_1, REG_ZERO, 3'b100, STATE_C1, 1'b0, 7'd0 };		//TEMP_1 is now C1_HI

		//sub(s,b0,b0 + 32);
		//square(r,c1 + 32);
		ucode[STATE_C1] = {3'b011, REG_TEMP_4, REG_TEMP_5, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is now S
			REG_ZERO, REG_TEMP_1, REG_TEMP_2, 3'b010, STATE_R, 1'b0, 7'd0 };		//TEMP_2 is now R

		//mult121665(t,s);
		ucode[STATE_R] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_121665,
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_T, 1'b0, 7'd0 };			//TEMP_3 is now T

		//add(u,t,b0);
		//mult(xznb,b0,b0 + 32);
		ucode[STATE_T] = {3'b011, REG_TEMP_3, REG_TEMP_4, REG_TEMP_4, REG_TEMP_5,	//TEMP_3 is now XZNB_LO
			REG_TEMP_4, REG_ZERO, REG_TEMP_3, 3'b010, STATE_XB_LOW, 1'b0, 7'd0 };	//TEMP_4 is now U

		//mult(xznb + 32,s,u);
		ucode[STATE_XB_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_4,	//TEMP_1 is now XZNB_HI
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_XB_HIGH, 1'b0, 7'd0 };

		//square(xzn1b,c1);
		ucode[STATE_XB_HIGH] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0, //TEMP_0 is now XZN1B_LO
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_XN_LOW, 1'b0, 7'd0 };

		//mult(xzn1b + 32,r,work);
		ucode[STATE_XN_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_WORK_LOW,//TEMP_2 is now XZN1B_HI
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_XN_HIGH, 1'b0, 7'd0 };

		//select(xzm,xzm1,xznb,xzn1b,b);
		ucode[STATE_XN_HIGH] = {3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_IDLE, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// RECIP

		//2: square(z2,z);
		ucode[STATE_RECIP] = { 3'b001, REG_ZERO, REG_ZERO, REG_WORK_HI, REG_WORK_HI,	//TEMP_0 is now z2
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_R4, 1'b0, 7'd0 };

		//4: square(t1,z2);
		ucode[STATE_R4] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0,			//TEMP_1 is now t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R8, 1'b0, 7'd0 };

		//8: square(t0,t1);
		ucode[STATE_R8] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,			//TEMP_2 is now t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R9, 1'b0, 7'd0 };

		//9: mult(z9,t0,z);
		ucode[STATE_R9] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_WORK_HI,			//TEMP_3 is now z9
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R11, 1'b0, 7'd0 };

		//11: mult(z11,z9,z2);
		ucode[STATE_R11] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_0,			//TEMP_0 is now z11
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_R22, 1'b0, 7'd0 };

		//22: square(t0,z11);
		ucode[STATE_R22] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0,			//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R31, 1'b0, 7'd0 };

		//31: mult(z2_5_0,t0,z9);
		ucode[STATE_R31] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_3,			//TEMP_3 is now z2_5_0
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R61, 1'b0, 7'd0 };

		//2^6 - 2^1: square(t0,z2_5_0);
		ucode[STATE_R61] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_3,			//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R72, 1'b0, 7'd0 };

		//2^7 - 2^2: square(t1,t0);
		ucode[STATE_R72] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,			//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R83, 1'b0, 7'd0 };

		//2^8 - 2^3: square(t0,t1);
		ucode[STATE_R83] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,			//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R94, 1'b0, 7'd0 };

		//2^9 - 2^4: square(t1,t0);
		ucode[STATE_R94] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,			//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R105, 1'b0, 7'd0 };

		//2^10 - 2^5: square(t0,t1);
		ucode[STATE_R105] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R100, 1'b0, 7'd0 };

		//2^10 - 2^0: mult(z2_10_0,t0,z2_5_0);
		ucode[STATE_R100] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_3,		//TEMP_3 is now z2_10_0
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R111, 1'b0, 7'd0 };

		//2^11 - 2^1: square(t0,z2_10_0);
		ucode[STATE_R111] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_3,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R122, 1'b0, 7'd0 };

		//2^12 - 2^2: square(t1,t0);
		ucode[STATE_R122] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2010_LOOP, 1'b0, 7'd0 };

		//2^12 - 2^2: for (i = 2;i < 10;i += 2) { square(t0,t1); square(t1,t0); }
		//We change this around a bit and square into t1 each iteration, and count by 1
		//since our loop algorithm doesn't allow multi-line loop bodies
		ucode[STATE_R2010_LOOP] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2010, 1'b1, 7'd10 };

		//2^20 - 2^0: mult(z2_20_0,t1,z2_10_0);
		ucode[STATE_R2010] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_3,		//TEMP_4 is now z2_20_0
			REG_ZERO, REG_ZERO, REG_TEMP_4, 3'b010, STATE_R200, 1'b0, 7'd0 };

		//2^21 - 2^1: square(t0,z2_20_0);
		ucode[STATE_R200] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_4, REG_TEMP_4,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R211, 1'b0, 7'd0 };

		//2^22 - 2^2: square(t1,t0);
		ucode[STATE_R211] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R4020_LOOP, 1'b0, 7'd0 };

		//2^40 - 2^20: for (i = 2;i < 20;i += 2) { square(t0,t1); square(t1,t0); }
		//Same as before, we do single multiplies instead of t1-t0-t1 pingponging
		ucode[STATE_R4020_LOOP] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R4020, 1'b1, 7'd20 };

		//2^40 - 2^0: mult(t0,t1,z2_20_0);
		ucode[STATE_R4020] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_4,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R400, 1'b0, 7'd0 };

		//2^41 - 2^1: square(t1,t0);
		ucode[STATE_R400] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R411, 1'b0, 7'd0 };

		//2^42 - 2^2: square(t0,t1);
		ucode[STATE_R411] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R5010_LOOP, 1'b0, 7'd0 };

		//2^50 - 2^10: for (i = 2;i < 10;i += 2) { square(t1,t0); square(t0,t1); }
		ucode[STATE_R5010_LOOP] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,	//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R5010, 1'b1, 7'd10 };

		//2^50 - 2^0: mult(z2_50_0,t0,z2_10_0);
		ucode[STATE_R5010] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_3,		//TEMP_3 is now z2_50_0
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R500, 1'b0, 7'd0 };

		//2^51 - 2^1: square(t0,z2_50_0);
		ucode[STATE_R500] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_3,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R522, 1'b0, 7'd0 };

		//2^52 - 2^2: square(t1,t0);
		ucode[STATE_R522] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R10050_LOOP, 1'b0, 7'd0 };

		//2^100 - 2^50: for (i = 2;i < 50;i += 2) { square(t0,t1); square(t1,t0); }
		ucode[STATE_R10050_LOOP] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R10050, 1'b1, 7'd50 };

		//2^100 - 2^0: mult(z2_100_0,t1,z2_50_0);
		ucode[STATE_R10050] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_3,		//TEMP_0 is now z2_100_0
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_R1000, 1'b0, 7'd0 };

		//2^101 - 2^1: square(t1,z2_100_0);
		ucode[STATE_R1000] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R1011, 1'b0, 7'd0 };

		//2^102 - 2^2: square(t0,t1);
		ucode[STATE_R1011] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R200100_LOOP, 1'b0, 7'd0 };

		//2^200 - 2^100: for (i = 2;i < 100;i += 2) { square(t1,t0); square(t0,t1); }
		ucode[STATE_R200100_LOOP] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R200100, 1'b1, 7'd100 };

		//2^200 - 2^0: mult(t1,t0,z2_100_0);
		ucode[STATE_R200100] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_0,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2000, 1'b0, 7'd0 };

		//2^201 - 2^1: square(t0,t1);
		ucode[STATE_R2000] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R2011, 1'b0, 7'd0 };

		//2^202 - 2^2: square(t1,t0);
		ucode[STATE_R2011] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R25050_LOOP, 1'b0, 7'd0 };

		//2^250 - 2^50: for (i = 2;i < 50;i += 2) { square(t0,t1); square(t1,t0); }
		ucode[STATE_R25050_LOOP] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R25050, 1'b1, 7'd50 };


	end

	microcode_t line;
	logic		advancing;
	logic		advancing_ff	= 0;

	logic[6:0]	loop_count		= 0;

	always_comb begin
		line 		= ucode[state];

		advancing	= (share_add_valid && line.next_on_add) ||
						(share_mult_valid && line.next_on_mult) ||
						(share_select_valid && line.next_on_select) ||
						out_valid;	//jump to STATE_RECIP
	end

	always_ff @(posedge clk) begin
		share_addsub_en	<= 0;
		share_select_en	<= 0;
		share_mult_en	<= 0;

		iter_out_valid	<= 0;

		advancing_ff	<= advancing;

		//Save output
		if(share_add_valid && (line.add_out <= REG_TEMP_5))
			temp_regs[line.add_out[2:0]]	<= share_add_out;
		if(share_sub_valid && (line.sub_out <= REG_TEMP_5))
			temp_regs[line.sub_out[2:0]]	<= share_sub_out;
		if(share_mult_valid && (line.mult_out <= REG_TEMP_5))
			temp_regs[line.mult_out[2:0]]	<= share_mult_out;
		if(share_select_valid && state == STATE_XN_HIGH)
			iter_out_valid	<= 1;

		//Move on to the next state
		if(advancing) begin
			if(out_valid)
				state		<= STATE_RECIP;
			else begin

				//Is this a loop? Go to the next iteration
				if(line.loop) begin

					//End of loop?
					if(loop_count+1 >= line.loop_max) begin
						state		<= line.next;
						loop_count	<= 2;
					end

					//Stay in current state, move to next iteration
					else
						loop_count	<= loop_count + 1;

				end

				//Nope, go to next state
				else begin
					state		<= line.next;
					loop_count	<= 2;
				end

			end
		end
		if(advancing_ff) begin

			//Enable blocks as needed
			share_select_en		<= line.select_en;
			share_addsub_en		<= line.addsub_en;
			share_mult_en		<= line.mult_en;

			//Selection always uses the same inputs
			share_select_r		<= {temp_regs[REG_TEMP_1][255:0], temp_regs[REG_TEMP_3][255:0]};
			share_select_s		<= {temp_regs[REG_TEMP_2][255:0], temp_regs[REG_TEMP_0][255:0]};

			//Special case a few "magic" inputs for constants etc.
			//Commented out paths are not used by current microcode.
			//Removing these mux paths saves a few LUTs. They can be added back if needed in the future.
			case(line.addsub_a)
				//REG_TEMP_0:		share_addsub_a	<= temp_regs[REG_TEMP_0];
				REG_TEMP_1:		share_addsub_a	<= temp_regs[REG_TEMP_1];
				//REG_TEMP_2:		share_addsub_a	<= temp_regs[REG_TEMP_2];
				REG_TEMP_3:		share_addsub_a	<= temp_regs[REG_TEMP_3];
				REG_TEMP_4:		share_addsub_a	<= temp_regs[REG_TEMP_4];
				//REG_TEMP_5:		share_addsub_a	<= temp_regs[REG_TEMP_5];

				REG_SELP_LO:	share_addsub_a	<= {8'h0, share_select_p[255:0]};
				REG_SELQ_LO:	share_addsub_a	<= {8'h0, share_select_q[255:0]};
			endcase

			case(line.addsub_b)
				REG_TEMP_0:		share_addsub_b	<= temp_regs[REG_TEMP_0];
				//REG_TEMP_1:		share_addsub_b	<= temp_regs[REG_TEMP_1];
				//REG_TEMP_2:		share_addsub_b	<= temp_regs[REG_TEMP_2];
				//REG_TEMP_3:		share_addsub_b	<= temp_regs[REG_TEMP_3];
				REG_TEMP_4:		share_addsub_b	<= temp_regs[REG_TEMP_4];
				REG_TEMP_5:		share_addsub_b	<= temp_regs[REG_TEMP_5];

				REG_SELP_HI:	share_addsub_b	<= {8'h0, share_select_p[511:256]};
				REG_SELQ_HI:	share_addsub_b	<= {8'h0, share_select_q[511:256]};
			endcase

			case(line.mult_a)
				REG_TEMP_0:		share_mult_a	<= temp_regs[REG_TEMP_0];
				REG_TEMP_1:		share_mult_a	<= temp_regs[REG_TEMP_1];
				REG_TEMP_2:		share_mult_a	<= temp_regs[REG_TEMP_2];
				REG_TEMP_3:		share_mult_a	<= temp_regs[REG_TEMP_3];
				REG_TEMP_4:		share_mult_a	<= temp_regs[REG_TEMP_4];
				//REG_TEMP_5:		share_mult_a	<= temp_regs[REG_TEMP_5];
				REG_WORK_HI:	share_mult_a	<= {8'h0, work_out[511:256]};
			endcase

			case(line.mult_b)
				REG_TEMP_0:		share_mult_b	<= temp_regs[REG_TEMP_0];
				REG_TEMP_1:		share_mult_b	<= temp_regs[REG_TEMP_1];
				REG_TEMP_2:		share_mult_b	<= temp_regs[REG_TEMP_2];
				REG_TEMP_3:		share_mult_b	<= temp_regs[REG_TEMP_3];
				REG_TEMP_4:		share_mult_b	<= temp_regs[REG_TEMP_4];
				REG_TEMP_5:		share_mult_b	<= temp_regs[REG_TEMP_5];
				REG_121665:		share_mult_b	<= 264'd121665;
				REG_WORK_LOW:	share_mult_b	<= {8'h0, work_in};
				REG_WORK_HI:	share_mult_b	<= {8'h0, work_out[511:256]};
			endcase
		end

		//Special case initialization
		//select(xzmb,xzm1b,xzm,xzm1,b);
		if(iter_en) begin
			share_select_en				<= 1;
			if(iter_first) begin
				share_select_r			<= 512'h1;
				share_select_s			<= {256'h1, work_in};
			end
			else begin
				share_select_r			<= share_select_p;
				share_select_s			<= share_select_q;
			end
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sequencing

	logic[7:0] round = 0;

	enum logic[2:0]
	{
		LSTATE_IDLE,
		LSTATE_MLSTART,
		LSTATE_MLWAIT,
		LSTATE_MLDONE
	} loopstate = LSTATE_IDLE;

	always_ff @(posedge clk) begin

		iter_en		<= 0;
		iter_first	<= 0;
		out_valid	<= 0;

		case(loopstate)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// mainloop()

			LSTATE_IDLE: begin

				//When starting a new multiply, go from the highest bit
				if(en) begin
					iter_en			<= 1;
					iter_first		<= 1;
					round			<= 254;
					b				<= e[254];
					loopstate		<= LSTATE_MLWAIT;
				end

			end	//end STATE_IDLE

			LSTATE_MLSTART: begin
				b					<= e[round];
				iter_en				<= 1;
				loopstate			<= LSTATE_MLWAIT;
			end	//end LSTATE_MLSTART

			LSTATE_MLWAIT: begin
				if(iter_out_valid) begin
					round			<= round - 1;

					if(round == 0)
						loopstate	<= LSTATE_MLDONE;
					else
						loopstate	<= LSTATE_MLSTART;

				end
			end	//end STATE_MLWAIT

			LSTATE_MLDONE: begin
				out_valid	<= 1;
				work_out	<= share_select_p;
				loopstate	<= LSTATE_IDLE;
			end	//end STATE_MLDONE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// recip()

		endcase

	end

endmodule
