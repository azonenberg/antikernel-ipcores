`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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

	Typical area (Kintex-7)_
		9315 LUT, 8559 FF, 32 DSP

		Refactored for hierarchy and separate muxes
			9415 LUT, 8775 FF, 32 DSP
			of which
				MultPass_stage1 = 2713 LUT
				MultPass_stage2 = 256 LUT
				MultMuxing = 520 LUT

		Full keep hierarchy
			9420 LUT, 8813 FF, 32 DSP
			of which
				Top level = 3233 LUT, 4460 FF
				Add = 275 LUT, 531 FF
				Mult = 5385 LUT, 2781 FF
					MultPass_stage1 = 18 LUT
					MultPass_stage2 = 2697 LUT
					MultMuxing = 520 LUT, 518 FF
					ReductionAdder stage3 512 LUT
					ReductionAdder stage4 128 LUT
					ReductionAdder stage5 32 LUT
					StreamingSqueeze 146 LUT, 846 FF

		Optimized mult muxing
			9158 LUT, 8816 FF
			of which
				MultMuxing = 261 LUT, 518 FF

			With flattened hierarchy
				9060 LUT, 8542 FF, 32 DSP

	Run time performance:
		crypto_scalarmult (ECDH): 563438 clocks

	Typical achievable performance:
		250 MHz in Kintex-7, -2 speed
			crypto_scalarmult (ECDH):	2.24 ms / 2.25 ms after
			scalarmult (ECDSA):			xx
 */
module X25519_ScalarMult(
	input wire			clk,

	//Common inputs
	input wire[255:0]	e,				//outer loop variable

	//ECDH interface
	input wire			dh_en,
	input wire[255:0]	work_in,

	//ECDSA interface

	//Common outputs
	output wire			out_valid,
	output wire[255:0]	work_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loop contents

	logic			dh_iter_en		= 0;
	logic			iter_first		= 0;
	logic			b				= 0;

	logic			iter_out_valid	= 0;

	logic[511:0]	ml_work_out		= 0;
	logic			ml_out_valid	= 0;

	logic[7:0] 		round = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Apply bit twiddling to "e" for ECDH

	logic[255:0]	e_fixed;
	always_comb begin
		e_fixed			= e;

		//e[0] &= 248;
		e_fixed[2:0]	= 0;

		//e[31] &= 127;
		e_fixed[255]	= 0;

		//e[31] |= 64;
		e_fixed[254]	= 1;
	end

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
	logic[255:0]	share_select_r	= 0;
	logic[255:0]	share_select_s	= 0;
	wire[255:0]		share_select_p;
	wire[255:0]		share_select_q;
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

	//freeze is a no-op with our representation
	logic			share_freeze_en	= 0;
	logic[263:0]	share_freeze_a;
	assign out_valid = share_freeze_en;
	assign work_out = {1'b0, share_freeze_a[254:0]};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main loop

	//going to have to bump this up to one more address bit
	typedef enum logic[6:0]
	{
		//mainloop(): 18 lines
		STATE_ECDH_START_FIRST,
		STATE_ECDH_SELECT_FIRST,
		STATE_ECDH_START,
		STATE_ECDH_SELECT,
		STATE_SELECT_DONE,
		STATE_A0,
		STATE_B0_LOW,
		STATE_B0_HIGH,
		STATE_B1_LOW,
		STATE_B1_HIGH,
		STATE_C1,
		STATE_R,
		STATE_T,
		STATE_XB_LOW,
		STATE_XB_HIGH,
		STATE_XN_LOW,
		STATE_XN_HIGH,
		STATE_XN_HIGH2,

		//recip(): 43 lines
		STATE_RECIP,
		STATE_R4,
		STATE_R8,
		STATE_R9,
		STATE_R11,
		STATE_R22,
		STATE_R31,
		STATE_R61,
		STATE_R72,
		STATE_R83,
		STATE_R94,
		STATE_R105,
		STATE_R100,
		STATE_R111,
		STATE_R122,
		STATE_R2010_LOOP,
		STATE_R2010,
		STATE_R200,
		STATE_R211,
		STATE_R4020_LOOP,
		STATE_R4020,
		STATE_R400,
		STATE_R411,
		STATE_R5010_LOOP,
		STATE_R5010,
		STATE_R500,
		STATE_R522,
		STATE_R10050_LOOP,
		STATE_R10050,
		STATE_R1000,
		STATE_R1011,
		STATE_R200100_LOOP,
		STATE_R200100,
		STATE_R2000,
		STATE_R2011,
		STATE_R25050_LOOP,
		STATE_R25050,
		STATE_R2500,
		STATE_R2511,
		STATE_R2522,
		STATE_R2533,
		STATE_R2544,
		STATE_R2555,

		//crypto_scalarmult(): 1 line
		STATE_FINAL_MULT,

		//completion: 2 lines
		STATE_ITER_DONE,
		STATE_DONE,

		STATE_MAX
	} state_t;

	state_t state = STATE_DONE;

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
		REG_TEMP_6		= 4'h06,
		REG_TEMP_7		= 4'h07,
		REG_TEMP_8		= 4'h08,
		REG_TEMP_9		= 4'h09,

		//Special registers (named, but not always usable in every operation)
		REG_121665		= 4'h0a,	//always 121665
		REG_ZERO		= 4'h0b,	//throw away unused values
		REG_ONE			= 4'h0c,
		REG_WORK_LOW	= 4'h0d,
		REG_WORK_HI		= 4'h0e		//must be last
	} regid_t;
	logic[263:0]	temp_regs[REG_TEMP_9:0];

	//Initialize registers, including constants
	initial begin

		for(integer i=0; i<=REG_TEMP_9; i++)
			temp_regs[i]		<= 0;

	end

	//Microcode definitions
	typedef struct packed
	{
		//inputs
		logic		select_en;
		logic		addsub_en;
		logic		mult_en;
		regid_t		addsub_a;
		regid_t		addsub_b;
		regid_t		mult_a;
		regid_t		mult_b;

		/////

		//new block for selection
		regid_t		select_r;	//inputs
		regid_t		select_s;
		regid_t		select_p;	//outputs
		regid_t		select_q;

		/////

		//outputs
		regid_t		add_out;
		regid_t		sub_out;
		regid_t		mult_out;

		//control flow
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
		// crypto_scalarmult MAINLOOP ENTRY

		//First iteration
		ucode[STATE_ECDH_START_FIRST] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ONE, REG_WORK_LOW, REG_TEMP_6, REG_TEMP_7,	// p/q low
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_ECDH_SELECT_FIRST, 1'b0, 7'd0 };

		ucode[STATE_ECDH_SELECT_FIRST] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ONE, REG_TEMP_8, REG_TEMP_9,	// p/q high
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SELECT_DONE, 1'b0, 7'd0 };

		//Subsequent iterations
		ucode[STATE_ECDH_START] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_6, REG_TEMP_7, REG_TEMP_6, REG_TEMP_7,	// p/q low
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_ECDH_SELECT, 1'b0, 7'd0 };

		ucode[STATE_ECDH_SELECT] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_8, REG_TEMP_9, REG_TEMP_8, REG_TEMP_9,	// p/q high
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SELECT_DONE, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// crypto_scalarmult MAINLOOP

		//add(a0,xzmb,xzmb + 32);
		//sub(a0 + 32,xzmb,xzmb + 32);
		ucode[STATE_SELECT_DONE] = { 3'b010, REG_TEMP_6, REG_TEMP_8, REG_ZERO, REG_ZERO, 	//TEMP_0 is now A0_LO
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_TEMP_0, REG_TEMP_1, REG_ZERO, 3'b100, STATE_A0, 1'b0, 7'd0 };				//TEMP_1 is now A0_HI

		//add(a1,xzm1b,xzm1b + 32);
		//sub(a1 + 32,xzm1b,xzm1b + 32);
		//square(b0,a0);
		ucode[STATE_A0] = { 3'b011, REG_TEMP_7, REG_TEMP_9, REG_TEMP_0, REG_TEMP_0,		//TEMP_2 is now A1_LO
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_TEMP_2, REG_TEMP_3, REG_TEMP_4, 3'b010, STATE_B0_LOW, 1'b0, 7'd0 };			//TEMP_3 is now A1_HI
																							//TEMP_4 is now B0_LO

		//square(b0 + 32,a0 + 32);
		ucode[STATE_B0_LOW] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,			//TEMP_5 is now B0_HI
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_5, 3'b010, STATE_B0_HIGH, 1'b0, 7'd0 };

		//mult(b1,a1,a0 + 32);
		ucode[STATE_B0_HIGH] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_1,	//last use of TEMP_1 as A0_HI
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_B1_LOW, 1'b0, 7'd0 };		//TEMP_1 is now B1_LO

		//mult(b1 + 32,a1 + 32,a0);
		ucode[STATE_B1_LOW] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_0,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_B1_HIGH, 1'b0, 7'd0 };	//TEMP_0 is now B1_HI

		//add(c1,b1,b1 + 32);
		//sub(c1 + 32,b1,b1 + 32);
		ucode[STATE_B1_HIGH] = { 3'b010, REG_TEMP_1, REG_TEMP_0, REG_ZERO, REG_ZERO, //TEMP_0 is now C1_LO
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_TEMP_0, REG_TEMP_1, REG_ZERO, 3'b100, STATE_C1, 1'b0, 7'd0 };		//TEMP_1 is now C1_HI

		//sub(s,b0,b0 + 32);
		//square(r,c1 + 32);
		ucode[STATE_C1] = { 3'b011, REG_TEMP_4, REG_TEMP_5, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is now S
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_TEMP_1, REG_TEMP_2, 3'b010, STATE_R, 1'b0, 7'd0 };		//TEMP_2 is now R

		//mult121665(t,s);
		ucode[STATE_R] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_121665,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_T, 1'b0, 7'd0 };			//TEMP_3 is now T

		//add(u,t,b0);
		//mult(xznb,b0,b0 + 32);
		ucode[STATE_T] = { 3'b011, REG_TEMP_3, REG_TEMP_4, REG_TEMP_4, REG_TEMP_5,	//TEMP_3 is now XZNB_LO
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_TEMP_4, REG_ZERO, REG_TEMP_3, 3'b010, STATE_XB_LOW, 1'b0, 7'd0 };	//TEMP_4 is now U

		//mult(xznb + 32,s,u);
		ucode[STATE_XB_LOW] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_4,	//TEMP_1 is now XZNB_HI
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_XB_HIGH, 1'b0, 7'd0 };

		//square(xzn1b,c1);
		ucode[STATE_XB_HIGH] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0, //TEMP_0 is now XZN1B_LO
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_XN_LOW, 1'b0, 7'd0 };

		//mult(xzn1b + 32,r,work);
		ucode[STATE_XN_LOW] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_WORK_LOW,//TEMP_2 is now XZN1B_HI
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_XN_HIGH, 1'b0, 7'd0 };

		//select(xzm,xzm1,xznb,xzn1b,b);
		ucode[STATE_XN_HIGH] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_3, REG_TEMP_0, REG_TEMP_6, REG_TEMP_7,	// p/q low
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_XN_HIGH2, 1'b0, 7'd0 };

		ucode[STATE_XN_HIGH2] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_1, REG_TEMP_2, REG_TEMP_8, REG_TEMP_9,	// p/q high
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_ITER_DONE, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// crypto_scalarmult RECIP

		//2: square(z2,z);
		ucode[STATE_RECIP] = { 3'b001, REG_ZERO, REG_ZERO, REG_WORK_HI, REG_WORK_HI,	//TEMP_0 is now z2
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_R4, 1'b0, 7'd0 };

		//4: square(t1,z2);
		ucode[STATE_R4] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0,			//TEMP_1 is now t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R8, 1'b0, 7'd0 };

		//8: square(t0,t1);
		ucode[STATE_R8] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,			//TEMP_2 is now t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R9, 1'b0, 7'd0 };

		//9: mult(z9,t0,z);
		ucode[STATE_R9] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_WORK_HI,			//TEMP_3 is now z9
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R11, 1'b0, 7'd0 };

		//11: mult(z11,z9,z2);
		ucode[STATE_R11] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_0,			//TEMP_0 is now z11
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_R22, 1'b0, 7'd0 };

		//22: square(t0,z11);
		ucode[STATE_R22] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0,			//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R31, 1'b0, 7'd0 };

		//31: mult(z2_5_0,t0,z9);
		ucode[STATE_R31] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_3,			//TEMP_3 is now z2_5_0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R61, 1'b0, 7'd0 };

		//2^6 - 2^1: square(t0,z2_5_0);
		ucode[STATE_R61] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_3,			//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R72, 1'b0, 7'd0 };

		//2^7 - 2^2: square(t1,t0);
		ucode[STATE_R72] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,			//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R83, 1'b0, 7'd0 };

		//2^8 - 2^3: square(t0,t1);
		ucode[STATE_R83] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,			//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R94, 1'b0, 7'd0 };

		//2^9 - 2^4: square(t1,t0);
		ucode[STATE_R94] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,			//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R105, 1'b0, 7'd0 };

		//2^10 - 2^5: square(t0,t1);
		ucode[STATE_R105] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R100, 1'b0, 7'd0 };

		//2^10 - 2^0: mult(z2_10_0,t0,z2_5_0);
		ucode[STATE_R100] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_3,		//TEMP_3 is now z2_10_0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R111, 1'b0, 7'd0 };

		//2^11 - 2^1: square(t0,z2_10_0);
		ucode[STATE_R111] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_3,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R122, 1'b0, 7'd0 };

		//2^12 - 2^2: square(t1,t0);
		ucode[STATE_R122] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2010_LOOP, 1'b0, 7'd0 };

		//2^12 - 2^2: for (i = 2;i < 10;i += 2) { square(t0,t1); square(t1,t0); }
		//We change this around a bit and square into t1 each iteration, and count by 1
		//since our loop algorithm doesn't allow multi-line loop bodies
		ucode[STATE_R2010_LOOP] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2010, 1'b1, 7'd10 };

		//2^20 - 2^0: mult(z2_20_0,t1,z2_10_0);
		ucode[STATE_R2010] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_3,		//TEMP_4 is now z2_20_0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_4, 3'b010, STATE_R200, 1'b0, 7'd0 };

		//2^21 - 2^1: square(t0,z2_20_0);
		ucode[STATE_R200] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_4, REG_TEMP_4,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R211, 1'b0, 7'd0 };

		//2^22 - 2^2: square(t1,t0);
		ucode[STATE_R211] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R4020_LOOP, 1'b0, 7'd0 };

		//2^40 - 2^20: for (i = 2;i < 20;i += 2) { square(t0,t1); square(t1,t0); }
		//Same as before, we do single multiplies instead of t1-t0-t1 pingponging
		ucode[STATE_R4020_LOOP] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R4020, 1'b1, 7'd20 };

		//2^40 - 2^0: mult(t0,t1,z2_20_0);
		ucode[STATE_R4020] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_4,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R400, 1'b0, 7'd0 };

		//2^41 - 2^1: square(t1,t0);
		ucode[STATE_R400] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R411, 1'b0, 7'd0 };

		//2^42 - 2^2: square(t0,t1);
		ucode[STATE_R411] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R5010_LOOP, 1'b0, 7'd0 };

		//2^50 - 2^10: for (i = 2;i < 10;i += 2) { square(t1,t0); square(t0,t1); }
		ucode[STATE_R5010_LOOP] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,	//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R5010, 1'b1, 7'd10 };

		//2^50 - 2^0: mult(z2_50_0,t0,z2_10_0);
		ucode[STATE_R5010] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_3,		//TEMP_3 is now z2_50_0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_R500, 1'b0, 7'd0 };

		//2^51 - 2^1: square(t0,z2_50_0);
		ucode[STATE_R500] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_3,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R522, 1'b0, 7'd0 };

		//2^52 - 2^2: square(t1,t0);
		ucode[STATE_R522] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R10050_LOOP, 1'b0, 7'd0 };

		//2^100 - 2^50: for (i = 2;i < 50;i += 2) { square(t0,t1); square(t1,t0); }
		ucode[STATE_R10050_LOOP] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R10050, 1'b1, 7'd50 };

		//2^100 - 2^0: mult(z2_100_0,t1,z2_50_0);
		ucode[STATE_R10050] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_3,		//TEMP_4 is now z2_100_0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_4, 3'b010, STATE_R1000, 1'b0, 7'd0 };

		//2^101 - 2^1: square(t1,z2_100_0);
		ucode[STATE_R1000] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_4, REG_TEMP_4,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R1011, 1'b0, 7'd0 };

		//2^102 - 2^2: square(t0,t1);
		ucode[STATE_R1011] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R200100_LOOP, 1'b0, 7'd0 };

		//2^200 - 2^100: for (i = 2;i < 100;i += 2) { square(t1,t0); square(t0,t1); }
		ucode[STATE_R200100_LOOP] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R200100, 1'b1, 7'd100 };

		//2^200 - 2^0: mult(t1,t0,z2_100_0);
		ucode[STATE_R200100] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_4,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2000, 1'b0, 7'd0 };

		//2^201 - 2^1: square(t0,t1);
		ucode[STATE_R2000] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R2011, 1'b0, 7'd0 };

		//2^202 - 2^2: square(t1,t0);
		ucode[STATE_R2011] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R25050_LOOP, 1'b0, 7'd0 };

		//2^250 - 2^50: for (i = 2;i < 50;i += 2) { square(t0,t1); square(t1,t0); }
		ucode[STATE_R25050_LOOP] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R25050, 1'b1, 7'd50 };

		//2^250 - 2^0: mult(t0,t1,z2_50_0);
		ucode[STATE_R25050] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_3,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R2500, 1'b0, 7'd0 };

		//2^251 - 2^1: square(t1,t0);
		ucode[STATE_R2500] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2511, 1'b0, 7'd0 };

		//2^252 - 2^2: square(t0,t1);
		ucode[STATE_R2511] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R2522, 1'b0, 7'd0 };

		//2^253 - 2^3: square(t1,t0);
		ucode[STATE_R2522] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2533, 1'b0, 7'd0 };

		//2^254 - 2^4: square(t0,t1);
		ucode[STATE_R2533] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,		//TEMP_2 is still t0
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_R2544, 1'b0, 7'd0 };

		//2^255 - 2^5: square(t1,t0);
		ucode[STATE_R2544] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_2,		//TEMP_1 is still t1
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_R2555, 1'b0, 7'd0 };

		//2^255 - 21: mult(out,t1,z11);
		ucode[STATE_R2555] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_0,		//TEMP_0 is now out
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_FINAL_MULT, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// crypto_scalarmult top

		//mult(work + 64,work,work + 32);
		ucode[STATE_FINAL_MULT] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_6,	//TEMP_0 is now work+64
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_DONE, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/// end

		ucode[STATE_DONE] = { 3'b000, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b000, STATE_DONE, 1'b0, 7'd0 };

		ucode[STATE_ITER_DONE] = { 3'b000, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b000, STATE_ITER_DONE, 1'b0, 7'd0 };

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
						ml_out_valid;	//jump to STATE_RECIP

		share_freeze_a	= temp_regs[REG_TEMP_0];
	end

	always_ff @(posedge clk) begin
		share_addsub_en	<= 0;
		share_select_en	<= 0;
		share_mult_en	<= 0;
		share_freeze_en	<= 0;

		iter_out_valid	<= 0;

		advancing_ff	<= advancing;

		//Save output
		if(share_add_valid && (line.add_out <= REG_TEMP_5))
			temp_regs[line.add_out]		<= share_add_out;
		if(share_sub_valid && (line.sub_out <= REG_TEMP_5))
			temp_regs[line.sub_out]		<= share_sub_out;
		if(share_mult_valid && (line.mult_out <= REG_TEMP_5))
			temp_regs[line.mult_out]	<= share_mult_out;
		if(share_select_valid) begin
			temp_regs[line.select_p]	<= share_select_p;
			temp_regs[line.select_q]	<= share_select_q;
		end

		if(share_select_valid && state == STATE_XN_HIGH2)
			iter_out_valid	<= 1;

		//Move on to the next state
		if(advancing) begin
			if(ml_out_valid)
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

			//Special case freeze() since it's only used for the output
			share_freeze_en		<= (state == STATE_DONE);

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
				REG_TEMP_6:		share_addsub_a	<= temp_regs[REG_TEMP_6];
				REG_TEMP_7:		share_addsub_a	<= temp_regs[REG_TEMP_7];
			endcase

			case(line.addsub_b)
				REG_TEMP_0:		share_addsub_b	<= temp_regs[REG_TEMP_0];
				//REG_TEMP_1:		share_addsub_b	<= temp_regs[REG_TEMP_1];
				//REG_TEMP_2:		share_addsub_b	<= temp_regs[REG_TEMP_2];
				//REG_TEMP_3:		share_addsub_b	<= temp_regs[REG_TEMP_3];
				REG_TEMP_4:		share_addsub_b	<= temp_regs[REG_TEMP_4];
				REG_TEMP_5:		share_addsub_b	<= temp_regs[REG_TEMP_5];
				REG_TEMP_8:		share_addsub_b	<= temp_regs[REG_TEMP_8];
				REG_TEMP_9:		share_addsub_b	<= temp_regs[REG_TEMP_9];
			endcase

			case(line.mult_a)
				REG_TEMP_0:		share_mult_a	<= temp_regs[REG_TEMP_0];
				REG_TEMP_1:		share_mult_a	<= temp_regs[REG_TEMP_1];
				REG_TEMP_2:		share_mult_a	<= temp_regs[REG_TEMP_2];
				REG_TEMP_3:		share_mult_a	<= temp_regs[REG_TEMP_3];
				REG_TEMP_4:		share_mult_a	<= temp_regs[REG_TEMP_4];
				//REG_TEMP_5:		share_mult_a	<= temp_regs[REG_TEMP_5];
				REG_WORK_HI:	share_mult_a	<= {8'h0, ml_work_out[511:256]};
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
				REG_WORK_HI:	share_mult_b	<= {8'h0, ml_work_out[511:256]};
				REG_TEMP_6:		share_mult_b	<= temp_regs[REG_TEMP_6];
			endcase

			case(line.select_r)
				REG_ZERO:		share_select_r	<= 256'h0;
				REG_ONE:		share_select_r	<= 256'h1;
				REG_TEMP_1:		share_select_r	<= temp_regs[REG_TEMP_1];
				REG_TEMP_3:		share_select_r	<= temp_regs[REG_TEMP_3];
				REG_TEMP_6:		share_select_r	<= temp_regs[REG_TEMP_6];
				REG_TEMP_8:		share_select_r	<= temp_regs[REG_TEMP_8];
			endcase

			case(line.select_s)
				REG_WORK_LOW:	share_select_s	<= {8'h0, work_in};
				REG_ONE:		share_select_s	<= 256'h1;
				REG_TEMP_0:		share_select_s	<= temp_regs[REG_TEMP_0];
				REG_TEMP_2:		share_select_s	<= temp_regs[REG_TEMP_2];
				REG_TEMP_7:		share_select_s	<= temp_regs[REG_TEMP_7];
				REG_TEMP_9:		share_select_s	<= temp_regs[REG_TEMP_9];
			endcase

			if(line.addsub_en && (round == 'hfe) ) begin
				$display("Starting add/sub (state %d)", state);
				if(!line.mult_en) begin
					$display("    r0 = %x", temp_regs[REG_TEMP_0]);
					$display("    r1 = %x", temp_regs[REG_TEMP_1]);
					$display("    r2 = %x", temp_regs[REG_TEMP_2]);
					$display("    r3 = %x", temp_regs[REG_TEMP_3]);
					$display("    r4 = %x", temp_regs[REG_TEMP_4]);
					$display("    r5 = %x", temp_regs[REG_TEMP_5]);
					$display("    r6 = %x", temp_regs[REG_TEMP_6]);
					$display("    r7 = %x", temp_regs[REG_TEMP_7]);
					$display("    r8 = %x", temp_regs[REG_TEMP_8]);
					$display("    r9 = %x", temp_regs[REG_TEMP_9]);
				end
			end

			if(line.mult_en && (round == 'hfe) ) begin
				$display("Starting multiply (state %d)", state);
				$display("    r0 = %x", temp_regs[REG_TEMP_0]);
				$display("    r1 = %x", temp_regs[REG_TEMP_1]);
				$display("    r2 = %x", temp_regs[REG_TEMP_2]);
				$display("    r3 = %x", temp_regs[REG_TEMP_3]);
				$display("    r4 = %x", temp_regs[REG_TEMP_4]);
				$display("    r5 = %x", temp_regs[REG_TEMP_5]);
				$display("    r6 = %x", temp_regs[REG_TEMP_6]);
				$display("    r7 = %x", temp_regs[REG_TEMP_7]);
				$display("    r8 = %x", temp_regs[REG_TEMP_8]);
				$display("    r9 = %x", temp_regs[REG_TEMP_9]);
			end

		end

		//Special case initialization
		//select(xzmb,xzm1b,xzm,xzm1,b);
		if(dh_iter_en) begin
			advancing_ff	<= 1;
			if(iter_first)
				state		<= STATE_ECDH_START_FIRST;
			else
				state 		<= STATE_ECDH_START;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sequencing

	enum logic[2:0]
	{
		LSTATE_IDLE,
		LSTATE_MLSTART,
		LSTATE_MLWAIT,
		LSTATE_MLDONE
	} loopstate = LSTATE_IDLE;

	always_ff @(posedge clk) begin

		dh_iter_en		<= 0;
		iter_first		<= 0;
		ml_out_valid	<= 0;

		if(share_mult_valid && (round < 5))
			$display("Multiply complete: %x", share_mult_out);
		if(share_add_valid && (round < 5))
			$display("Add complete: %x", share_add_out);
		if(share_sub_valid && (round < 5))
			$display("Sub complete: %x", share_sub_out);

		case(loopstate)

			LSTATE_IDLE: begin

				//When starting a new crypto_scalarmult(), go from the highest bit
				if(dh_en) begin
					dh_iter_en		<= 1;
					iter_first		<= 1;
					round			<= 254;
					b				<= e_fixed[254];
					loopstate		<= LSTATE_MLWAIT;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// mainloop()

			LSTATE_MLSTART: begin
				b					<= e_fixed[round];
				dh_iter_en			<= 1;
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
				ml_out_valid	<= 1;
				ml_work_out		<= {temp_regs[REG_TEMP_8][255:0], temp_regs[REG_TEMP_6][255:0]};
				loopstate		<= LSTATE_IDLE;
			end	//end STATE_MLDONE

		endcase

	end

endmodule
