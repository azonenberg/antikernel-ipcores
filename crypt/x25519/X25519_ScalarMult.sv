`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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

	Typical area:
		REGFILE_OUT_REG = 0
			Kintex-7: 7632 LUT, 5737 FF, 32 DSP
			Trion: 21625 LUT, 11528 FF, 96 MULT (30644 LE total)
				Limited in large part by lack of LUTRAM...
				TODO consider optional synchronous regfile to enable BRAM packing
			Titanium: 21246 LUT4, 2350 ADD, 96 DSP48, 11488 FF, 1 SRL8, 31746 XLR

		REGFILE_OUT_REG = 1
			Kintex-7: 7421 LUT, 1408 LUTRAM, 6774 FF, 32 DSP
			Trion: 5156 LUT, 2798 ADD, 6297 FF, 96 MULT, 112 RAM5K, 9471 LE
			Titanium: 5159 LUT, 2349 ADD, 6240 FF, 96 DSP, 112 RAM10, 8976 XLR
			Topaz: 5159 LUT, 2349 ADD, 6240 FF, 96 DSP48, 112 RAM10, 8976 XLR

	Run time (constant cycle count):
		REGFILE_OUT_REG = 0:
			crypto_scalarmult (ECDH): 567786 clocks
			scalarmult (ECDSA):       957287 clocks
		REGFILE_OUT_REG = 1:
			TODO

	Typical achievable performance:
		REGFILE_OUT_REG = 0
			Kintex-7, -2 speed: 250 MHz
				crypto_scalarmult (ECDH):	2.27 ms
				scalarmult (ECDSA):			3.90 ms
			Trion T55, C4 speed: 59 MHz
			Titanium 375, C4 speed: 177 MHz

		REGFILE_OUT_REG = 1
			Trion T35, C4 speed: 96 MHz
			Topaz TZ50, C2 speed: 171 MHz
			Titanium 375, C4 speed: 288 MHz

	To do a crypto_scalarmult():
		assert dh_en with e/work_in valid

	To do a scalarmult():
		Load q0...q3
			Assert dsa_load with dsa_addr set to index of q and work_in set to the input data
		Start the operation
			assert dsa_en
		Wait for out_valid to pulse high
		Read results
			assert dsa_rd then read corresponding result word from work_out

	To do a scalarbase():
		Load q0...q1
			Same as scalarmult()
		Start the operation
			assert dsa_base_en
		same as scalarmult() from here
 */
import Curve25519Registers::*;
module X25519_ScalarMult #(
	parameter REGFILE_OUT_REG = 0
)(
	input wire			clk,

	//Common inputs
	input wire[255:0]	e,				//outer loop variable (e for crypto_scalarmult, s for scalarmult)
	input wire[255:0]	work_in,		//Input data bus
										//work for crypto_scalarmult
										//q[] for scalarmult

	//ECDH interface
	input wire			dh_en,

	//ECDSA interface
	input wire			dsa_en,
	input wire			dsa_base_en,
	input wire			dsa_load,
	input wire			dsa_rd,
	output logic		dsa_done,
	input wire[1:0]		dsa_addr,

	//Common outputs
	output logic		out_valid	`ifdef XILINX = 0 `endif,
	output wire[255:0]	work_out
);

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		out_valid = 0;
	end
	`endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loop contents

	logic			dh_iter_en		= 0;
	logic			dsa_iter_en		= 0;
	logic			iter_first		= 0;
	logic			b				= 0;

	logic			iter_out_valid	= 0;

	logic			ml_out_valid	= 0;

	logic[7:0] 		round 			= 0;

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
	// Execution units

	logic		share_addsub_en	= 0;
	regval_t	share_addsub_a;
	regval_t	share_addsub_b;
	wire		share_add_valid;
	wire[263:0]	share_add_out;

	X25519_Add share_add(
		.clk(clk),
		.en(share_addsub_en),
		.a(share_addsub_a),
		.b(share_addsub_b),
		.out_valid(share_add_valid),
		.out(share_add_out)
	);

	wire		share_sub_valid;
	wire[263:0]	share_sub_out;

	X25519_Sub share_sub(
		.clk(clk),
		.en(share_addsub_en),
		.a(share_addsub_a),
		.b(share_addsub_b),
		.out_valid(share_sub_valid),
		.out(share_sub_out)
	);

	logic		share_select_en	= 0;
	wire[255:0]	share_select_p;
	wire[255:0]	share_select_q;
	wire		share_select_valid;

	X25519_Select share_select(
		.clk(clk),
		.en(share_select_en),
		.p(share_select_p),
		.q(share_select_q),
		.r(share_mult_a[255:0]),
		.s(share_mult_b[255:0]),
		.b(b),
		.out_valid(share_select_valid));

	logic		share_mult_en	= 0;
	regval_t	share_mult_a;
	regval_t	share_mult_b;
	wire[263:0]	share_mult_out;
	wire		share_mult_valid;

	X25519_Mult share_mult(
		.clk(clk),
		.en(share_mult_en),
		.a(share_mult_a),
		.b(share_mult_b),
		.out_valid(share_mult_valid),
		.out(share_mult_out));

	//freeze is a no-op with our representation
	logic			share_freeze_en	= 0;
	regval_t		share_freeze_a;
	assign work_out = {1'b0, share_freeze_a[254:0]};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main loop

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

		//scalarmult() entry: 3 lines
		STATE_DSA_INIT1,
		STATE_DSA_INIT2,
		STATE_DSA_INIT3,

		//scalarmult() loop part 1: 4 lines
		STATE_SCALARMULT_FIRST_SEL1,
		STATE_SCALARMULT_FIRST_SEL2,
		STATE_SCALARMULT_FIRST_SEL3,
		STATE_SCALARMULT_FIRST_SEL4,

		//first add(): 9 lines
		STATE_ADD_FIRST_1,
		STATE_ADD_FIRST_2,
		STATE_ADD_FIRST_3,
		STATE_ADD_FIRST_4,
		STATE_ADD_FIRST_5,
		STATE_ADD_FIRST_6,
		STATE_ADD_FIRST_7,
		STATE_ADD_FIRST_8,
		STATE_ADD_FIRST_9,

		//second add(): 9 lines
		STATE_ADD_SECOND_1,
		STATE_ADD_SECOND_2,
		STATE_ADD_SECOND_3,
		STATE_ADD_SECOND_4,
		STATE_ADD_SECOND_5,
		STATE_ADD_SECOND_6,
		STATE_ADD_SECOND_7,
		STATE_ADD_SECOND_8,
		STATE_ADD_SECOND_9,

		//scalarmult() loop part 2: 4 lines
		STATE_SCALARMULT_SECOND_SEL1,
		STATE_SCALARMULT_SECOND_SEL2,
		STATE_SCALARMULT_SECOND_SEL3,
		STATE_SCALARMULT_SECOND_SEL4,

		//scalarbase() setup: x lines
		STATE_SCALARBASE_INIT1,

		//completion: 2 lines
		STATE_ITER_DONE,
		STATE_DONE,

		STATE_MAX
	} state_t;

	state_t state = STATE_DONE;

	//Microcode definitions
	typedef struct packed
	{
		//inputs
		logic		select_en;
		logic		addsub_en;
		logic		mult_en;
		xregid_t	addsub_a;
		xregid_t	addsub_b;
		xregid_t	mult_a;
		xregid_t	mult_b;

		/////

		//new block for selection
		xregid_t	select_r;	//inputs
		xregid_t	select_s;
		xregid_t	select_p;	//outputs
		xregid_t	select_q;

		/////

		//outputs
		xregid_t	add_out;
		xregid_t	sub_out;
		xregid_t	mult_out;

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
			REG_ONE, REG_TEMP_10, REG_TEMP_6, REG_TEMP_7,	// p/q low
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
		ucode[STATE_XN_LOW] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_10,//TEMP_2 is now XZN1B_HI
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

		//TEMP_8 is arbitrarily work_high

		//2: square(z2,z);
		ucode[STATE_RECIP] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_8, REG_TEMP_8,	//TEMP_0 is now z2
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
		ucode[STATE_R9] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_8,			//TEMP_3 is now z9
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
		// scalarmult one-time init (first iteration)

		//At entry: temp[1:0] are q[1:0] (x/y point coordinates)

		//Load constant inputs
		//set25519(q[2],gf1);
		//bignumMult(q[3],X,Y);
		ucode[STATE_DSA_INIT1] = { 3'b011, REG_ONE, REG_ZERO, REG_TEMP_0, REG_TEMP_1,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_2, REG_ZERO, REG_TEMP_3, 3'b010, STATE_DSA_INIT2, 1'b0, 7'd0 };

		//set25519(out[0], gf0)
		//set25519(out[1], gf1)
		ucode[STATE_DSA_INIT2] = { 3'b110, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,		//TEMP_4 is out[0] = 0
			REG_ONE, REG_ONE, REG_TEMP_5, REG_ZERO,										//TEMP_5 is out[1] = 1
			REG_TEMP_4, REG_ZERO, REG_ZERO, 3'b100, STATE_DSA_INIT3, 1'b0, 7'd0 };

		//set25519(out[2],gf1);
		//set25519(out[3],gf0);
		ucode[STATE_DSA_INIT3] = { 3'b110, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,		//TEMP_6 is out[2] = 1
			REG_ONE, REG_ONE, REG_TEMP_6, REG_ZERO,										//TEMP_7 is out[3] = 0
			REG_TEMP_7, REG_ZERO, REG_ZERO, 3'b100, STATE_SCALARMULT_FIRST_SEL1, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// scalarmult selection before add()

		/*
			q[3:0]   = TEMP_3:0
			out[3:0] = TEMP_7:4
		 */

		//sel25519(out[0], q[0], b);
		ucode[STATE_SCALARMULT_FIRST_SEL1] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_0, REG_TEMP_4, REG_TEMP_0, REG_TEMP_4,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SCALARMULT_FIRST_SEL2, 1'b0, 7'd0 };

		//sel25519(out[1], q[1], b);
		ucode[STATE_SCALARMULT_FIRST_SEL2] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_1, REG_TEMP_5, REG_TEMP_1, REG_TEMP_5,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SCALARMULT_FIRST_SEL3, 1'b0, 7'd0 };

		//sel25519(out[2], q[2], b);
		ucode[STATE_SCALARMULT_FIRST_SEL3] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_2, REG_TEMP_6, REG_TEMP_2, REG_TEMP_6,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SCALARMULT_FIRST_SEL4, 1'b0, 7'd0 };

		//sel25519(out[3], q[3], b);
		ucode[STATE_SCALARMULT_FIRST_SEL4] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_3, REG_TEMP_7, REG_TEMP_3, REG_TEMP_7,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_ADD_FIRST_1, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// scalarmult add() when called with p=q[], q=out[], pout=q[]

		/*
			Register assignments
			p[3:0]   = TEMP_3:0
			q[3:0]   = TEMP_7:4
			out[3:0] = TEMP_3:0
			a = TEMP_8
			b = TEMP_9
			c = TEMP_10
		 */

		//bignumAddSub(b, a, p1, p0);
		//bignumMult(pout[3], p3, q3);
		ucode[STATE_ADD_FIRST_1] = {3'b011, REG_TEMP_1, REG_TEMP_0, REG_TEMP_3, REG_TEMP_7,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_9, REG_TEMP_8, REG_TEMP_3, 3'b010, STATE_ADD_FIRST_2, 1'b0, 7'd0 };

		//bignumAddSub(pout[1], pout[0], q1, q0);
		//bignumMult(pout[2], p2, q2);
		ucode[STATE_ADD_FIRST_2] = {3'b011, REG_TEMP_5, REG_TEMP_4, REG_TEMP_2, REG_TEMP_6,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_1, REG_TEMP_0, REG_TEMP_2, 3'b010, STATE_ADD_FIRST_3, 1'b0, 7'd0 };

		//bignumAdd(c, pout[2], pout[2]);
		//bignumMult(pout[3], pout[3], D2);
		ucode[STATE_ADD_FIRST_3] = {3'b011, REG_TEMP_2, REG_TEMP_2, REG_TEMP_3, REG_D2,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_10, REG_ZERO, REG_TEMP_3, 3'b010, STATE_ADD_FIRST_4, 1'b0, 7'd0 };

		//bignumAddSub(c, pout[3], c, pout[3]);
		//bignumMult(pout[1], b, pout[1]);
		ucode[STATE_ADD_FIRST_4] = {3'b011, REG_TEMP_10, REG_TEMP_3, REG_TEMP_9, REG_TEMP_1,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_10, REG_TEMP_3, REG_TEMP_1, 3'b010, STATE_ADD_FIRST_5, 1'b0, 7'd0 };

		//bignumMult(pout[0], a, pout[0]);
		ucode[STATE_ADD_FIRST_5] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_8, REG_TEMP_0,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_ADD_FIRST_6, 1'b0, 7'd0 };

		//bignumAddSub(b, pout[1], pout[1], pout[0]);
		//bignumMult(pout[2], c, pout[3]);
		ucode[STATE_ADD_FIRST_6] = {3'b011, REG_TEMP_1, REG_TEMP_0, REG_TEMP_3, REG_TEMP_10,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_9, REG_TEMP_1, REG_TEMP_2, 3'b010, STATE_ADD_FIRST_7, 1'b0, 7'd0 };

		//bignumMult(pout[0], pout[1], pout[3]);
		ucode[STATE_ADD_FIRST_7] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_3,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_ADD_FIRST_8, 1'b0, 7'd0 };

		//bignumMult(pout[3], pout[1], b);
		ucode[STATE_ADD_FIRST_8] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_9,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_ADD_FIRST_9, 1'b0, 7'd0 };

		//bignumMult(pout[1], b, c);
		ucode[STATE_ADD_FIRST_9] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_9, REG_TEMP_10,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_ADD_SECOND_1, 1'b0, 7'd0 };

		/*
			After first add, in the top level scalarmult() loop
			q is TEMP[3:0]
			out is TEMP[7:4]
		 */

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Now we call add() again

		/*
			Register assignments
				out=out, p=out, q=out

			In other words
				p[] = temp[7:4]
				q[] = temp[7:4]
				out[] = temp[7:4]

				and top level q is in 3:0 so we're not allowed to touch those

				a = TEMP_8
				b = TEMP_9
				c = TEMP_10
		 */

		//bignumAddSub(b, a, p1, p0);
		//bignumMult(pout[3], p3, q3);
		ucode[STATE_ADD_SECOND_1] = {3'b011, REG_TEMP_5, REG_TEMP_4, REG_TEMP_7, REG_TEMP_7,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_9, REG_TEMP_8, REG_TEMP_7, 3'b010, STATE_ADD_SECOND_2, 1'b0, 7'd0 };

		//bignumAddSub(pout[1], pout[0], q1, q0);
		//bignumMult(pout[2], p2, q2);
		ucode[STATE_ADD_SECOND_2] = {3'b011, REG_TEMP_5, REG_TEMP_4, REG_TEMP_6, REG_TEMP_6,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_5, REG_TEMP_4, REG_TEMP_6, 3'b010, STATE_ADD_SECOND_3, 1'b0, 7'd0 };

		//bignumAdd(c, pout[2], pout[2]);
		//bignumMult(pout[3], pout[3], D2);
		ucode[STATE_ADD_SECOND_3] = {3'b011, REG_TEMP_6, REG_TEMP_6, REG_TEMP_7, REG_D2,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_10, REG_ZERO, REG_TEMP_7, 3'b010, STATE_ADD_SECOND_4, 1'b0, 7'd0 };

		//bignumAddSub(c, pout[3], c, pout[3]);
		//bignumMult(pout[1], b, pout[1]);
		ucode[STATE_ADD_SECOND_4] = {3'b011, REG_TEMP_10, REG_TEMP_7, REG_TEMP_9, REG_TEMP_5,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_10, REG_TEMP_7, REG_TEMP_5, 3'b010, STATE_ADD_SECOND_5, 1'b0, 7'd0 };

		//bignumMult(pout[0], a, pout[0]);
		ucode[STATE_ADD_SECOND_5] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_8, REG_TEMP_4,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_4, 3'b010, STATE_ADD_SECOND_6, 1'b0, 7'd0 };

		//bignumAddSub(b, pout[1], pout[1], pout[0]);
		//bignumMult(pout[2], c, pout[3]);
		ucode[STATE_ADD_SECOND_6] = {3'b011, REG_TEMP_5, REG_TEMP_4, REG_TEMP_10, REG_TEMP_7,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_9, REG_TEMP_5, REG_TEMP_6, 3'b010, STATE_ADD_SECOND_7, 1'b0, 7'd0 };

		//bignumMult(pout[0], pout[1], pout[3]);
		ucode[STATE_ADD_SECOND_7] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_5, REG_TEMP_7,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_4, 3'b010, STATE_ADD_SECOND_8, 1'b0, 7'd0 };

		//bignumMult(pout[3], pout[1], b);
		ucode[STATE_ADD_SECOND_8] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_5, REG_TEMP_9,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_7, 3'b010, STATE_ADD_SECOND_9, 1'b0, 7'd0 };

		//bignumMult(pout[1], b, c);
		ucode[STATE_ADD_SECOND_9] = { 3'b001, REG_ZERO, REG_ZERO, REG_TEMP_9, REG_TEMP_10,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_TEMP_5, 3'b010, STATE_SCALARMULT_SECOND_SEL1, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// scalarmult selection after add()

		/*
			at top level
			q[] = temp[3:0]
			out = temp[7:4]
		 */

		//sel25519(out[0], q[0], b);
		ucode[STATE_SCALARMULT_SECOND_SEL1] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_0, REG_TEMP_4, REG_TEMP_0, REG_TEMP_4,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SCALARMULT_SECOND_SEL2, 1'b0, 7'd0 };

		//sel25519(out[1], q[1], b);
		ucode[STATE_SCALARMULT_SECOND_SEL2] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_1, REG_TEMP_5, REG_TEMP_1, REG_TEMP_5,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SCALARMULT_SECOND_SEL3, 1'b0, 7'd0 };

		//sel25519(out[2], q[2], b);
		ucode[STATE_SCALARMULT_SECOND_SEL3] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_2, REG_TEMP_6, REG_TEMP_2, REG_TEMP_6,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SCALARMULT_SECOND_SEL4, 1'b0, 7'd0 };

		//sel25519(out[3], q[3], b);
		ucode[STATE_SCALARMULT_SECOND_SEL4] = { 3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_3, REG_TEMP_7, REG_TEMP_3, REG_TEMP_7,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_ITER_DONE, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// scalarbase() initialization before jumping to scalarmult()

		//TODO: do this part too
		//set25519(q[0],X);

		//Load constant inputs
		//set25519(q[1],Y);
		ucode[STATE_SCALARBASE_INIT1] = { 3'b010, REG_Y, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_TEMP_1, REG_ZERO, REG_ZERO, 3'b100, STATE_DSA_INIT1, 1'b0, 7'd0 };

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/// end

		ucode[STATE_DONE] = { 3'b000, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b000, STATE_DONE, 1'b0, 7'd0 };

		ucode[STATE_ITER_DONE] = { 3'b000, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO, //no select
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b000, STATE_ITER_DONE, 1'b0, 7'd0 };

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The register file

	logic	want_add_result		= 0;
	logic	want_sub_result		= 0;
	logic	want_mult_result	= 0;
	logic	want_select_result	= 0;

	(* KEEP = "true" *)
	xregid_t add_rd_ff 		= REG_TEMP_0;

	(* KEEP = "true" *)
	xregid_t sub_rd_ff 		= REG_TEMP_0;

	(* KEEP = "true" *)
	xregid_t mult_rd_ff 	= REG_TEMP_0;

	(* KEEP = "true" *)
	xregid_t select_p_rd_ff = REG_TEMP_0;

	(* KEEP = "true" *)
	xregid_t select_q_rd_ff = REG_TEMP_0;

	xregid_t	addsub_a_ff	= REG_TEMP_0;
	xregid_t	addsub_b_ff	= REG_TEMP_0;
	xregid_t	mult_a_ff	= REG_TEMP_0;
	xregid_t	mult_b_ff	= REG_TEMP_0;
	xregid_t	select_r_ff	= REG_TEMP_0;
	xregid_t	select_s_ff	= REG_TEMP_0;

	logic		advancing;
	logic		advancing_ff		= 0;
	logic		advancing_ff2		= 0;

	logic		regfile_rd_en;
	logic		regfile_rd_en_ff	= 0;
	wire		regfile_rd_valid;

	always_comb begin
		regfile_rd_en	= advancing_ff2;
	end

	always_ff @(posedge clk) begin

		if(advancing_ff) begin
			addsub_a_ff		<= line.addsub_a;
			addsub_b_ff		<= line.addsub_b;
			mult_a_ff		<= line.mult_a;
			mult_b_ff		<= line.mult_b;
			select_r_ff		<= line.select_r;
			select_s_ff		<= line.select_s;
		end

		advancing_ff		<= advancing || dh_iter_en || dsa_iter_en;
		advancing_ff2		<= advancing_ff;
		regfile_rd_en_ff	<= regfile_rd_en;
	end

	microcode_t line;

	(* keep_hierarchy = "yes" *)
	X25519_Regfile #(
		.REGFILE_OUT_REG(REGFILE_OUT_REG)
	) regfile (
		.clk(clk),

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Writes

		.share_add_valid(share_add_valid),
		.share_sub_valid(share_sub_valid),
		.share_mult_valid(share_mult_valid),
		.share_select_valid(share_select_valid),

		.share_add_out(share_add_out),
		.share_sub_out(share_sub_out),
		.share_mult_out(share_mult_out),
		.share_select_p({8'h0, share_select_p}),
		.share_select_q({8'h0, share_select_q}),
		.work_in({8'h0, work_in}),

		.dh_en(dh_en),
		.dsa_load(dsa_load),
		.dsa_addr(dsa_addr),
		.dsa_rd(dsa_rd),

		.want_add_result(want_add_result),
		.want_sub_result(want_sub_result),
		.want_mult_result(want_mult_result),
		.want_select_result(want_select_result),

		.add_rd_ff(add_rd_ff),
		.sub_rd_ff(sub_rd_ff),
		.mult_rd_ff(mult_rd_ff),
		.select_p_rd_ff(select_p_rd_ff),
		.select_q_rd_ff(select_q_rd_ff),

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Reads

		.rd_en(regfile_rd_en),
		.rd_valid(regfile_rd_valid),
		.share_freeze_en(share_freeze_en),

		.addsub_a_regid(addsub_a_ff),
		.addsub_b_regid(addsub_b_ff),
		.mult_a_regid(mult_a_ff),
		.mult_b_regid(mult_b_ff),
		.select_r_regid(select_r_ff),
		.select_s_regid(select_s_ff),

		.share_addsub_a(share_addsub_a),
		.share_addsub_b(share_addsub_b),
		.share_mult_a(share_mult_a),
		.share_mult_b(share_mult_b),
		.share_freeze_a(share_freeze_a)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Top level control

	logic[6:0]	loop_count		= 0;

	always_comb begin
		line 		= ucode[state];

		advancing	= (share_add_valid && line.next_on_add) ||
						(share_mult_valid && line.next_on_mult) ||
						(share_select_valid && line.next_on_select) ||
						ml_out_valid;	//jump to STATE_RECIP
	end

	logic		load_base		= 0;

	always_ff @(posedge clk) begin
		share_addsub_en		<= 0;
		share_select_en		<= 0;
		share_mult_en		<= 0;
		share_freeze_en		<= 0;

		iter_out_valid		<= 0;

		out_valid 			<= share_freeze_en || dsa_rd;

		//Save flags indicating whether output should be processed
		want_add_result		<= (line.add_out <= REG_TEMP_10);
		want_sub_result		<= (line.sub_out <= REG_TEMP_10);
		want_mult_result	<= (line.mult_out <= REG_TEMP_10);
		want_select_result	<= (line.select_p <= REG_TEMP_10);

		//Pipeline indexes for output registers
		add_rd_ff			<= line.add_out;
		sub_rd_ff			<= line.sub_out;
		mult_rd_ff			<= line.mult_out;
		select_p_rd_ff		<= line.select_p;
		select_q_rd_ff		<= line.select_q;

		if(share_select_valid && ( (state == STATE_XN_HIGH2) || (state == STATE_SCALARMULT_SECOND_SEL4) ) )
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

		if(regfile_rd_valid) begin

			//Enable blocks as needed
			share_select_en		<= line.select_en;
			share_addsub_en		<= line.addsub_en;
			share_mult_en		<= line.mult_en;

			//Special case freeze() since it's only used for the output
			share_freeze_en		<= (state == STATE_DONE);

		end

		//Special case initialization
		//select(xzmb,xzm1b,xzm,xzm1,b);
		if(dh_iter_en) begin
			if(iter_first)
				state		<= STATE_ECDH_START_FIRST;
			else
				state 		<= STATE_ECDH_START;
		end

		if(dsa_iter_en) begin

			//scalarbase one-time init
			if(load_base)
				state		<= STATE_SCALARBASE_INIT1;

			//one time initialization
			else if(iter_first)
				state		<= STATE_DSA_INIT1;

			//no jump right into loop entry
			else
				state 		<= STATE_SCALARMULT_FIRST_SEL1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sequencing

	enum logic[2:0]
	{
		LSTATE_IDLE,

		//crypto_scalarmult
		LSTATE_MLSTART,
		LSTATE_MLWAIT,
		LSTATE_MLDONE,

		//scalarmult
		LSTATE_SCALARSTART,
		LSTATE_SCALARMULT,
		LSTATE_SCALARDONE
	} loopstate = LSTATE_IDLE;

	always_ff @(posedge clk) begin

		dh_iter_en		<= 0;
		dsa_iter_en		<= 0;
		iter_first		<= 0;
		ml_out_valid	<= 0;
		dsa_done		<= 0;
		load_base		<= 0;

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

				//When starting a new scalarmult(), go from the highest bit
				//but don't twiddle bits like crypto_scalarmult() needs
				if(dsa_en || dsa_base_en) begin
					dsa_iter_en		<= 1;
					iter_first		<= 1;
					load_base		<= dsa_base_en;
					round			<= 255;
					b				<= e[255];
					loopstate		<= LSTATE_SCALARMULT;
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

				loopstate		<= LSTATE_IDLE;
			end	//end STATE_MLDONE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// scalarmult()

			LSTATE_SCALARSTART: begin
				b					<= e[round];
				dsa_iter_en			<= 1;
				loopstate			<= LSTATE_SCALARMULT;
			end	//end LSTATE_SCALARSTART

			LSTATE_SCALARMULT: begin

				if(iter_out_valid) begin
					round			<= round - 1;

					if(round == 0) begin
						loopstate	<= LSTATE_SCALARDONE;
					end
					else
						loopstate	<= LSTATE_SCALARSTART;
				end

			end

			LSTATE_SCALARDONE: begin
				dsa_done		<= 1;
				loopstate		<= LSTATE_IDLE;
			end	//end STATE_SCALARDONE

		endcase

	end

	/*
	//DEBUG
	ila_0 ila(
		.clk(clk),
		.probe0(share_addsub_en),
		.probe1(share_mult_en),
		.probe2(regfile_rd_en),
		.probe3(regfile_rd_valid),
		.probe4(share_add_out),
		.probe5(share_freeze_en),
		.probe6(share_freeze_a),

		.probe7(regfile.rd_en_ff),
		.probe8(regfile.rd_valid_adv),
		.probe9(regfile.share_freeze_en),
		.probe10(regfile.freeze_en_ff),
		.probe11(regfile.freeze_valid),
		.probe12(regfile.p0_rd_data),
		.probe13(share_addsub_a)
	);*/

endmodule

