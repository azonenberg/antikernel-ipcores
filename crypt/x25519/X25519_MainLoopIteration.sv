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

	typedef enum logic[3:0]
	{
		STATE_IDLE			= 4'h0,
		STATE_SELECT_DONE	= 4'h1,
		STATE_A0			= 4'h2,
		STATE_B0_LOW		= 4'h3,
		STATE_B0_HIGH		= 4'h4,
		STATE_B1_LOW		= 4'h5,
		STATE_B1_HIGH		= 4'h6,
		STATE_C1			= 4'h7,
		STATE_R				= 4'h8,
		STATE_T				= 4'h9,
		STATE_XB_LOW		= 4'ha,
		STATE_XB_HIGH		= 4'hb,
		STATE_XN_LOW		= 4'hc,
		STATE_XN_HIGH		= 4'hd,
		STATE_MAX
	} state_t;

	state_t state = STATE_IDLE;

	//Memory for temporary variables
	typedef enum logic[3:0]
	{
		REG_TEMP_0		= 4'h00,
		REG_TEMP_1		= 4'h01,
		REG_TEMP_2		= 4'h02,
		REG_TEMP_3		= 4'h03,
		REG_B0_LO		= 4'h04,
		REG_B0_HI		= 4'h05,
		REG_U			= 4'h06,

		//None of these are writable
		REG_XZMB_LO		= 4'h07,	//4x select outputs
		REG_XZMB_HI		= 4'h08,
		REG_XZM1B_LO	= 4'h09,
		REG_XZM1B_HI	= 4'h0a,
		REG_121665		= 4'h0b,	//always 121665
		REG_ZERO		= 4'h0c,	//always 0
		REG_WORK_LOW	= 4'h0d		//must be last
	} regid_t;
	logic[263:0]	temp_regs[REG_WORK_LOW-1:0];

	//Initialize registers, including constants
	initial begin

		for(integer i=0; i<REG_WORK_LOW; i++)
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
	} microcode_t;

	microcode_t[STATE_MAX-1:0] ucode;
	initial begin

		//Filler
		for(integer i=0; i<STATE_MAX; i++)
			ucode[i] = {$bits(microcode_t){1'b0}};

		//Idle
		ucode[STATE_IDLE] = { 3'b000, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_SELECT_DONE };

		//add(a0,xzmb,xzmb + 32);
		//sub(a0 + 32,xzmb,xzmb + 32);
		ucode[STATE_SELECT_DONE] = { 3'b010, REG_XZMB_LO, REG_XZMB_HI, REG_ZERO, REG_ZERO,	//TEMP_0 is now A0_LO
			REG_TEMP_0, REG_TEMP_1, REG_ZERO, 3'b100, STATE_A0 };							//TEMP_1 is now A0_HI

		//add(a1,xzm1b,xzm1b + 32);
		//sub(a1 + 32,xzm1b,xzm1b + 32);
		//square(b0,a0);
		ucode[STATE_A0] = {3'b011, REG_XZM1B_LO, REG_XZM1B_HI, REG_TEMP_0, REG_TEMP_0,		//TEMP_2 is now A1_LO
			REG_TEMP_2, REG_TEMP_3, REG_B0_LO, 3'b010, STATE_B0_LOW };						//TEMP_3 is now A1_HI

		//square(b0 + 32,a0 + 32);
		ucode[STATE_B0_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_TEMP_1,
			REG_ZERO, REG_ZERO, REG_B0_HI, 3'b010, STATE_B0_HIGH };

		//mult(b1,a1,a0 + 32);
		ucode[STATE_B0_HIGH] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_TEMP_1,	//last use of TEMP_1 as A0_HI
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_B1_LOW };					//TEMP_1 is now B1_LO

		//mult(b1 + 32,a1 + 32,a0);
		ucode[STATE_B1_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_3, REG_TEMP_0,
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_B1_HIGH };				//TEMP_0 is now B1_HI

		//add(c1,b1,b1 + 32);
		//sub(c1 + 32,b1,b1 + 32);
		ucode[STATE_B1_HIGH] = {3'b010, REG_TEMP_1, REG_TEMP_0, REG_ZERO, REG_ZERO, //TEMP_0 is now C1_LO
			REG_TEMP_0, REG_TEMP_1, REG_ZERO, 3'b100, STATE_C1 };					//TEMP_1 is now C1_HI

		//sub(s,b0,b0 + 32);
		//square(r,c1 + 32);
		ucode[STATE_C1] = {3'b011, REG_B0_LO, REG_B0_HI, REG_TEMP_1, REG_TEMP_1,	//TEMP_1 is now S
			REG_ZERO, REG_TEMP_1, REG_TEMP_2, 3'b010, STATE_R };					//TEMP_2 is now R

		//mult121665(t,s);
		ucode[STATE_R] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_121665,
			REG_ZERO, REG_ZERO, REG_TEMP_3, 3'b010, STATE_T };						//TEMP_3 is now T

		//add(u,t,b0);
		//mult(xznb,b0,b0 + 32);
		ucode[STATE_T] = {3'b011, REG_TEMP_3, REG_B0_LO, REG_B0_LO, REG_B0_HI,
			REG_U, REG_ZERO, REG_TEMP_3, 3'b010, STATE_XB_LOW };					//TEMP_3 is now XZNB_LO

		//mult(xznb + 32,s,u);
		ucode[STATE_XB_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_1, REG_U,		//TEMP_1 is now XZNB_HI
			REG_ZERO, REG_ZERO, REG_TEMP_1, 3'b010, STATE_XB_HIGH };

		//square(xzn1b,c1);
		ucode[STATE_XB_HIGH] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_0, REG_TEMP_0, //TEMP_0 is now XZN1B_LO
			REG_ZERO, REG_ZERO, REG_TEMP_0, 3'b010, STATE_XN_LOW };

		//mult(xzn1b + 32,r,work);
		ucode[STATE_XN_LOW] = {3'b001, REG_ZERO, REG_ZERO, REG_TEMP_2, REG_WORK_LOW,//TEMP_2 is now XZN1B_HI
			REG_ZERO, REG_ZERO, REG_TEMP_2, 3'b010, STATE_XN_HIGH };

		//select(xzm,xzm1,xznb,xzn1b,b);
		ucode[STATE_XN_HIGH] = {3'b100, REG_ZERO, REG_ZERO, REG_ZERO, REG_ZERO,
			REG_ZERO, REG_ZERO, REG_ZERO, 3'b001, STATE_IDLE };

	end

	microcode_t line;
	logic		advancing;
	logic		advancing_ff	= 0;

	always_comb begin
		line 		= ucode[state];

		advancing	= (share_add_valid && line.next_on_add) ||
						(share_mult_valid && line.next_on_mult) ||
						(share_select_valid && line.next_on_select);
	end

	always_ff @(posedge clk) begin
		share_addsub_en	<= 0;
		share_select_en	<= 0;
		share_mult_en	<= 0;

		out_valid		<= 0;

		advancing_ff	<= advancing;

		//Save output
		if(share_add_valid && (line.add_out <= REG_U))
			temp_regs[line.add_out[2:0]]	<= share_add_out;
		if(share_sub_valid && (line.sub_out <= REG_U))
			temp_regs[line.sub_out[2:0]]	<= share_sub_out;
		if(share_mult_valid && (line.mult_out <= REG_U))
			temp_regs[line.mult_out[2:0]]	<= share_mult_out;
		if(share_select_valid) begin
			if(state == STATE_IDLE) begin
				temp_regs[REG_XZMB_LO]		<= {8'h0, share_select_p[255:0]};
				temp_regs[REG_XZMB_HI]		<= {8'h0, share_select_p[511:256]};
				temp_regs[REG_XZM1B_LO]		<= {8'h0, share_select_q[255:0]};
				temp_regs[REG_XZM1B_HI]		<= {8'h0, share_select_q[511:256]};
			end
			else if(state == STATE_XN_HIGH) begin
				xzm_out			<= share_select_p;
				xzm1_out		<= share_select_q;
				out_valid		<= 1;
			end
		end

		//Move on to the next state
		if(advancing)
			state		<= line.next;
		if(advancing_ff) begin
			share_select_en		<= line.select_en;
			share_select_r		<= {temp_regs[REG_TEMP_1][255:0], temp_regs[REG_TEMP_3][255:0]};
			share_select_s		<= {temp_regs[REG_TEMP_2][255:0], temp_regs[REG_TEMP_0][255:0]};
			share_addsub_en		<= line.addsub_en;
			share_addsub_a		<= temp_regs[line.addsub_a[3:0]];
			share_addsub_b		<= temp_regs[line.addsub_b[3:0]];
			share_mult_en		<= line.mult_en;
			share_mult_a		<= temp_regs[line.mult_a[3:0]];

			case(line.mult_b)
				REG_121665:		share_mult_b	<= 264'd121665;
				REG_WORK_LOW:	share_mult_b	<= work_low;
				default:		share_mult_b	<= temp_regs[line.mult_b];
			endcase
		end

		//Special case initialization
		//select(xzmb,xzm1b,xzm,xzm1,b);
		if(en) begin
			share_select_en			<= 1;
			share_select_r			<= xzm_in;
			share_select_s			<= xzm1_in;
		end

	end

endmodule
