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

import Curve25519Registers::*;

//enable this to turn on KEEP_HIERARCHY for better area feedback during optimization
//turn off to enable flattening and improve performance/area
//`define FORCE_HIERARCHY

/**
	@file
	@author Andrew D. Zonenberg
	@brief X25519 multiplication

	Derived from mult() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)
 */
module X25519_MultPass #(
	parameter MULT_AREA_OPT		= 0		//0 = default
)(
	input wire			clk,
	input wire			en,
	input wire[4:0]		i,
	input wire bignum_t	a,
	input wire bignum_t	b,
	output wire			out_valid,
	output wire[31:0]	out
	);

	wire				stage3_en;
	bignum32_t			stage3_tmp;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if(MULT_AREA_OPT == 0) begin
		//Initial version: stage1/2
		X25519_MultPass_stage12_areaopt0 stage12(
			.clk(clk),
			.en(en),
			.i(i),
			.a(a),
			.b(b),
			.stage3_en(stage3_en),
			.stage3_tmp(stage3_tmp)
		);
	end

	else if(MULT_AREA_OPT == 1) begin
		X25519_MultPass_stage12_areaopt1 stage12(
			.clk(clk),
			.en(en),
			.i(i),
			.a(a),
			.b(b),
			.stage3_en(stage3_en),
			.stage3_tmp(stage3_tmp)
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Adder tree for the 32 temporary values. Do fanin of 4 per pipeline stage.

	wire		stage4_en;
	wire[255:0]	stage4_tmp;

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	ReductionAdder #(
		.IN_WORD_WIDTH(32),
		.OUT_WORD_WIDTH(32),
		.IN_BLOCKS(32),
		.REDUCTION(4)
	) stage3 (
		.clk(clk),
		.en(stage3_en),
		.din(stage3_tmp),
		.dout_valid(stage4_en),
		.dout(stage4_tmp)
	);

	wire		stage5_en;
	wire[63:0]	stage5_tmp;

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	ReductionAdder #(
		.IN_WORD_WIDTH(32),
		.OUT_WORD_WIDTH(32),
		.IN_BLOCKS(8),
		.REDUCTION(4)
	) stage4 (
		.clk(clk),
		.en(stage4_en),
		.din(stage4_tmp),
		.dout_valid(stage5_en),
		.dout(stage5_tmp)
	);

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	ReductionAdder #(
		.IN_WORD_WIDTH(32),
		.OUT_WORD_WIDTH(32),
		.IN_BLOCKS(2),
		.REDUCTION(2)
	) stage5 (
		.clk(clk),
		.en(stage5_en),
		.din(stage5_tmp),
		.dout_valid(out_valid),
		.dout(out)
	);

endmodule

/**
	@brief X25519_MultPass_stage1 and X25519_MultPass_stage2 with AREA_OPT=0
 */
module X25519_MultPass_stage12_areaopt0(
	input wire			clk,
	input wire			en,
	input wire[4:0]		i,
	input wire bignum_t	a,
	input wire bignum_t	b,

	output wire			stage3_en,
	output bignum32_t	stage3_tmp
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First stage of multiplication

	wire				stage2_en;
	wire[31:0]			stage2_do38;
	bignum32_t			stage2_tmp;

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	X25519_MultPass_stage1 stage1(
		.clk(clk),
		.en(en),
		.i(i),
		.a(a),
		.b(b),
		.stage2_en(stage2_en),
		.stage2_do38(stage2_do38),
		.stage2_tmp(stage2_tmp)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second stage of multiplication (only for one side of the diagonal)

	`ifdef FORCE_HIERARCHY
	(* keep_hierarchy = "yes" *)
	`endif
	X25519_MultPass_stage2 stage2(
		.clk(clk),
		.stage2_en(stage2_en),
		.stage2_do38(stage2_do38),
		.stage2_tmp(stage2_tmp),
		.stage3_en(stage3_en),
		.stage3_tmp(stage3_tmp)
	);

endmodule

/**
	@brief X25519_MultPass_stage1 and X25519_MultPass_stage2 with AREA_OPT=1
 */
module X25519_MultPass_stage12_areaopt1(
	input wire			clk,
	input wire			en,
	input wire[4:0]		i,
	input wire bignum_t	a,
	input wire bignum_t	b,

	output logic			stage3_en	`ifdef XILINX = 0 `endif,
	output bignum32_t		stage3_tmp	`ifdef XILINX = 0 `endif
);

	logic		stage2_en	= 0;
	logic[31:0]	stage2_do38 = 0;
	bignum32_t	stage2_tmp	= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First stage of multiplication

	always_ff @(posedge clk) begin

		stage2_en	<= en;

		if(en) begin
			for(integer j=0; j<32; j=j+1)
				stage2_do38[j]			<= (j > i);
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual multipliers

	for(genvar g=0; g<32; g=g+1) begin
		always_ff @(posedge clk) begin
			if(en)
				stage2_tmp.blocks[g]	<= a.blocks[g] * b.blocks[g];
		end
	end

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		stage3_en = 0;
		stage3_tmp = 0;
	end
	`endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second multiplication stage

	always_ff @(posedge clk) begin
		stage3_en	<= stage2_en;

		if(stage2_en) begin
			for(integer j=0; j<32; j=j+1) begin
				if(stage2_do38[j])
					stage3_tmp.blocks[j]	<= stage2_tmp.blocks[j] * 38;
				else
					stage3_tmp.blocks[j]	<= stage2_tmp.blocks[j];
			end
		end

	end

end

/**
	@brief Helper for X25519_MultPass
 */
(* USE_DSP="yes" *)
module X25519_MultPass_stage1(
	input wire			clk,
	input wire			en,
	input wire[4:0]		i,
	input wire bignum_t	a,
	input wire bignum_t	b,

	output logic		stage2_en		`ifdef XILINX = 0 `endif,
	output logic[31:0]	stage2_do38		`ifdef XILINX = 0 `endif,
	output bignum32_t	stage2_tmp		`ifdef XILINX = 0 `endif
	);

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		stage2_en = 0;
		stage2_do38 = 0;
	end
	`endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First stage of multiplication

	always_ff @(posedge clk) begin

		stage2_en	<= en;

		if(en) begin
			for(integer j=0; j<32; j=j+1)
				stage2_do38[j]			<= (j > i);
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual multipliers

	for(genvar g=0; g<32; g=g+1) begin
		always_ff @(posedge clk) begin
			if(en)
				stage2_tmp.blocks[g]	<= a.blocks[g] * b.blocks[g];
		end
	end

endmodule

/**
	@brief Helper for X25519_MultPass

	x38 is only 3 bits (32 + 4 + 2) so makes sense to save DSPs for other stuff
 */
(* USE_DSP="no" *)	//this constraint is xilinx specific, still use hard multipliers on efinix platforms
module X25519_MultPass_stage2(
	input wire				clk,

	input wire				stage2_en,
	input wire[31:0]		stage2_do38,
	input wire bignum32_t	stage2_tmp,

	output logic			stage3_en	`ifdef XILINX = 0 `endif,
	output bignum32_t		stage3_tmp	`ifdef XILINX = 0 `endif
	);

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		stage3_en = 0;
		stage3_tmp = 0;
	end
	`endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second multiplication stage

	always_ff @(posedge clk) begin
		stage3_en	<= stage2_en;

		if(stage2_en) begin
			for(integer j=0; j<32; j=j+1) begin
				if(stage2_do38[j])
					stage3_tmp.blocks[j]	<= stage2_tmp.blocks[j] * 38;
				else
					stage3_tmp.blocks[j]	<= stage2_tmp.blocks[j];
			end
		end

	end

endmodule
