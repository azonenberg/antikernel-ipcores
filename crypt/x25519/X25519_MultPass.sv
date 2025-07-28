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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Top level multiplication

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

	//debug
	wire				stage3_en_ref;
	bignum32_t			stage3_tmp_ref;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if(MULT_AREA_OPT == 0) begin : optsel0
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

	else if(MULT_AREA_OPT == 1) begin : optsel1
		X25519_MultPass_stage12_areaopt1 stage12(
			.clk(clk),
			.en(en),
			.i(i),
			.a(a),
			.b(b),
			.stage3_en(stage3_en),
			.stage3_tmp(stage3_tmp));

		//uncomment to verify optimization level 0 against 1
		X25519_MultPass_stage12_areaopt0 stage12_knowngood(
			.clk(clk),
			.en(en),
			.i(i),
			.a(a),
			.b(b),
			.stage3_en(stage3_en_ref),
			.stage3_tmp(stage3_tmp_ref));

		//Sanity check here
		always_ff @(posedge clk) begin
			if(stage3_en !== stage3_en_ref) begin
				$display("en mismatch");
				$finish;
			end
			if(stage3_en) begin
				if(stage3_tmp !== stage3_tmp_ref) begin
					$display("data mismatch");
					$finish;
				end
			end
		end

	end

	else if(MULT_AREA_OPT == 2) begin : optsel2
		X25519_MultPass_stage12_areaopt2 stage12(
			.clk(clk),
			.en(en),
			.i(i),
			.a(a),
			.b(b),
			.stage3_en(stage3_en),
			.stage3_tmp(stage3_tmp));

		//uncomment to verify optimization level 0 against 2
		X25519_MultPass_stage12_areaopt0 stage12_knowngood(
			.clk(clk),
			.en(en),
			.i(i),
			.a(a),
			.b(b),
			.stage3_en(stage3_en_ref),
			.stage3_tmp(stage3_tmp_ref));

		//Sanity check here
		always_ff @(posedge clk) begin
			if(stage3_en !== stage3_en_ref) begin
				$display("en mismatch");
				$finish;
			end
			if(stage3_en) begin
				if(stage3_tmp !== stage3_tmp_ref) begin
					$display("data mismatch");
					$finish;
				end
			end
		end

	end

	else begin
		invalid_area_opt_setting();
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AREA_OPT=0

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

	logic		stage1_en;		//this SHOULD be logically equivalent to "en"
	always_comb stage1_en = en;	//but for reasons unclear, vivado 2024.1 and 2025.1 simulator (at least) require this
								//something somewhere is waiting on a delta cycle for something?

	always_ff @(posedge clk) begin

		//Pipeline timing flags
		stage2_en						<= stage1_en;

		//The actual multipliers
		for(integer j=0; j<32; j=j+1) begin
			if(stage1_en) begin
				stage2_do38[j]			<= (j > i);
				stage2_tmp.blocks[j]	<= a.blocks[j] * b.blocks[j];
			end
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AREA_OPT=1

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

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		stage3_en = 0;
		stage3_tmp = 0;
	end
	`endif

	logic		stage2_en	= 0;
	logic[31:0]	stage2_do38 = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Combined multipliers with resource sharing

	bignum32_t	mult_a;
	bignum_t	mult_b;

	logic		stage1_en;		//this SHOULD be logically equivalent to "en"
	always_comb stage1_en = en;	//but for reasons unclear, vivado 2024.1 simulator makes
								//en and stage2_en change the same cycle which they clearly will never do in real hardware
								//doing this gives what looks more like a correct result

	always_ff @(posedge clk) begin

		stage2_en	<= stage1_en;
		stage3_en	<= stage2_en;

		for(integer j=0; j<32; j++) begin

			//Input muxing
			if(stage1_en) begin
				mult_a.blocks[j]		= a.blocks[j];
				mult_b.blocks[j]		= b.blocks[j];
			end
			else begin
				mult_a.blocks[j]		= stage3_tmp.blocks[j];
				mult_b.blocks[j]		= 38;
			end

			if(stage1_en)
				stage2_do38[j]			<= (j > i);

			//The multipliers
			if(stage1_en || (stage2_en && stage2_do38[j]) )
				stage3_tmp.blocks[j]	<= mult_a.blocks[j] * mult_b.blocks[j];

		end

	end

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AREA_OPT=2 (BROKEN)

/**
	@brief X25519_MultPass_stage1 and X25519_MultPass_stage2 with AREA_OPT=2
 */
module X25519_MultPass_stage12_areaopt2(
	input wire			clk,
	input wire			en,
	input wire[4:0]		i,
	input wire bignum_t	a,
	input wire bignum_t	b,

	output wire			stage3_en,
	output bignum32_t	stage3_tmp
);

	logic		stage2_en	= 0;
	logic[31:0]	stage2_do38 = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input muxing

	bignum32_t	mult_a;
	bignum_t	mult_b;
	logic		stage1_en;		//this SHOULD be logically equivalent to "en"
	always_comb stage1_en = en;	//but for reasons unclear, vivado 2024.1 simulator makes
								//en and stage2_en change the same cycle which they clearly will never do in real hardware
								//doing this gives what looks more like a correct result

	always_comb begin
		for(integer j=0; j<32; j++) begin
			if(en) begin
				mult_a.blocks[j]	= a.blocks[j];
				mult_b.blocks[j]	= b.blocks[j];
			end
			else begin
				mult_a.blocks[j]	= stage3_tmp.blocks[j];
				mult_b.blocks[j]	= 38;
			end
		end
	end

	logic		stage2_en_ff = 0;
	always_ff @(posedge clk) begin

		stage2_en				<= stage1_en;
		stage2_en_ff			<= stage2_en;

		for(integer j=0; j<32; j++) begin
			if(stage1_en)
				stage2_do38[j]	<= (j > i);
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual multipliers

	wire[31:0]	mult_done;
	for(genvar g=0; g<32; g++) begin
		X25519_MultPass_32x16_areaopt2 mult(
			.clk(clk),
			.en(stage1_en || (stage2_en && stage2_do38[g]) ),
			.a(mult_a.blocks[g]),
			.b(mult_b.blocks[g]),
			.out_valid(mult_done[g]),
			.out(stage3_tmp.blocks[g])
		);
	end

	assign stage3_en = stage2_en_ff;

endmodule

module X25519_MultPass_32x16_areaopt2(
	input wire			clk,
	input wire			en,
	input wire[31:0]	a,
	input wire[15:0]	b,

	output logic		out_valid	`ifdef XILINX = 0 `endif,
	output logic[31:0]	out			`ifdef XILINX = 0 `endif
);

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		out_valid	= 0;
		out			= 0;
	end
	`endif

	//single cycle implementation for initial testing
	always_ff @(posedge clk) begin
		out_valid	<= en;
		if(en)
			out		<= a * b;
	end

endmodule
