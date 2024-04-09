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
	@brief A single bank of the register file

	Dual port, one R/W port (we cannot retire an instruction and fetch simultaneously) and three R/O ports
 */
module X25519_RegfileBank(

	input wire			clk,
	input wire			wr_en,
	input wire regid_t	wr_addr,
	input wire regval_t	wr_data,

	input wire regid_t	rd_addr0,
	input wire regid_t	rd_addr1,
	input wire regid_t	rd_addr2,
	input wire regid_t	rd_addr3,

	output regval_t		rd_data0,
	output regval_t		rd_data1,
	output regval_t		rd_data2,
	output regval_t		rd_data3
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write port address muxing

	regid_t p3_addr;
	always_comb begin
		if(wr_en)
			p3_addr = wr_addr;
		else
			p3_addr = rd_addr3;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual memory array

	(* RAM_STYLE = "distributed" *)
	regval_t		mem[REG_COUNT-1:0];

	//Clear all registers to zero on reset
	initial begin

		//Normal GPRs
		for(integer i=0; i<=REG_TEMP_10; i++)
			mem[i]			<= 0;

		//Special registers for constants (read only)
		mem[REG_ONE]		<= 264'h1;
		mem[REG_ZERO]		<= 264'h0;
		mem[REG_121665]		<= 264'd121665;
		mem[REG_D2]			<= 264'h2406d9dc56dffce7198e80f2eef3d13000e0149a8283b156ebd69b9426b2f159;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Writing

	always_ff @(posedge clk) begin
		if(wr_en)
			mem[wr_addr] <= wr_data;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reading

	assign rd_data0 = mem[rd_addr0];
	assign rd_data1 = mem[rd_addr1];
	assign rd_data2 = mem[rd_addr2];
	assign rd_data3 = mem[p3_addr];

endmodule

/**
	@brief Register file
 */
module X25519_Regfile(
	input wire			clk,

	//Write ports
	input wire			share_add_valid,
	input wire			share_sub_valid,
	input wire			share_mult_valid,
	input wire			share_select_valid,

	input wire			want_add_result,
	input wire			want_sub_result,
	input wire			want_mult_result,
	input wire			want_select_result,

	input wire			dh_en,
	input wire			dsa_load,
	input wire[1:0]		dsa_addr,

	input wire regid_t	add_rd_ff,
	input wire regid_t	sub_rd_ff,
	input wire regid_t	mult_rd_ff,
	input wire regid_t	select_p_rd_ff,
	input wire regid_t	select_q_rd_ff,

	input wire regval_t	share_add_out,
	input wire regval_t	share_sub_out,
	input wire regval_t	share_mult_out,
	input wire regval_t	share_select_p,
	input wire regval_t	share_select_q,
	input wire regval_t	work_in,

	//Read stuff
	input wire			dsa_rd,
	input wire			rd_en,
	input wire			share_freeze_en,
	input wire			ml_rd_en,
	input wire regid_t	addsub_a_regid,
	input wire regid_t	addsub_b_regid,
	input wire regid_t	mult_a_regid,
	input wire regid_t	mult_b_regid,
	input wire regid_t	select_r_regid,
	input wire regid_t	select_s_regid,

	output regval_t		share_addsub_a,
	output regval_t		share_addsub_b,
	output regval_t		share_mult_a,
	output regval_t		share_mult_b,
	output regval_t		share_freeze_a
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memory banks

	logic		p0_wr_en;
	logic		p1_wr_en;

	regid_t		p0_wr_addr;
	regid_t		p1_wr_addr;

	regval_t	p0_wr_data;
	regval_t	p1_wr_data;

	regid_t		p0_rd_addr;
	regid_t		p1_rd_addr;
	regid_t		p2_rd_addr;
	regid_t		p3_rd_addr;

	regval_t	p0_rd_data_bank0;
	regval_t	p1_rd_data_bank0;
	regval_t	p2_rd_data_bank0;
	regval_t	p3_rd_data_bank0;

	X25519_RegfileBank bank0(
		.clk(clk),

		.wr_en(p0_wr_en),
		.wr_addr(p0_wr_addr),
		.wr_data(p0_wr_data),

		.rd_addr0(p0_rd_addr),
		.rd_addr1(p1_rd_addr),
		.rd_addr2(p2_rd_addr),
		.rd_addr3(p3_rd_addr),

		.rd_data0(p0_rd_data_bank0),
		.rd_data1(p1_rd_data_bank0),
		.rd_data2(p2_rd_data_bank0),
		.rd_data3(p3_rd_data_bank0)
		);

	regval_t	p0_rd_data_bank1;
	regval_t	p1_rd_data_bank1;
	regval_t	p2_rd_data_bank1;
	regval_t	p3_rd_data_bank1;

	X25519_RegfileBank bank1(
		.clk(clk),

		.wr_en(p1_wr_en),
		.wr_addr(p1_wr_addr),
		.wr_data(p1_wr_data),

		.rd_addr0(p0_rd_addr),
		.rd_addr1(p1_rd_addr),
		.rd_addr2(p2_rd_addr),
		.rd_addr3(p3_rd_addr),

		.rd_data0(p0_rd_data_bank1),
		.rd_data1(p1_rd_data_bank1),
		.rd_data2(p2_rd_data_bank1),
		.rd_data3(p3_rd_data_bank1)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Live value table and read muxing

	regval_t	p0_rd_data;
	regval_t	p1_rd_data;
	regval_t	p2_rd_data;
	regval_t	p3_rd_data;

	logic[15:0]	lvt = 0;

	always_ff @(posedge clk) begin

		if(p0_wr_en)
			lvt[p0_wr_addr]	<= 0;
		if(p1_wr_en)
			lvt[p1_wr_addr]	<= 1;

	end

	always_comb begin

		if(lvt[p0_rd_addr])
			p0_rd_data	= p0_rd_data_bank1;
		else
			p0_rd_data	= p0_rd_data_bank0;

		if(lvt[p1_rd_addr])
			p1_rd_data	= p1_rd_data_bank1;
		else
			p1_rd_data	= p1_rd_data_bank0;

		if(lvt[p2_rd_addr])
			p2_rd_data	= p2_rd_data_bank1;
		else
			p2_rd_data	= p2_rd_data_bank0;

		if(lvt[p3_rd_addr])
			p3_rd_data	= p3_rd_data_bank1;
		else
			p3_rd_data	= p3_rd_data_bank0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write port address muxing and arbitration

	always_comb begin

		p0_wr_en	= 0;
		p1_wr_en	= 0;
		p0_wr_addr	= REG_ZERO;
		p1_wr_addr	= REG_ZERO;
		p0_wr_data	= 0;
		p1_wr_data	= 0;

		//Select completed, use both ports to retire
		if(share_select_valid && want_select_result) begin
			p0_wr_en	= 1;
			p1_wr_en	= 1;

			p0_wr_addr	= select_p_rd_ff;
			p1_wr_addr	= select_q_rd_ff;

			p0_wr_data	= share_select_p;
			p1_wr_data	= share_select_q;
		end

		//Multiply completed, use first port to retire
		//(even if an add/sub issued concurrently it's long done by now so no conflicts)
		if(share_mult_valid && want_mult_result) begin
			p0_wr_en	= 1;
			p0_wr_addr	= mult_rd_ff;
			p0_wr_data	= share_mult_out;
		end

		//Add/subtract completed, use first port for add and second for subtract
		if(share_add_valid && want_add_result) begin
			p0_wr_en	= 1;
			p0_wr_addr	= add_rd_ff;
			p0_wr_data	= share_add_out;
		end
		if(share_sub_valid && want_sub_result) begin
			p1_wr_en	= 1;
			p1_wr_addr	= sub_rd_ff;
			p1_wr_data	= share_sub_out;
		end

		//External input loading uses first port
		if(dsa_load) begin
			p0_wr_en	= 1;
			p0_wr_addr	= regid_t'({2'b0, dsa_addr});
			p0_wr_data	= work_in;
		end
		if(dh_en) begin
			p0_wr_en	= 1;
			p0_wr_addr	= REG_TEMP_10;
			p0_wr_data	= work_in;
		end

		//Do not allow writing to any of the constant registers
		if(p0_wr_addr > REG_TEMP_10)
			p0_wr_en = 0;
		if(p1_wr_addr > REG_TEMP_10)
			p1_wr_en = 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read port address muxing

	/*
		We have a total of 7 logical read ports: freeze, addsub_a/b, mult_a/b, select_r/s

		freeze is exclusive with the others so can use any port

		add/sub and mult can execute concurrently so we need at least 4 ports for them
		select can execute concurrently with add/sub but is never issued concurrently with multiply
	 */

	always_comb begin

		//default to reading nothing
		p0_rd_addr		= REG_ZERO;
		p1_rd_addr		= REG_ZERO;
		p2_rd_addr		= REG_ZERO;
		p3_rd_addr		= REG_ZERO;

		//Freeze: port 0
		if(share_freeze_en)
			p0_rd_addr	= REG_TEMP_0;

		//DSA output: port 0
		if(dsa_rd)
			p0_rd_addr	= regid_t'(REG_TEMP_4 + dsa_addr);

		//Add/sub: ports 0/1
		if(rd_en) begin
			p0_rd_addr	= addsub_a_regid;
			p1_rd_addr	= addsub_b_regid;
		end

		//Multiply or select: ports 2/3
		if(rd_en) begin

			//TODO: maybe we can shorten critical path here by passing multiply enable line?
			if(mult_a_regid != REG_ZERO) begin
				p2_rd_addr	= mult_a_regid;
				p3_rd_addr	= mult_b_regid;
			end

			else begin
				p2_rd_addr	= select_r_regid;
				p3_rd_addr	= select_s_regid;
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output registers

	always_ff @(posedge clk) begin

		if(share_freeze_en || dsa_rd)
			share_freeze_a	<= p0_rd_data;

		if(rd_en) begin
			share_addsub_a	<= p0_rd_data;
			share_addsub_b	<= p1_rd_data;
			share_mult_a	<= p2_rd_data;
			share_mult_b	<= p3_rd_data;
		end

	end

endmodule
