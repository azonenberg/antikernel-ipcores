`default_nettype none
`timescale 1ns/1ps
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

`include "APBTypes.sv"

/**
	@brief Wrapper around YosysHQ's PicoRV32 core providing local RAM/ROM blocks
	and an APB requester for peripheral access
 */
module PicoRV32Subsystem #(
	parameter TCM_RAM_BASE	= 32'h2000_0000,
	parameter TCM_RAM_SIZE	= 32'h0000_2000,

	parameter TCM_ROM_BASE	= 32'h0800_0000,
	parameter TCM_ROM_SIZE	= 32'h0000_2000,
	parameter TCM_ROM_IMAGE = "/dev/null",

	parameter APB_BASE		= 32'h4000_0000,
	parameter APB_SIZE		= 32'h1000_0000
)(
	input wire		clk,
	input wire		rst_n,

	APB.requester	apb
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Convert memory sizes from bytes to words

	localparam TCM_RAM_WORDS = TCM_RAM_SIZE / 4;
	localparam TCM_ROM_WORDS = TCM_ROM_SIZE / 4;

	localparam TCM_RAM_MAX = TCM_RAM_BASE + TCM_RAM_SIZE - 1;
	localparam TCM_ROM_MAX = TCM_ROM_BASE + TCM_ROM_SIZE - 1;

	localparam APB_MAX = APB_BASE + APB_SIZE - 1;

	localparam TCM_RAM_BITS = $clog2(TCM_RAM_WORDS);
	localparam TCM_ROM_BITS = $clog2(TCM_ROM_WORDS);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CPU core

	wire		mem_valid;
	wire		mem_instr;
	logic		mem_ready	= 0;

	wire[31:0]	mem_addr;
	wire[31:0]	mem_wdata;
	wire[3:0]	mem_wstrb;
	logic[31:0]	mem_rdata	= 0;

	wire		trap;

	picorv32 #(
		.ENABLE_FAST_MUL(1),
		.ENABLE_DIV(1),
		.PROGADDR_RESET(TCM_ROM_BASE),					//reset vector is the start of ROM
		.PROGADDR_IRQ(32'h0000_0000),					//TODO: IRQ vector if/when we enable IRQs
		.STACKADDR(TCM_RAM_BASE + TCM_RAM_SIZE - 16)	//initial stack pointer at the end of RAM
	) mcu (

		.clk(clk),
		.resetn(rst_n),

		.trap(trap),

		//memory bus
		.mem_valid(mem_valid),
		.mem_instr(mem_instr),
		.mem_ready(mem_ready),
		.mem_addr(mem_addr),
		.mem_wdata(mem_wdata),
		.mem_wstrb(mem_wstrb),
		.mem_rdata(mem_rdata),

		//lookahead interface not used at the moment
		.mem_la_read(),
		.mem_la_write(),
		.mem_la_addr(),
		.mem_la_wdata(),
		.mem_la_wstrb(),

		//PCPI not used
		.pcpi_valid(),
		.pcpi_insn(),
		.pcpi_rs1(),
		.pcpi_rs2(),
		.pcpi_wr(1'b0),
		.pcpi_rd(32'h0),
		.pcpi_wait(1'b0),
		.pcpi_ready(1'b0),

		//IRQ interface not used
		.irq(32'h0),
		.eoi()

	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address decoding

	logic		mem_addr_is_ram;
	logic		mem_addr_is_rom;
	logic		mem_addr_is_apb;

	logic		mem_request;
	logic		mem_read;

	always_comb begin
		mem_addr_is_ram = (mem_addr >= TCM_RAM_BASE) && (mem_addr <= TCM_RAM_MAX);
		mem_addr_is_rom = (mem_addr >= TCM_ROM_BASE) && (mem_addr <= TCM_ROM_MAX);
		mem_addr_is_apb = (mem_addr >= APB_BASE) && (mem_addr <= APB_MAX);

		mem_request		= mem_valid && !mem_ready;
		mem_read		= mem_request && !mem_wstrb;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Instruction ROM

	(* RAM_STYLE = "block" *)
	logic[31:0] rom[TCM_ROM_WORDS-1:0];

	initial begin
		$readmemh(TCM_ROM_IMAGE, rom);
	end

	logic					rom_rd_en;
	logic[TCM_ROM_BITS-1:0]	rom_addr;

	//Control signal decoding
	always_comb begin
		rom_rd_en			= mem_read && mem_addr_is_rom;
		rom_addr			= mem_addr[2 +: TCM_ROM_BITS];
	end

	//Registered reads
	logic		rom_rd_valid = 0;
	logic[31:0]	rom_rd_data;
	always_ff @(posedge clk) begin
		rom_rd_valid		<= rom_rd_en;
		if(rom_rd_en)
			rom_rd_data		<= rom[rom_addr];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Data RAM

	(* RAM_STYLE = "block" *)
	logic[7:0] ram0[TCM_RAM_WORDS-1:0];

	(* RAM_STYLE = "block" *)
	logic[7:0] ram1[TCM_RAM_WORDS-1:0];

	(* RAM_STYLE = "block" *)
	logic[7:0] ram2[TCM_RAM_WORDS-1:0];

	(* RAM_STYLE = "block" *)
	logic[7:0] ram3[TCM_RAM_WORDS-1:0];

	initial begin
		for(integer i=0; i<2048; i++) begin
			ram0[i] = 0;
			ram1[i] = 0;
			ram2[i] = 0;
			ram3[i] = 0;
		end
	end

	logic[3:0]	ram_wr_en;
	logic		ram_rd_en;

	logic[TCM_RAM_BITS-1:0]	ram_addr;

	//Control signal decoding
	always_comb begin

		for(integer i=0; i<4; i=i+1)
			ram_wr_en[i]	= (mem_request && mem_wstrb[i] && mem_addr_is_ram);
		ram_rd_en			= mem_read && mem_addr_is_ram;

		ram_addr			= mem_addr[2 +: TCM_RAM_BITS];
	end

	//Registered writes
	always_ff @(posedge clk) begin
		if(ram_wr_en[0])
			ram0[ram_addr]	<= mem_wdata[7:0];
		if(ram_wr_en[1])
			ram1[ram_addr]	<= mem_wdata[15:8];
		if(ram_wr_en[2])
			ram2[ram_addr]	<= mem_wdata[23:16];
		if(ram_wr_en[3])
			ram3[ram_addr]	<= mem_wdata[31:24];
	end

	//Registered reads
	logic		ram_rd_valid = 0;
	logic[31:0]	ram_rd_data;
	always_ff @(posedge clk) begin
		ram_rd_valid		<= ram_rd_en;
		if(ram_rd_en)
			ram_rd_data		<= { ram3[ram_addr], ram2[ram_addr], ram1[ram_addr], ram0[ram_addr] };
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memory read/write logic

	assign apb.pclk			= clk;
	assign apb.preset_n		= rst_n;

	assign apb.pprot		= 3'h0;
	assign apb.pwakeup		= 0;
	assign apb.pauser		= 0;
	assign apb.pwuser		= 0;

	initial begin
		apb.penable = 0;
		apb.psel	= 0;
		apb.paddr	= 0;
		apb.pwrite	= 0;
		apb.pwdata	= 0;
		apb.pstrb 	= 0;
	end

	//Muxing of RAM read state
	always_ff @(posedge clk) begin
		mem_rdata	<= 0;
		mem_ready	<= 0;

		//Handle APB reply coming back
		if(apb.pslverr || apb.pready) begin
			mem_ready	<= 1;
			mem_rdata	<= apb.prdata;
		end

		//Acknowledge RAM writes immediately
		if(ram_wr_en)
			mem_ready	<= 1;

		//Mux RAM/ROM reads/writes on completion
		if(ram_rd_valid) begin
			mem_ready	<= 1;
			mem_rdata	<= ram_rd_data;
		end
		if(rom_rd_valid) begin
			mem_ready	<= 1;
			mem_rdata	<= rom_rd_data;
		end

		//any invalid memory access will never be acknowledged and hang the bus
		//TODO: add some kind of watchdog or fault handler for this?

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interfacing

	always_ff @(posedge clk) begin

		//clear flags when pready is asserted
		if(apb.pready || mem_ready) begin
			apb.penable	<= 0;
			apb.psel	<= 0;
			apb.pwrite	<= 0;
			apb.pstrb	<= 0;
		end

		//assert sel one cycle after enable
		else if(apb.penable && !apb.psel)
			apb.psel	<= 1;

		//Dispatch APB requests
		if(mem_request && mem_addr_is_apb) begin
			apb.penable	<= 1;
			apb.paddr	<= mem_addr;
			apb.pstrb	<= mem_wstrb;
			apb.pwdata	<= mem_wdata;

			if(mem_wstrb)
				apb.pwrite	<= 1;
			else
				apb.pwrite	<= 0;
		end

	end

endmodule
