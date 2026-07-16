`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg                                                                          *
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
	@brief An internal logic analyzer with APB readout

	Debug ROM tag "ILA_"

	TODO scale to >32 ports
 */
module APB_ILA #(
	parameter DEPTH			= 2048,
	parameter CLK_PERIOD	= 10000,
	parameter DATA_BUF_ADDR	= 0,
	parameter ROM_ADDR		= 0,

	parameter PROBE0_WIDTH	= 0,
	parameter PROBE1_WIDTH	= 0,
	parameter PROBE2_WIDTH	= 0,
	parameter PROBE3_WIDTH	= 0,
	parameter PROBE4_WIDTH	= 0,
	parameter PROBE5_WIDTH	= 0,
	parameter PROBE6_WIDTH	= 0,
	parameter PROBE7_WIDTH	= 0,
	parameter PROBE8_WIDTH	= 0,
	parameter PROBE9_WIDTH	= 0,
	parameter PROBE10_WIDTH	= 0,
	parameter PROBE11_WIDTH	= 0,
	parameter PROBE12_WIDTH	= 0,
	parameter PROBE13_WIDTH	= 0,
	parameter PROBE14_WIDTH	= 0,
	parameter PROBE15_WIDTH	= 0,
	parameter PROBE16_WIDTH	= 0,
	parameter PROBE17_WIDTH	= 0,
	parameter PROBE18_WIDTH	= 0,
	parameter PROBE19_WIDTH	= 0,
	parameter PROBE20_WIDTH	= 0,
	parameter PROBE21_WIDTH	= 0,
	parameter PROBE22_WIDTH	= 0,
	parameter PROBE23_WIDTH	= 0,
	parameter PROBE24_WIDTH	= 0,
	parameter PROBE25_WIDTH	= 0,
	parameter PROBE26_WIDTH	= 0,
	parameter PROBE27_WIDTH	= 0,
	parameter PROBE28_WIDTH	= 0,
	parameter PROBE29_WIDTH	= 0,
	parameter PROBE30_WIDTH	= 0,
	parameter PROBE31_WIDTH	= 0,

	parameter PROBE0_NAME	= "",
	parameter PROBE1_NAME	= "",
	parameter PROBE2_NAME	= "",
	parameter PROBE3_NAME	= "",
	parameter PROBE4_NAME	= "",
	parameter PROBE5_NAME	= "",
	parameter PROBE6_NAME	= "",
	parameter PROBE7_NAME	= "",
	parameter PROBE8_NAME	= "",
	parameter PROBE9_NAME	= "",
	parameter PROBE10_NAME	= "",
	parameter PROBE11_NAME	= "",
	parameter PROBE12_NAME	= "",
	parameter PROBE13_NAME	= "",
	parameter PROBE14_NAME	= "",
	parameter PROBE15_NAME	= "",

	parameter PROBE16_NAME	= "",
	parameter PROBE17_NAME	= "",
	parameter PROBE18_NAME	= "",
	parameter PROBE19_NAME	= "",
	parameter PROBE20_NAME	= "",
	parameter PROBE21_NAME	= "",
	parameter PROBE22_NAME	= "",
	parameter PROBE23_NAME	= "",
	parameter PROBE24_NAME	= "",
	parameter PROBE25_NAME	= "",
	parameter PROBE26_NAME	= "",
	parameter PROBE27_NAME	= "",
	parameter PROBE28_NAME	= "",
	parameter PROBE29_NAME	= "",
	parameter PROBE30_NAME	= "",
	parameter PROBE31_NAME	= ""
)(
	//APB interface for the control plane registers (small, can be standard 1 kB)
	APB.completer 						apbControl,

	//APB interface for memory buffer readback (must be same clock domain as apbControl)
	APB.completer 						apbData,

	//APB interface for symbol table ROM (no restrictions on clock domain)
	APB.completer 						apbRom,

	//Probe inputs
	input wire							clk,
	input wire[PROBE0_WIDTH-1:0]		probe0,
	input wire[PROBE1_WIDTH-1:0]		probe1,
	input wire[PROBE2_WIDTH-1:0]		probe2,
	input wire[PROBE3_WIDTH-1:0]		probe3,
	input wire[PROBE4_WIDTH-1:0]		probe4,
	input wire[PROBE5_WIDTH-1:0]		probe5,
	input wire[PROBE6_WIDTH-1:0]		probe6,
	input wire[PROBE7_WIDTH-1:0]		probe7,
	input wire[PROBE8_WIDTH-1:0]		probe8,
	input wire[PROBE9_WIDTH-1:0]		probe9,
	input wire[PROBE10_WIDTH-1:0]		probe10,
	input wire[PROBE11_WIDTH-1:0]		probe11,
	input wire[PROBE12_WIDTH-1:0]		probe12,
	input wire[PROBE13_WIDTH-1:0]		probe13,
	input wire[PROBE14_WIDTH-1:0]		probe14,
	input wire[PROBE15_WIDTH-1:0]		probe15,
	input wire[PROBE16_WIDTH-1:0]		probe16,
	input wire[PROBE17_WIDTH-1:0]		probe17,
	input wire[PROBE18_WIDTH-1:0]		probe18,
	input wire[PROBE19_WIDTH-1:0]		probe19,
	input wire[PROBE20_WIDTH-1:0]		probe20,
	input wire[PROBE21_WIDTH-1:0]		probe21,
	input wire[PROBE22_WIDTH-1:0]		probe22,
	input wire[PROBE23_WIDTH-1:0]		probe23,
	input wire[PROBE24_WIDTH-1:0]		probe24,
	input wire[PROBE25_WIDTH-1:0]		probe25,
	input wire[PROBE26_WIDTH-1:0]		probe26,
	input wire[PROBE27_WIDTH-1:0]		probe27,
	input wire[PROBE28_WIDTH-1:0]		probe28,
	input wire[PROBE29_WIDTH-1:0]		probe29,
	input wire[PROBE30_WIDTH-1:0]		probe30,
	input wire[PROBE31_WIDTH-1:0]		probe31,

	//Trigger sync
	input wire							trig_in,
	output logic						trig_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Endianness-swap the channel names because systemverilog has derpy string ordering

	localparam PROBE0_NAME_BSWAP  = {<<8{PROBE0_NAME}};
	localparam PROBE1_NAME_BSWAP  = {<<8{PROBE1_NAME}};
	localparam PROBE2_NAME_BSWAP  = {<<8{PROBE2_NAME}};
	localparam PROBE3_NAME_BSWAP  = {<<8{PROBE3_NAME}};
	localparam PROBE4_NAME_BSWAP  = {<<8{PROBE4_NAME}};
	localparam PROBE5_NAME_BSWAP  = {<<8{PROBE5_NAME}};
	localparam PROBE6_NAME_BSWAP  = {<<8{PROBE6_NAME}};
	localparam PROBE7_NAME_BSWAP  = {<<8{PROBE7_NAME}};
	localparam PROBE8_NAME_BSWAP  = {<<8{PROBE8_NAME}};
	localparam PROBE9_NAME_BSWAP  = {<<8{PROBE9_NAME}};
	localparam PROBE10_NAME_BSWAP = {<<8{PROBE10_NAME}};
	localparam PROBE11_NAME_BSWAP = {<<8{PROBE11_NAME}};
	localparam PROBE12_NAME_BSWAP = {<<8{PROBE12_NAME}};
	localparam PROBE13_NAME_BSWAP = {<<8{PROBE13_NAME}};
	localparam PROBE14_NAME_BSWAP = {<<8{PROBE14_NAME}};
	localparam PROBE15_NAME_BSWAP = {<<8{PROBE15_NAME}};
	localparam PROBE16_NAME_BSWAP = {<<8{PROBE16_NAME}};
	localparam PROBE17_NAME_BSWAP = {<<8{PROBE17_NAME}};
	localparam PROBE18_NAME_BSWAP = {<<8{PROBE18_NAME}};
	localparam PROBE19_NAME_BSWAP = {<<8{PROBE19_NAME}};
	localparam PROBE20_NAME_BSWAP = {<<8{PROBE20_NAME}};
	localparam PROBE21_NAME_BSWAP = {<<8{PROBE21_NAME}};
	localparam PROBE22_NAME_BSWAP = {<<8{PROBE22_NAME}};
	localparam PROBE23_NAME_BSWAP = {<<8{PROBE23_NAME}};
	localparam PROBE24_NAME_BSWAP = {<<8{PROBE24_NAME}};
	localparam PROBE25_NAME_BSWAP = {<<8{PROBE25_NAME}};
	localparam PROBE26_NAME_BSWAP = {<<8{PROBE26_NAME}};
	localparam PROBE27_NAME_BSWAP = {<<8{PROBE27_NAME}};
	localparam PROBE28_NAME_BSWAP = {<<8{PROBE28_NAME}};
	localparam PROBE29_NAME_BSWAP = {<<8{PROBE29_NAME}};
	localparam PROBE30_NAME_BSWAP = {<<8{PROBE30_NAME}};
	localparam PROBE31_NAME_BSWAP = {<<8{PROBE31_NAME}};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Width of each channel's name, in bytes

	localparam[15:0] PROBE0_NAME_WIDTH  = $bits(PROBE0_NAME) / 8;
	localparam[15:0] PROBE1_NAME_WIDTH  = $bits(PROBE1_NAME) / 8;
	localparam[15:0] PROBE2_NAME_WIDTH  = $bits(PROBE2_NAME) / 8;
	localparam[15:0] PROBE3_NAME_WIDTH  = $bits(PROBE3_NAME) / 8;
	localparam[15:0] PROBE4_NAME_WIDTH  = $bits(PROBE4_NAME) / 8;
	localparam[15:0] PROBE5_NAME_WIDTH  = $bits(PROBE5_NAME) / 8;
	localparam[15:0] PROBE6_NAME_WIDTH  = $bits(PROBE6_NAME) / 8;
	localparam[15:0] PROBE7_NAME_WIDTH  = $bits(PROBE7_NAME) / 8;
	localparam[15:0] PROBE8_NAME_WIDTH  = $bits(PROBE8_NAME) / 8;
	localparam[15:0] PROBE9_NAME_WIDTH  = $bits(PROBE9_NAME) / 8;
	localparam[15:0] PROBE10_NAME_WIDTH = $bits(PROBE10_NAME) / 8;
	localparam[15:0] PROBE11_NAME_WIDTH = $bits(PROBE11_NAME) / 8;
	localparam[15:0] PROBE12_NAME_WIDTH = $bits(PROBE12_NAME) / 8;
	localparam[15:0] PROBE13_NAME_WIDTH = $bits(PROBE13_NAME) / 8;
	localparam[15:0] PROBE14_NAME_WIDTH = $bits(PROBE14_NAME) / 8;
	localparam[15:0] PROBE15_NAME_WIDTH = $bits(PROBE15_NAME) / 8;
	localparam[15:0] PROBE16_NAME_WIDTH = $bits(PROBE16_NAME) / 8;
	localparam[15:0] PROBE17_NAME_WIDTH = $bits(PROBE17_NAME) / 8;
	localparam[15:0] PROBE18_NAME_WIDTH = $bits(PROBE18_NAME) / 8;
	localparam[15:0] PROBE19_NAME_WIDTH = $bits(PROBE19_NAME) / 8;
	localparam[15:0] PROBE20_NAME_WIDTH = $bits(PROBE20_NAME) / 8;
	localparam[15:0] PROBE21_NAME_WIDTH = $bits(PROBE21_NAME) / 8;
	localparam[15:0] PROBE22_NAME_WIDTH = $bits(PROBE22_NAME) / 8;
	localparam[15:0] PROBE23_NAME_WIDTH = $bits(PROBE23_NAME) / 8;
	localparam[15:0] PROBE24_NAME_WIDTH = $bits(PROBE24_NAME) / 8;
	localparam[15:0] PROBE25_NAME_WIDTH = $bits(PROBE25_NAME) / 8;
	localparam[15:0] PROBE26_NAME_WIDTH = $bits(PROBE26_NAME) / 8;
	localparam[15:0] PROBE27_NAME_WIDTH = $bits(PROBE27_NAME) / 8;
	localparam[15:0] PROBE28_NAME_WIDTH = $bits(PROBE28_NAME) / 8;
	localparam[15:0] PROBE29_NAME_WIDTH = $bits(PROBE29_NAME) / 8;
	localparam[15:0] PROBE30_NAME_WIDTH = $bits(PROBE30_NAME) / 8;
	localparam[15:0] PROBE31_NAME_WIDTH = $bits(PROBE31_NAME) / 8;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Raw ID ROM: everything but the initial "number of valid words" header

	/*
		Format in linear addres range is:
			uint16_t probe_width_bits
			uint16_t name_length_bytes
			char name[name_length_bytes]

		note that we need to reverse this in the HDL because MSB first ordering
	 */
	localparam RAW_ID_ROM =
	{
		 PROBE31_NAME_BSWAP, PROBE31_NAME_WIDTH[15:0], PROBE31_WIDTH[15:0],
		 PROBE30_NAME_BSWAP, PROBE30_NAME_WIDTH[15:0], PROBE30_WIDTH[15:0],

		 PROBE29_NAME_BSWAP, PROBE29_NAME_WIDTH[15:0], PROBE29_WIDTH[15:0],
		 PROBE28_NAME_BSWAP, PROBE28_NAME_WIDTH[15:0], PROBE28_WIDTH[15:0],
		 PROBE27_NAME_BSWAP, PROBE27_NAME_WIDTH[15:0], PROBE27_WIDTH[15:0],
		 PROBE26_NAME_BSWAP, PROBE26_NAME_WIDTH[15:0], PROBE26_WIDTH[15:0],
		 PROBE25_NAME_BSWAP, PROBE25_NAME_WIDTH[15:0], PROBE25_WIDTH[15:0],
		 PROBE24_NAME_BSWAP, PROBE24_NAME_WIDTH[15:0], PROBE24_WIDTH[15:0],
		 PROBE23_NAME_BSWAP, PROBE23_NAME_WIDTH[15:0], PROBE23_WIDTH[15:0],
		 PROBE22_NAME_BSWAP, PROBE22_NAME_WIDTH[15:0], PROBE22_WIDTH[15:0],
		 PROBE21_NAME_BSWAP, PROBE21_NAME_WIDTH[15:0], PROBE21_WIDTH[15:0],
		 PROBE20_NAME_BSWAP, PROBE20_NAME_WIDTH[15:0], PROBE20_WIDTH[15:0],

		 PROBE19_NAME_BSWAP, PROBE19_NAME_WIDTH[15:0], PROBE19_WIDTH[15:0],
		 PROBE18_NAME_BSWAP, PROBE18_NAME_WIDTH[15:0], PROBE18_WIDTH[15:0],
		 PROBE17_NAME_BSWAP, PROBE17_NAME_WIDTH[15:0], PROBE17_WIDTH[15:0],
		 PROBE16_NAME_BSWAP, PROBE16_NAME_WIDTH[15:0], PROBE16_WIDTH[15:0],
		 PROBE15_NAME_BSWAP, PROBE15_NAME_WIDTH[15:0], PROBE15_WIDTH[15:0],
		 PROBE14_NAME_BSWAP, PROBE14_NAME_WIDTH[15:0], PROBE14_WIDTH[15:0],
		 PROBE13_NAME_BSWAP, PROBE13_NAME_WIDTH[15:0], PROBE13_WIDTH[15:0],
		 PROBE12_NAME_BSWAP, PROBE12_NAME_WIDTH[15:0], PROBE12_WIDTH[15:0],
		 PROBE11_NAME_BSWAP, PROBE11_NAME_WIDTH[15:0], PROBE11_WIDTH[15:0],
		 PROBE10_NAME_BSWAP, PROBE10_NAME_WIDTH[15:0], PROBE10_WIDTH[15:0],

		 PROBE9_NAME_BSWAP,  PROBE9_NAME_WIDTH[15:0],  PROBE9_WIDTH[15:0],
		 PROBE8_NAME_BSWAP,  PROBE8_NAME_WIDTH[15:0],  PROBE8_WIDTH[15:0],
		 PROBE7_NAME_BSWAP,  PROBE7_NAME_WIDTH[15:0],  PROBE7_WIDTH[15:0],
		 PROBE6_NAME_BSWAP,  PROBE6_NAME_WIDTH[15:0],  PROBE6_WIDTH[15:0],
		 PROBE5_NAME_BSWAP,  PROBE5_NAME_WIDTH[15:0],  PROBE5_WIDTH[15:0],
		 PROBE4_NAME_BSWAP,  PROBE4_NAME_WIDTH[15:0],  PROBE4_WIDTH[15:0],
		 PROBE3_NAME_BSWAP,  PROBE3_NAME_WIDTH[15:0],  PROBE3_WIDTH[15:0],
		 PROBE2_NAME_BSWAP,  PROBE2_NAME_WIDTH[15:0],  PROBE2_WIDTH[15:0],
		 PROBE1_NAME_BSWAP,  PROBE1_NAME_WIDTH[15:0],  PROBE1_WIDTH[15:0],
		 PROBE0_NAME_BSWAP,  PROBE0_NAME_WIDTH[15:0],  PROBE0_WIDTH[15:0]
	};

	localparam RAW_ID_ROM_BITS = $bits(RAW_ID_ROM);
	localparam RAW_ID_ROM_WORDS = RAW_ID_ROM_BITS[4:0] ? (RAW_ID_ROM_BITS[31:5] + 1) : RAW_ID_ROM_BITS[31:5];

	localparam ID_ROM_PADDED =
	{
		32'h0,			//Zero padding to ensure any trailing bytes are 0x00
		RAW_ID_ROM		//The actual ROM content
	};

	logic[31:0] rom[RAW_ID_ROM_WORDS + 1 : 0];

	initial begin
		rom[0]			<= RAW_ID_ROM_WORDS;			//length of rest of ROM

		for(integer i=0; i<RAW_ID_ROM_WORDS; i++)
			rom[i+1]	<= ID_ROM_PADDED[32*i +: 32];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Registered reads to allow for ROM to be implemented as a block RAM if it's big enough

	always_ff @(posedge apbRom.pclk) begin

		//Clear flags
		apbRom.pready	<= 0;
		apbRom.prdata	<= 0;
		apbRom.pslverr	<= 0;

		if(apbRom.psel && apbRom.penable) begin

			apbRom.pready		<= 1;

			//Reject all writes
			if(apbRom.pwrite)
				apbRom.pslverr	<= 1;

			//Reads
			else
				apbRom.prdata	<= rom[ apbRom.paddr[apbRom.ADDR_WIDTH-1 : 2] ];

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Probe bit position indexes within capture buffer

	localparam PROBE0_BASE	= 0;
	localparam PROBE1_BASE	= PROBE0_BASE  + PROBE0_WIDTH;
	localparam PROBE2_BASE	= PROBE1_BASE  + PROBE1_WIDTH;
	localparam PROBE3_BASE	= PROBE2_BASE  + PROBE2_WIDTH;
	localparam PROBE4_BASE	= PROBE3_BASE  + PROBE3_WIDTH;
	localparam PROBE5_BASE	= PROBE4_BASE  + PROBE4_WIDTH;
	localparam PROBE6_BASE	= PROBE5_BASE  + PROBE5_WIDTH;
	localparam PROBE7_BASE	= PROBE6_BASE  + PROBE6_WIDTH;
	localparam PROBE8_BASE	= PROBE7_BASE  + PROBE7_WIDTH;
	localparam PROBE9_BASE	= PROBE8_BASE  + PROBE8_WIDTH;
	localparam PROBE10_BASE	= PROBE9_BASE  + PROBE9_WIDTH;
	localparam PROBE11_BASE	= PROBE10_BASE + PROBE10_WIDTH;
	localparam PROBE12_BASE	= PROBE11_BASE + PROBE11_WIDTH;
	localparam PROBE13_BASE	= PROBE12_BASE + PROBE12_WIDTH;
	localparam PROBE14_BASE	= PROBE13_BASE + PROBE13_WIDTH;
	localparam PROBE15_BASE	= PROBE14_BASE + PROBE14_WIDTH;
	localparam PROBE16_BASE	= PROBE15_BASE + PROBE15_WIDTH;
	localparam PROBE17_BASE	= PROBE16_BASE + PROBE16_WIDTH;
	localparam PROBE18_BASE	= PROBE17_BASE + PROBE17_WIDTH;
	localparam PROBE19_BASE	= PROBE18_BASE + PROBE18_WIDTH;
	localparam PROBE20_BASE	= PROBE19_BASE + PROBE19_WIDTH;
	localparam PROBE21_BASE	= PROBE20_BASE + PROBE20_WIDTH;
	localparam PROBE22_BASE	= PROBE21_BASE + PROBE21_WIDTH;
	localparam PROBE23_BASE	= PROBE22_BASE + PROBE22_WIDTH;
	localparam PROBE24_BASE	= PROBE23_BASE + PROBE23_WIDTH;
	localparam PROBE25_BASE	= PROBE24_BASE + PROBE24_WIDTH;
	localparam PROBE26_BASE	= PROBE25_BASE + PROBE25_WIDTH;
	localparam PROBE27_BASE	= PROBE26_BASE + PROBE26_WIDTH;
	localparam PROBE28_BASE	= PROBE27_BASE + PROBE27_WIDTH;
	localparam PROBE29_BASE	= PROBE28_BASE + PROBE28_WIDTH;
	localparam PROBE30_BASE	= PROBE29_BASE + PROBE29_WIDTH;
	localparam PROBE31_BASE	= PROBE30_BASE + PROBE30_WIDTH;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Concatenate the inputs into a single bus

	localparam MEM_WIDTH		= PROBE31_BASE + PROBE31_WIDTH;

	wire[MEM_WIDTH-1:0] probe_bus;
	assign probe_bus =
	{
		probe31,
		probe30,
		probe29,
		probe28,
		probe27,
		probe26,
		probe25,
		probe24,
		probe23,
		probe22,
		probe21,
		probe20,
		probe19,
		probe18,
		probe17,
		probe16,
		probe15,
		probe14,
		probe13,
		probe12,
		probe11,
		probe10,
		probe9,
		probe8,
		probe7,
		probe6,
		probe5,
		probe4,
		probe3,
		probe2,
		probe1,
		probe0
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage on the input (for now, a single hard coded stage)

	logic[MEM_WIDTH-1:0] probe_pipe = 0;

	always_ff @(posedge clk) begin
		probe_pipe	<= probe_bus;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sanity check bus config

	if(apbData.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();
	if(apbControl.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();
	if(apbRom.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apbControl.pruser = 0;
	assign apbControl.pbuser = 0;

	assign apbData.pruser = 0;
	assign apbData.pbuser = 0;

	assign apbRom.pruser = 0;
	assign apbRom.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Capture buffer

	localparam ROW_BITS = $clog2(DEPTH);

	//The buffer itself
	logic[MEM_WIDTH-1:0] capture[DEPTH-1:0];

	//Write any time we have valid data and are in the capturing state
	logic				wr_en	= 0;
	logic[ROW_BITS-1:0]	wr_addr = 0;

	//Write logic
	always_ff @(posedge clk) begin
		if(wr_en)
			capture[wr_addr]	<= probe_pipe;
	end

	logic					rd_en;
	logic[ROW_BITS-1:0]		rd_addr;
	logic[MEM_WIDTH-1:0]	rd_data;

	always_ff @(posedge apbData.pclk) begin
		if(rd_en)
			rd_data		<= capture[rd_addr];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write side trigger combining

	logic	trigger;

	//for now just use ext trigger
	always_comb begin
		trigger = trig_in;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write side control state machine
	// TODO: this is the same as SERDES ILA, refactor into shared module??

	wire[ROW_BITS-1:0] trigger_offset_words_sync;

	logic[ROW_BITS-1:0] trigger_pos = 0;
	logic[ROW_BITS-1:0] trigger_endpos = 0;

	typedef enum logic[1:0]
	{
		ILA_STATE_PRE_TRIG	= 0,
		ILA_STATE_ARMED		= 1,
		ILA_STATE_POST_TRIG	= 2,
		ILA_STATE_DONE		= 3
	} ila_state_t;

	ila_state_t ila_state = ILA_STATE_DONE;

	logic[ROW_BITS-1:0]	wr_addr_next;
	always_comb begin
		wr_addr_next = wr_addr + 1'h1;
	end

	//Status flags to read side
	logic	ila_done_pulse	= 0;

	//Status flags from read side
	wire	ila_arm_pulse_sync;

	always_ff @(posedge clk) begin

		ila_done_pulse	<= 0;
		trig_out		<= 0;

		if(wr_en && wr_en)
			wr_addr	<= wr_addr_next;

		case(ila_state)

			//Pre trigger: wr_en initial samples into the buffer, not yet looking for trigger events
			ILA_STATE_PRE_TRIG: begin
				wr_en	<= 1;

				if(wr_addr >= trigger_offset_words_sync)
					ila_state	<= ILA_STATE_ARMED;
			end //ILA_STATE_PRE_TRIG

			//Armed: waiting for a trigger event
			ILA_STATE_ARMED: begin

				//Wait for the trigger event
				if(trigger) begin
					trig_out		<= 1;
					trigger_pos		<= wr_addr;
					trigger_endpos	<= wr_addr + DEPTH - trigger_offset_words_sync;
					ila_state		<= ILA_STATE_POST_TRIG;
				end

			end //ILA_STATE_ARMED

			ILA_STATE_POST_TRIG: begin

				if(wr_en && (wr_addr_next == trigger_endpos) ) begin
					wr_en			<= 0;
					ila_done_pulse	<= 1;
					ila_state		<= ILA_STATE_DONE;
				end

			end //ILA_STATE_POST_TRIG

			ILA_STATE_DONE: begin
				if(ila_arm_pulse_sync) begin
					ila_state	<= ILA_STATE_PRE_TRIG;
					wr_addr		<= 0;
				end
			end //ILA_STATE_DONE

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC from write to read side

	wire[ROW_BITS-1:0]	trigger_pos_sync;
	wire				ila_done_pulse_sync;

	RegisterSynchronizer #(
		.WIDTH(ROW_BITS)
	) sync_ila_trigger_pos (
		.clk_a(clk),
		.en_a(ila_done_pulse),
		.ack_a(),
		.reg_a(trigger_pos),

		.clk_b(apbData.pclk),
		.updated_b(ila_done_pulse_sync),
		.reset_b(1'b0),
		.reg_b(trigger_pos_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC from read to write side

	logic	ila_arm_pulse	= 0;

	PulseSynchronizer sync_arm(
		.clk_a(apbControl.pclk),
		.pulse_a(ila_arm_pulse),
		.clk_b(clk),
		.pulse_b(ila_arm_pulse_sync)
	);

	logic				trigger_offset_update	= 0;
	logic[ROW_BITS-1:0] trigger_offset_words = DEPTH / 2;

	RegisterSynchronizer #(
		.WIDTH(ROW_BITS),
		.INIT(DEPTH / 2)
	) sync_ila_trigger_offset (
		.clk_a(apbData.pclk),
		.en_a(trigger_offset_update),
		.ack_a(),
		.reg_a(trigger_offset_words),

		.clk_b(clk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(trigger_offset_words_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[4:0]
	{
		REG_TRIGGER		= 'h00,			//[0] trigger arm (RAZ, write 1 to arm trigger)
										//[1] data ready flag (RW, write 0 after readout)
										//[2] force trigger bit (RAZ)
		REG_TRIGGER_IDX	= 'h04,			//Sample address of the trigger within the buffer, in words (RO)
		REG_DATA_BASE	= 'h08,			//Base address of data bus
		REG_DEPTH		= 'h0c,			//Total memory depth
		REG_RATE		= 'h10,			//Picoseconds per sample
		REG_TRIG_POS	= 'h14,			//Word index for the trigger within the logical space
		REG_ROM_BASE	= 'h18			//Base address of symbol table
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control plane register interface: Combinatorial read, synchronous write

	logic	ila_ready_sticky = 0;

	always_comb begin

		apbControl.pready	= apbControl.psel && apbControl.penable;
		apbControl.prdata	= 0;
		apbControl.pslverr	= 0;

		if(apbControl.pready) begin

			//read
			if(!apbControl.pwrite) begin
				case( apbControl.paddr)
					REG_TRIGGER:		apbControl.prdata	= { 30'h0, ila_ready_sticky, 1'b0 } ;
					REG_TRIGGER_IDX:	apbControl.prdata	= trigger_pos_sync;
					REG_DATA_BASE:		apbControl.prdata	= DATA_BUF_ADDR;
					REG_DEPTH:			apbControl.prdata	= DEPTH;
					REG_RATE:			apbControl.prdata	= CLK_PERIOD;
					REG_TRIG_POS:		apbControl.prdata	= trigger_offset_words;
					REG_ROM_BASE:		apbControl.prdata	= ROM_ADDR;
					default:			apbControl.pslverr	= 1;
				endcase
			end

			//write
			else begin

				case(apbControl.paddr)
					REG_TRIGGER: 	begin end
					REG_TRIG_POS:	begin end
					default:		apbControl.pslverr	= 1;
				endcase

			end

		end

	end

	always_ff @(posedge apbControl.pclk or negedge apbControl.preset_n) begin

		if(!apbControl.preset_n) begin
			ila_ready_sticky		<= 0;
			ila_arm_pulse			<= 0;
			trigger_offset_update	<= 0;
			trigger_offset_words 	<= DEPTH / 2;
		end

		else begin

			if(ila_done_pulse_sync)
				ila_ready_sticky 	<= 1;

			ila_arm_pulse			<= 0;
			trigger_offset_update	<= 0;

			if(apbControl.pready) begin

				if(apbControl.pwrite) begin
					case(apbControl.paddr)

						REG_TRIGGER: begin
							if(!apbControl.pwdata[2])
								ila_ready_sticky	<= 0;
							if(apbControl.pwdata[0])
								ila_arm_pulse		<= 1;
						end

						REG_TRIG_POS: begin
							trigger_offset_words	<= apbControl.pwdata;
							trigger_offset_update	<= 1;
						end

						default: begin
						end
					endcase
				end

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Data plane readout interface: sequential on both

	//Memory word size is variable! As a result we can't hardcode how many 32-bit APB words fit in a memory word
	//Round up memory width to the next multiple of 32
	localparam MEM_WIDTH_ROUND_UP = MEM_WIDTH[4:0] ? (MEM_WIDTH | 5'h1f) + 1 : MEM_WIDTH;

	//Divide by 32 bits to get the number of APB words in a single memory row
	localparam APB_WORDS_PER_SAMPLE = MEM_WIDTH_ROUND_UP / 32;

	//Number of APB address bits to select word index within a row
	localparam WORD_BITS = (APB_WORDS_PER_SAMPLE == 1) ? 1 : $clog2(APB_WORDS_PER_SAMPLE);

	//Number of address bits to select row index is ROW_BITS
	localparam TOTAL_ADDR_BITS = ROW_BITS + WORD_BITS;
	logic[WORD_BITS-1:0]	readout_word_idx;

	logic apbData_pready_next;

	always_comb begin

		//Ready one cycle after requesting a transaction
		apbData_pready_next = apbData.psel && apbData.penable;

		//Read any time an APB read comes in
		rd_en				= apbData_pready_next && !apbData.pwrite;

		//Read the memory address word
		rd_addr				= apbData.paddr[WORD_BITS + 2 +: ROW_BITS];

		//Get column index
		readout_word_idx	= apbData.paddr[2 +: WORD_BITS];

		//Reject all writes
		apbData.pslverr		= apbData.pready && apbData.pwrite;

		//Mux output
		apbData.prdata		= rd_data[32*readout_word_idx +: 32];

	end

	always_ff @(posedge apbData.pclk or negedge apbData.preset_n) begin

		if(!apbData.preset_n) begin
			apbData.pready	<= 0;
		end

		else begin
			apbData.pready	<= apbData_pready_next;
		end

	end

endmodule
