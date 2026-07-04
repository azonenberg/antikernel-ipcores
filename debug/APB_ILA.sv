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

	TODO scale to >16 ports

	Due to the large number of ports this block requires more than the standard 1 kB of address space for
	the register block.

	Each probe descriptor is 32 bytes, if we want to expand up to say 128 channels without a memory map change that's
	4 kB of descriptors.

	Register map (bank sel 13:12)
		0x0000				Base control registers
		0x1000				Debug ROM
		0x2000				Trigger config (not yet implemented)
 */
module APB_ILA #(
	parameter DEPTH			= 2048,
	parameter CLK_PERIOD	= 10000,
	parameter DATA_BUF_ADDR	= 0,

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

	parameter[247:0]	PROBE0_NAME		= 248'h0,
	parameter[247:0]	PROBE1_NAME		= 248'h0,
	parameter[247:0]	PROBE2_NAME		= 248'h0,
	parameter[247:0]	PROBE3_NAME		= 248'h0,
	parameter[247:0]	PROBE4_NAME		= 248'h0,
	parameter[247:0]	PROBE5_NAME		= 248'h0,
	parameter[247:0]	PROBE6_NAME		= 248'h0,
	parameter[247:0]	PROBE7_NAME		= 248'h0,
	parameter[247:0]	PROBE8_NAME		= 248'h0,
	parameter[247:0]	PROBE9_NAME		= 248'h0,
	parameter[247:0]	PROBE10_NAME	= 248'h0,
	parameter[247:0]	PROBE11_NAME	= 248'h0,
	parameter[247:0]	PROBE12_NAME	= 248'h0,
	parameter[247:0]	PROBE13_NAME	= 248'h0,
	parameter[247:0]	PROBE14_NAME	= 248'h0,
	parameter[247:0]	PROBE15_NAME	= 248'h0
)(
	//APB interface for the control plane registers
	APB.completer 						apbControl,

	//APB interface for memory buffer readback (must be same clock domain as apbControl)
	APB.completer 						apbData,

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

	//Trigger sync
	input wire							trig_in,
	output logic						trig_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IDs: name concatenated with width

	localparam PROBE0_ID  = { PROBE0_WIDTH[7:0],  PROBE0_NAME[247:0] };
	localparam PROBE1_ID  = { PROBE1_WIDTH[7:0],  PROBE1_NAME[247:0] };
	localparam PROBE2_ID  = { PROBE2_WIDTH[7:0],  PROBE2_NAME[247:0] };
	localparam PROBE3_ID  = { PROBE3_WIDTH[7:0],  PROBE3_NAME[247:0] };
	localparam PROBE4_ID  = { PROBE4_WIDTH[7:0],  PROBE4_NAME[247:0] };
	localparam PROBE5_ID  = { PROBE5_WIDTH[7:0],  PROBE5_NAME[247:0] };
	localparam PROBE6_ID  = { PROBE6_WIDTH[7:0],  PROBE6_NAME[247:0] };
	localparam PROBE7_ID  = { PROBE7_WIDTH[7:0],  PROBE7_NAME[247:0] };
	localparam PROBE8_ID  = { PROBE8_WIDTH[7:0],  PROBE8_NAME[247:0] };
	localparam PROBE9_ID  = { PROBE9_WIDTH[7:0],  PROBE9_NAME[247:0] };
	localparam PROBE10_ID = { PROBE10_WIDTH[7:0], PROBE10_NAME[247:0] };
	localparam PROBE11_ID = { PROBE11_WIDTH[7:0], PROBE11_NAME[247:0] };
	localparam PROBE12_ID = { PROBE12_WIDTH[7:0], PROBE12_NAME[247:0] };
	localparam PROBE13_ID = { PROBE13_WIDTH[7:0], PROBE13_NAME[247:0] };
	localparam PROBE14_ID = { PROBE14_WIDTH[7:0], PROBE14_NAME[247:0] };
	localparam PROBE15_ID = { PROBE15_WIDTH[7:0], PROBE15_NAME[247:0] };

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Probe bit position indexes

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Concatenate the inputs into a single bus

	localparam MEM_WIDTH		= PROBE15_BASE + PROBE15_WIDTH;

	wire[MEM_WIDTH-1:0] probe_bus;
	assign probe_bus =
	{
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

	if(apbControl.ADDR_WIDTH < 14)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apbControl.pruser = 0;
	assign apbControl.pbuser = 0;

	assign apbData.pruser = 0;
	assign apbData.pbuser = 0;

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
				//For now: trigger immediately
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
		REG_DATA_BASE	= 'h08,			//base address of data bus
		REG_DEPTH		= 'h0c,			//Total memory depth
		REG_RATE		= 'h10,			//Picoseconds per sample
		REG_TRIG_POS	= 'h14			//Word index for the trigger within the logical space
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control plane register interface: Combinatorial read, synchronous write

	logic	ila_ready_sticky = 0;

	//Divide the register space into 4 kB blocks
	logic[1:0]	ctl_block;
	logic[11:0]	ctl_regid;
	logic[6:0]	ctl_port;
	logic[2:0]	ctl_word_idx;

	typedef enum logic[1:0]
	{
		BLOCK_CSR			= 'h00,
		BLOCK_ROM			= 'h01,
		BLOCK_TRIGGER_PAT	= 'h02,
		BLOCK_TRIGGER_MASK	= 'h03
	} block_t;

	always_comb begin

		apbControl.pready	= apbControl.psel && apbControl.penable;
		apbControl.prdata	= 0;
		apbControl.pslverr	= 0;

		//Crack address
		ctl_block			= apbControl.paddr[13:12];
		ctl_regid			= apbControl.paddr[11:0];
		ctl_port			= apbControl.paddr[11:5];
		ctl_word_idx		= apbControl.paddr[4:2];

		if(apbControl.pready) begin

			//read
			if(!apbControl.pwrite) begin
				case(ctl_block)

					//Small control registers
					BLOCK_CSR: begin
						case(ctl_regid)
							REG_TRIGGER:		apbControl.prdata	= { 30'h0, ila_ready_sticky, 1'b0 } ;
							REG_TRIGGER_IDX:	apbControl.prdata	= trigger_pos_sync;
							REG_DATA_BASE:		apbControl.prdata	= DATA_BUF_ADDR;
							REG_DEPTH:			apbControl.prdata	= DEPTH;
							REG_RATE:			apbControl.prdata	= CLK_PERIOD;
							REG_TRIG_POS:		apbControl.prdata	= trigger_offset_words;
							default:			apbControl.pslverr	= 1;
						endcase
					end	//BLOCK_CSR

					//Signal ID ROM
					BLOCK_ROM: begin
						case(ctl_port)
							0:  apbControl.prdata = PROBE0_ID[ctl_word_idx*32 +: 32];
							1:  apbControl.prdata = PROBE1_ID[ctl_word_idx*32 +: 32];
							2:  apbControl.prdata = PROBE2_ID[ctl_word_idx*32 +: 32];
							3:  apbControl.prdata = PROBE3_ID[ctl_word_idx*32 +: 32];
							4:  apbControl.prdata = PROBE4_ID[ctl_word_idx*32 +: 32];
							5:  apbControl.prdata = PROBE5_ID[ctl_word_idx*32 +: 32];
							6:  apbControl.prdata = PROBE6_ID[ctl_word_idx*32 +: 32];
							7:  apbControl.prdata = PROBE7_ID[ctl_word_idx*32 +: 32];
							8:  apbControl.prdata = PROBE8_ID[ctl_word_idx*32 +: 32];
							9:  apbControl.prdata = PROBE9_ID[ctl_word_idx*32 +: 32];
							10: apbControl.prdata = PROBE10_ID[ctl_word_idx*32 +: 32];
							11: apbControl.prdata = PROBE11_ID[ctl_word_idx*32 +: 32];
							12: apbControl.prdata = PROBE12_ID[ctl_word_idx*32 +: 32];
							13: apbControl.prdata = PROBE13_ID[ctl_word_idx*32 +: 32];
							14: apbControl.prdata = PROBE14_ID[ctl_word_idx*32 +: 32];
							15: apbControl.prdata = PROBE15_ID[ctl_word_idx*32 +: 32];
						endcase
					end //BLOCK_ROM

					//invalid / unallocated
					default: begin
						apbControl.pslverr	= 1;
					end

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
	localparam WORD_BITS = $clog2(APB_WORDS_PER_SAMPLE);

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
