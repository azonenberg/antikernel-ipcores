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
	@brief An internal logic analyzer that captures 8B/10B code words and writes to block RAM then reads back over APB

	External triggering required, no pattern matching included for now
	Both APB buses are assumed to be in the same clock domain

	Debug ROM tag "8B10"
 */
module APB_SerdesILA_8b10b #(
	parameter DEPTH			= 2048,
	parameter WIDTH_SYMBOLS	= 2,
	parameter DATA_BUF_ADDR	= 0,
	parameter PS_PER_SYMBOL	= 8000
) (
	//APB interface for the control plane registers
	APB.completer 						apbControl,

	//APB interface for memory buffer readback
	APB.completer 						apbData,

	//8b10b data interface (clk domain)
	input wire							clk,
	input wire[WIDTH_SYMBOLS*8 - 1 : 0]	data,
	input wire[WIDTH_SYMBOLS-1:0]		data_is_ctl,
	input wire[WIDTH_SYMBOLS-1:0]		symbol_err,
	input wire[WIDTH_SYMBOLS-1:0]		disparity_err,

	//External trigger (clk domain)
	input wire							trig_in,

	//Trigger output (clk domain)
	output logic						trig_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sanity check bus config

	if(apbData.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();
	if(apbControl.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apbControl.pruser = 0;
	assign apbControl.pbuser = 0;

	assign apbData.pruser = 0;
	assign apbData.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sample packing into memory words

	//Each symbol needs 11 bits of storage space in the BRAM. We support 1 to 4 byte lanes of width
	//The buffer is always 44 bits wide, padded out to 64 for readback

	//Pack 1, 2, or 4 samples per clock into the write buffer
	logic				wr_valid = 0;
	logic[43:0] 		wr_data = 0;
	logic[1:0]			wr_idx = 0;

	always_ff @(posedge clk) begin

		wr_valid						<= 0;

		//Single word datapath
		if(WIDTH_SYMBOLS == 1) begin

			wr_data[8*wr_idx +: 8]		<= data;
			wr_data[wr_idx + 32]		<= data_is_ctl;
			wr_data[wr_idx + 36]		<= symbol_err;
			wr_data[wr_idx + 40]		<= disparity_err;

			wr_idx 						<= wr_idx + 1;
			if(wr_idx == 3)
				wr_valid				<= 1;
		end

		//Double word datapath
		else if(WIDTH_SYMBOLS == 2) begin

			wr_data[16*wr_idx +: 16]	<= data;
			wr_data[wr_idx*2 + 32 +: 2]	<= data_is_ctl;
			wr_data[wr_idx*2 + 36 +: 2]	<= symbol_err;
			wr_data[wr_idx*2 + 40 +: 2]	<= disparity_err;

			if(wr_idx) begin
				wr_valid				<= 1;
				wr_idx					<= 0;
			end
			else
				wr_idx					<= 1;

		end

		//Quad word datapath
		else if(WIDTH_SYMBOLS == 4) begin
			wr_valid					<= 1;
			wr_data						<= { disparity_err, symbol_err, data_is_ctl, data };
		end

		else
			datapath_width_is_invalid();

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Capture buffer

	localparam DEPTH_WORDS = DEPTH / 4;
	localparam ROW_BITS = $clog2(DEPTH_WORDS);

	//The buffer itself
	logic[43:0] capture[DEPTH_WORDS-1:0];

	//Write any time we have valid data and are in the capturing state
	logic				writing	= 0;
	logic				wr_en	= 0;
	logic[ROW_BITS-1:0]	wr_addr = 0;

	always_comb begin
		wr_en	= writing & wr_valid;
	end

	//Write logic
	always_ff @(posedge clk) begin
		if(wr_en)
			capture[wr_addr]	<= wr_data;
	end

	logic				rd_en;
	logic[ROW_BITS-1:0]	rd_addr;
	logic[43:0]			rd_data;

	always_ff @(posedge apbData.pclk) begin
		if(rd_en)
			rd_data		<= capture[rd_addr];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write side trigger combining

	logic	trigger;

	//for now just use ext trigger
	always_comb begin
		//trigger = trig_in;

		trigger = (data[7:0] != 8'hbc);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write side control state machine

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

		if(writing && wr_en)
			wr_addr	<= wr_addr_next;

		case(ila_state)

			//Pre trigger: writing initial samples into the buffer, not yet looking for trigger events
			ILA_STATE_PRE_TRIG: begin
				writing	<= 1;

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
					trigger_endpos	<= wr_addr + DEPTH_WORDS - trigger_offset_words_sync;
					ila_state		<= ILA_STATE_POST_TRIG;
				end

			end //ILA_STATE_ARMED

			ILA_STATE_POST_TRIG: begin

				if(wr_en && (wr_addr_next == trigger_endpos) ) begin
					writing			<= 0;
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
	logic[ROW_BITS-1:0] trigger_offset_words = DEPTH_WORDS / 2;

	RegisterSynchronizer #(
		.WIDTH(ROW_BITS),
		.INIT(DEPTH_WORDS / 2)
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

	always_comb begin

		apbControl.pready	= apbControl.psel && apbControl.penable;
		apbControl.prdata	= 0;
		apbControl.pslverr	= 0;

		if(apbControl.pready) begin

			//read
			if(!apbControl.pwrite) begin
				case(apbControl.paddr)
					REG_TRIGGER:		apbControl.prdata	= { 30'h0, ila_ready_sticky, 1'b0 } ;
					REG_TRIGGER_IDX:	apbControl.prdata	= trigger_pos_sync;
					REG_DATA_BASE:		apbControl.prdata	= DATA_BUF_ADDR;
					REG_DEPTH:			apbControl.prdata	= DEPTH;
					REG_RATE:			apbControl.prdata	= PS_PER_SYMBOL;
					REG_TRIG_POS:		apbControl.prdata	= trigger_offset_words;
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
			trigger_offset_words 	<= DEPTH_WORDS / 2;
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

	logic apbData_pready_next;

	always_comb begin

		//Ready one cycle after requesting a transaction
		apbData_pready_next = apbData.psel && apbData.penable;

		//Read any time an APB read comes in
		rd_en				= apbData_pready_next && !apbData.pwrite;

		//Bits 1:0 of APB address are byte within the word, ignore
		//Bit 2 is low/high word within the memory row
		rd_addr				= apbData.paddr[apbData.ADDR_WIDTH-1 : 3];

		//Reject all writes
		apbData.pslverr		= apbData.pready && apbData.pwrite;

		//Mux output
		if(apbData.paddr[2])
			apbData.prdata	= rd_data[43:32];
		else
			apbData.prdata	= rd_data[31:0];

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
