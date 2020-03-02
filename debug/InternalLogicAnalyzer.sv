`default_nettype none
`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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

`include "InternalLogicAnalyzer_types.svh"

/**
	@brief Internal logic analyzer for debug
 */
module InternalLogicAnalyzer #(
	parameter integer						CHANNELS 				= 1,			//Number of channels
	parameter integer						WIDTHS[CHANNELS-1:0] 	= '{0},			//Width of each channel
	parameter integer						NAME_LEN				= 16,			//Maximum length of a name, in characters
	localparam integer						NAME_BITS				= NAME_LEN*8,	//Maximum length of a name, in bits
	parameter[NAME_BITS-1:0]				NAMES[CHANNELS-1:0],					//Display name of each channel
																					//(truncated to X chars)
	parameter integer						MAX_WIDTH				= 1024,			//Maximum legal width of any channel
	parameter integer						TOTAL_WIDTH				= 1,			//Total width of all channels

	parameter integer 						DEPTH					= 1024,				//Number of samples to capture
	localparam integer						ADDR_BITS				= $clog2(DEPTH),	//Number of bits in a pointer
	localparam integer						CHANNEL_BITS			= $clog2(CHANNELS)	//Number of bits in a channel ID
) (
	input wire								clk,						//Capture clock
	input wire[CHANNELS-1:0][MAX_WIDTH-1:0]	probe_in,					//Probe signals

	input wire[CHANNELS-1:0][MAX_WIDTH-1:0]	compare_target,				//Comparison targets for trigger comparators
	input wire ila_compare_t[CHANNELS-1:0]	compare_mode,				//Comparison modes for trigger comparators
	input wire								compare_match_all,			//true = match all triggers at once
																		//false = match any trigger
	input wire								use_ext_trig,				//if true, ignore internal trigger block
																		//and just use trig_in instead

	input wire								trig_armed,					//Trigger is ignored unless this is high
	input wire								trig_force,					//Set high for one cycle to force a trigger
																		//(bypasses normal trigger subsystem)
	input wire[ADDR_BITS-1:0]				trig_offset,				//time point of trigger in capture buffer

	input wire								trig_in,					//external trigger input
	output logic							trig_out	= 0,			//trigger status output
	output ila_status_t						status		= STATUS_IDLE,	//current LA state

	input wire								symtab_rd_en,
	input wire[CHANNEL_BITS-1:0]			symtab_rd_addr,
	output logic[NAME_BITS-1:0]				symtab_rd_data	= 0,

	input wire								data_rd_en,
	input wire[ADDR_BITS-1:0]				data_rd_addr,
	output logic[TOTAL_WIDTH-1:0]			data_rd_data	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Figure out position of each probe within the ILA memory

	typedef integer indexes_t[CHANNELS-1:0];

	`include "InternalLogicAnalyzer_functions.svh"

	//Lower / upper bit index of each probe
	localparam indexes_t PROBE_LOW = CalcLowerBound();
	localparam indexes_t PROBE_HIGH = CalcUpperBound();

	initial begin

		//Debug print configuration

		$info("ILA has %d channels (%d total bits)", CHANNELS, TOTAL_WIDTH);
		for(integer i=0; i<CHANNELS; i++)
			$display("Channel %d (%s) is %d bits wide (from %d to %d)",
				i, NAMES[i], WIDTHS[i], PROBE_LOW[i], PROBE_HIGH[i]);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Symbol table memory

	logic[NAME_BITS-1:0]	symbol_table[CHANNELS-1:0];

	initial begin
		for(integer i=0; i<CHANNELS; i++)
			symbol_table[i] = NAMES[i];
	end

	always_ff @(posedge clk) begin
		if(symtab_rd_en)
			symtab_rd_data	<= symbol_table[symtab_rd_addr];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Trigger system

	logic[CHANNELS-1:0]		comparator_match = 0;

	always_ff @(posedge clk) begin

		//Phase 1: detect trigger event
		for(integer i=0; i<CHANNELS; i++) begin
			case(compare_mode[i])
				COMPARE_ALWAYS:		comparator_match[i] <= 1;
				COMPARE_NEVER:		comparator_match[i] <= 0;
				COMPARE_EQUAL:		comparator_match[i] <= (probe_in[i] == compare_target[i]);
				COMPARE_LEQUAL:		comparator_match[i] <= (probe_in[i] <= compare_target[i]);
				COMPARE_GEQUAL:		comparator_match[i] <= (probe_in[i] >= compare_target[i]);
				COMPARE_LESS:		comparator_match[i] <= (probe_in[i] < compare_target[i]);
				COMPARE_GREATER:	comparator_match[i] <= (probe_in[i] > compare_target[i]);
				COMPARE_UNEQUAL:	comparator_match[i] <= (probe_in[i] != compare_target[i]);
				default:			comparator_match[i] <= 0;
			endcase
		end

		//Phase 2: combine all triggers
		if(use_ext_trig)
			trig_out	<= trig_in;
		else if(compare_match_all)
			trig_out	<= &comparator_match;
		else
			trig_out	<= |comparator_match;

		//If trigger isn't armed, never trigger
		if(!trig_armed)
			trig_out	<= 0;

		//Force trigger overrides everything else
		if(trig_force)
			trig_out	<= 1;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Signal memory

	logic[TOTAL_WIDTH-1:0]	capture_wdata	= 0;
	logic					capture_we		= 0;

	logic[ADDR_BITS-1:0]	base_wptr		= 0;
	logic[ADDR_BITS-1:0]	capture_wptr	= 0;

	logic[TOTAL_WIDTH-1:0]	capture_buf[DEPTH-1:0];

	always_ff @(posedge clk) begin
		if(capture_we)
			capture_buf[capture_wptr]	<= capture_wdata;
		if(data_rd_en)
			data_rd_data	<= capture_buf[base_wptr + data_rd_addr];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Capture logic

	enum logic[3:0]
	{
		STATE_IDLE		= 0,
		STATE_PRE_TRIG	= 1,
		STATE_READY		= 2,
		STATE_CAPTURE	= 3,
		STATE_DONE		= 4
	} state;

	always_ff @(posedge clk) begin

		//Register capture data
		for(integer i=0; i<CHANNELS; i++)
			capture_wdata[PROBE_HIGH[i] : PROBE_LOW[i]] <= probe_in[i][WIDTHS[i]-1:0];

		case(state)

			STATE_IDLE: begin

				//When trigger is armed, begin capturing the "head" area
				if(trig_armed) begin
					state			<= STATE_PRE_TRIG;
					capture_wptr	<= 0;
				end

			end	//end STATE_IDLE

			//Capture the pre-trigger stuff before even looking for a trigger
			STATE_PRE_TRIG: begin
				capture_wptr	<= capture_wptr + 1;
				capture_we		<= 1;
				if(capture_wptr == trig_offset) begin
					status		<= STATUS_ARMED;
					state		<= STATE_READY;
				end
			end	//end STATE_PRE_TRIG

			STATE_READY: begin

				//Continue capturing the rolling buffer until we trigger
				capture_wptr	<= capture_wptr	+ 1;
				capture_we		<= 1;

				if(trig_out) begin
					base_wptr	<= capture_wptr - trig_offset;	//nominal start of the capture
					status		<= STATUS_CAPTURING;
					state		<= STATE_CAPTURE;
				end

			end	//end STATE_READY

			STATE_CAPTURE: begin

				if( (capture_wptr + 1) == base_wptr) begin
					status			<= STATUS_DONE;
					state			<= STATE_DONE;
				end

				//Continue capturing the rolling buffer until we hit the end
				else begin
					capture_wptr	<= capture_wptr	+ 1;
					capture_we		<= 1;
				end

			end	//end STATE_CAPTURE

			STATE_DONE: begin
				if(!trig_armed) begin
					state	<= STATE_IDLE;
					status	<= STATUS_DONE;
				end
			end	//end STATE_DONE

		endcase
	end

endmodule
