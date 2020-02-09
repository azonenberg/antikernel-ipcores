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

	parameter integer 						DEPTH					= 1024,			//Number of samples to capture
	localparam integer						ADDR_BITS				= $clog2(DEPTH)	//Number of bits in a pointer
) (
	input wire								clk,				//Capture clock
	input wire[CHANNELS-1:0][MAX_WIDTH-1:0]	probe_in,			//Probe signals

	input wire[CHANNELS-1:0][MAX_WIDTH-1:0]	compare_target,		//Comparison targets for trigger comparators
	input wire ila_compare_t[CHANNELS-1:0]	compare_mode,		//Comparison modes for trigger comparators
	input wire								compare_match_all,	//true = match all triggers at once
																//false = match any trigger
	input wire								use_ext_trig,		//if true, ignore internal trigger block
																//and just use trig_in instead

	input wire[ADDR_BITS-1:0]				trig_offset,		//time point of trigger in capture buffer

	input wire								trig_in,
	output logic							trig_out	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Figure out position of each probe within the ILA memory

	typedef integer indexes_t[CHANNELS-1:0];

	function integer CalcTotalWidth;
		integer w;
		begin
			w = 0;
			for(integer i=0; i<CHANNELS; i++)
				w += WIDTHS[i];
			return w;
		end
	endfunction

	function indexes_t CalcLowerBound;
		integer base;
		indexes_t ret;
		begin
			base = 0;
			for(integer i=0; i<CHANNELS; i++) begin
				ret[i] = base;
				base += WIDTHS[i];
			end
			return ret;
		end
	endfunction

	function indexes_t CalcUpperBound;
		integer base;
		indexes_t ret;
		begin
			base = 0;
			for(integer i=0; i<CHANNELS; i++) begin
				base += WIDTHS[i];
				ret[i] = base - 1;
			end
			return ret;
		end
	endfunction

	//Total width of all probes
	localparam TOTAL_WIDTH = CalcTotalWidth();

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

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Signal memory

	logic[TOTAL_WIDTH-1:0] capture_buf[DEPTH-1:0];

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Capture logic

endmodule
