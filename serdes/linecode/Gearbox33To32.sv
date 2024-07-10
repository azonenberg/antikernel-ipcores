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
	@brief Gearboxes a 33-bit data stream to 32 bits

	Input data must come as a sequence of 64 valid words followed by two idle words
 */
module Gearbox33To32(
	input wire			clk,
	input wire[32:0]	data_in,
	input wire			valid_in,

	output logic[31:0]	data_out	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The gearbox

	logic			valid_in_ff	= 0;

	//Append new data to the existing buffer
	logic[6:0]		work_valid	= 0;
	logic[6:0]		combined_valid;
	logic[95:0] 	combined;
	logic[63:0]		workbuf		= 0;
	always_comb begin

		//Clear all unset bits to zero
		combined			= 0;

		//Copy working buffer
		combined[32 +: 64]	= workbuf;
		combined_valid		= work_valid;

		//Add new data (left justified)
		if(valid_in) begin

			//if recovering from a stall, flush things out
			if(!valid_in_ff) begin
				combined[63 +: 33]				= data_in;
				combined_valid					= 33;
			end

			//explicit dontcare for impossible states to optimize synthesis
			else if(work_valid >= 64) begin
			end

			else begin
				combined[(63-work_valid) +: 33]	= data_in;
				combined_valid					= work_valid + 33;
			end

		end

	end

	always_ff @(posedge clk) begin

		valid_in_ff	<= valid_in;

		//Mirror output to SERDES bit ordering
		for(integer i=0; i<32; i++)
			data_out[i] <= combined[95 - i];

		workbuf		<= combined[63:0];
		work_valid	<= combined_valid - 32;

	end

endmodule
