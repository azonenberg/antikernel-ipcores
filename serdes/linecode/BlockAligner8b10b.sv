`default_nettype none
`timescale 1ns/1ps
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief 8B/10B block aligner

	Currently intended for 7 series GTP using 32-bit datapath and 16-bit internal width.

	Ensures commas are always in the lane 0 position.
 */
module BlockAligner8b10b #(
	parameter COMMA_CHAR = 8'hbc	//K28.5
) (
	input wire			clk,

	input wire[3:0]		kchar_in,
	input wire[31:0]	data_in,

	output logic[3:0]	kchar_out,
	output logic[31:0]	data_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline delay on input data

	logic[3:0]	kchar_ff	= 0;
	logic[31:0]	data_ff		= 0;

	always_ff @(posedge clk) begin
		kchar_ff	<= kchar_in;
		data_ff		<= data_in;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Find commas and figure out if they're in the low or high half

	logic		comma_lane	= 0;

	always_ff @(posedge clk) begin

		if( (data_in[7:0] == COMMA_CHAR) && kchar_in[0])
			comma_lane	<= 0;

		if( (data_in[23:16] == COMMA_CHAR) && kchar_in[2])
			comma_lane	<= 1;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output mux

	always_comb begin

		if(comma_lane == 0) begin
			kchar_out	= kchar_in;
			data_out	= data_in;
		end

		//Phase shift the data
		else begin
			kchar_out	= { kchar_in[1:0], kchar_ff[3:2] };
			data_out	= { data_in[15:0], data_ff[31:16] };
		end
	end

endmodule
