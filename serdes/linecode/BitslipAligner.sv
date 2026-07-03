`default_nettype none
`timescale 1ns/1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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
	@brief Bitslip module for aligning incoming serial data to symbol/word boundaries without changing its width
 */
module BitslipAligner #(
	parameter WIDTH		= 20,
	parameter REVERSE	= 1
)(
	input wire				clk,
	input wire[WIDTH-1:0]	data_in,
	input wire				bitslip,

	output wire[WIDTH-1:0]	data_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output registers

	logic[WIDTH-1:0]		data_out_int	= 0;

	assign data_out = data_out_int;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Window of incoming data

	logic[WIDTH*2 - 1 : 0]		window_int;
	logic[WIDTH*2 - 1 : 0]		window;
	logic[WIDTH-1:0]			data_in_ff = 0;

	always_comb begin
		window_int			= { data_in, data_in_ff };

		//Bit reverse the data window
		if(REVERSE) begin
			for(integer i=0; i<WIDTH*2; i=i+1)
				window[i]	= window_int[WIDTH*2-1 - i];
		end
		else
			window 			= window_int;
	end

	always_ff @(posedge clk) begin
		data_in_ff	<= data_in;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Bitslip logic

	logic[$clog2(WIDTH)-1:0]	shiftpos = 0;

	always_ff @(posedge clk) begin

		//Update shift position
		if(bitslip) begin
			if(shiftpos >= (WIDTH - 1) )
				shiftpos	<= 0;
			else
				shiftpos	<= shiftpos + 1;
		end

		//Barrel shift
		data_out_int	<= window[shiftpos +: WIDTH];

	end

endmodule
