`timescale 1ns/1ps
`default_nettype none
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
	@brief	One stage of a reduction tree for summing a large number of numbers
 */
module ReductionAdder #(
	parameter IN_WORD_WIDTH		= 32,
	parameter OUT_WORD_WIDTH	= 32,
	parameter IN_BLOCKS			= 16,
	parameter REDUCTION			= 4,

	localparam OUT_BLOCKS		= (IN_BLOCKS + REDUCTION-1) / REDUCTION	//divide and round up
) (
	input wire									clk,

	input wire									en,
	input wire[IN_WORD_WIDTH*IN_BLOCKS-1:0]		din,

	output logic								dout_valid,
	output logic[OUT_WORD_WIDTH*OUT_BLOCKS-1:0]	dout
	);

	//output initialization for efinix toolchain compatibility
	initial begin
		dout_valid = 0;
		dout = 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Adder tree

	logic[OUT_WORD_WIDTH*OUT_BLOCKS-1:0]	dout_comb	= 0;

	logic[OUT_WORD_WIDTH-1:0]	tmp;

	always_comb begin
		for(integer i=0; i < OUT_BLOCKS; i=i+1) begin

			tmp = 0;

			for(integer j=0; j < REDUCTION; j++)
				tmp = tmp + din[(i*REDUCTION + j)*IN_WORD_WIDTH +: IN_WORD_WIDTH];

			dout_comb[i*OUT_WORD_WIDTH +: OUT_WORD_WIDTH] = tmp;

		end
	end

	always_ff @(posedge clk) begin
		dout_valid	<= en;
		dout		<= dout_comb;
	end

endmodule
