`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
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
	@file	PRBS31.v
	@author	Andrew D. Zonenberg
	@brief	Generates a PRBS-31 bit sequence in parallel (one word per clock) with parameterizable width.

	Before use, the PRBS generator must be seeded by asserting "init" for one cycle and loading a 31-bit nonzero value
	into "seed". Using a seed of all zeroes will cause the LFSR to become stuck in the all-zeroes state and not toggle.

	To generate a new random number, assert "update" for one cycle. "dout" will update on the next cycle.

	Asserting "init" and "update" the same cycle is legal, and will work the same as if the re-seeding had occurred
	before the update was requested.
 */
module PRBS31 #(
	parameter WIDTH			= 32,
	parameter MSB_FIRST		= 0,
	parameter INITIAL_SEED	= 0
) (
	input wire				clk,
	input wire				update,
	input wire				init,
	input wire[30:0]		seed,
	output logic[WIDTH-1:0]	dout
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	logic[30:0]			state = INITIAL_SEED;

	logic[30:0]			state_comb;
	logic				xorout;

	logic[WIDTH-1:0]	dout_comb;

	always_ff @(posedge clk) begin

		//Synchronous reset
		if(init)
			state	<= seed;

		//Save configuration
		if(update) begin
			dout	<= dout_comb;
			state	<= state_comb;
		end

	end

	always_comb begin

		//Run the LFSR by N cycles every time we update
		if(init)
			state_comb = seed;
		else
			state_comb = state;

		dout_comb = dout;

		for(integer i=0; i<WIDTH; i=i+1) begin
			xorout = state_comb[30] ^ state_comb[27];

			if(MSB_FIRST)
				dout_comb[WIDTH-1-i]	= xorout;
			else
				dout_comb[i]	= xorout;

			state_comb	= { state_comb[29:0], xorout };
		end

	end

endmodule
