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
	@brief Hamming FEC decoder

	Developed and tested for a (72, 64) code. Might work for other code sizes, but not tested
 */
module HammingDecoder #(

	//Number of data bits in the block
	parameter DATA_BITS		= 64,

	//Number of check bits
	localparam FEC_BITS		= $clog2(DATA_BITS)+1,

	//Global parity bit over the entire message
	localparam PARITY_BITS	= 1,

	//Total size of an encoded message
	localparam BLOCK_BITS	= DATA_BITS + FEC_BITS + PARITY_BITS
)(

	input wire						clk,

	input wire						valid_in,
	input wire[BLOCK_BITS-1:0]		data_in,

	output logic					valid_out			= 0,
	output logic[DATA_BITS-1:0]		data_out			= 0,
	output logic					correctable_err		= 0,
	output logic					uncorrectable_err	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	integer drptr;
	integer dcount;

	logic[FEC_BITS-1:0]	check_expected	= 0;
	logic				parity_expected	= 0;

	logic[FEC_BITS-1:0]	check_actual	= 0;
	logic				parity_actual	= 0;

	logic[FEC_BITS-1:0]	check_err		= 0;
	logic				parity_err		= 0;

	always_ff @(posedge clk) begin

		//Clear status flags
		correctable_err		<= 0;
		uncorrectable_err	<= 0;

		//Pipeline valid flag
		valid_out			<= valid_in;

		//Calculated expected check bit values
		for(integer i=0; i<FEC_BITS; i=i+1) begin
			check_expected[i] = 0;

			//XOR together all message bits with a 1 in this position
			for(integer j = 2**i + 1; j<DATA_BITS; j=j+1) begin
				if(j & 2**i)
					check_expected[i] = check_expected[i] ^ data_in[j];
			end

			check_actual[i] = data_in[2**i];
			check_err[i]	= check_actual[i] ^ check_expected[i];
		end

		//Bit 0 is global parity: XOR reduction of the entire code block including other check bits
		parity_expected	= ^data_in[BLOCK_BITS-1 : 1];
		parity_actual 	= data_in[0];
		parity_err		= parity_expected ^ parity_actual;

		//If syndrome is zero but we have a global parity error,
		//or syndrome is nonzero but we do not have a global parity error, we have multiple bit flips
		//(uncorrectable error)
		if( (check_err && !parity_err) || (!check_err && parity_err) )
			uncorrectable_err	<= 1;

		//If syndrome is nonzero and we have a global parity error, we have a single bit flip
		//(correctable error)
		else if(check_err)
			correctable_err		<= 1;

		//else no errors at all

		//Extract data bits
		//Messy loop but all gets unrolled at synthesis time to static assignments
		drptr = 3;
		for(dcount = 0; dcount < DATA_BITS; dcount = dcount + 1) begin

			//If syndrome is nonzero, flip the bit at that position
			if(check_err == drptr)
				data_out[dcount] = !data_in[drptr];

			//Otherwise just copy the bit unchanged
			else
				data_out[dcount] = data_in[drptr];

			//Find next valid write position
			//If it's a power of two, skip and move to the next.
			//(cannot be two consecutive powers of two since we start at 3)
			drptr = drptr + 1;
			if( (drptr & drptr-1) == 0)
				drptr = drptr + 1;

		end

	end

endmodule
