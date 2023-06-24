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
	@brief Hamming FEC encoder

	Developed and tested for a (72, 64) code. Might work for other code sizes, but not tested
 */
module HammingEncoder #(

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
	input wire[DATA_BITS-1:0]		data_in,

	output logic					valid_out	= 0,
	output logic[BLOCK_BITS-1:0]	data_out	=0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	integer dwptr;
	integer dcount;

	logic	check;

	always_ff @(posedge clk) begin

		//Pipeline valid flag
		valid_out	<= valid_in;

		//Bit position zero, and all powers of two, are reserved for check bits
		//Shove data bits into the remaining slots
		//Messy loop but all gets unrolled at synthesis time to static assignments
		dwptr = 3;
		for(dcount = 0; dcount < DATA_BITS; dcount = dcount + 1) begin

			//Write next data bit
			data_out[dwptr]	= data_in[dcount];

			//Find next valid write position
			//If it's a power of two, skip and move to the next.
			//(cannot be two consecutive powers of two since we start at 3)
			dwptr = dwptr + 1;
			if( (dwptr & dwptr-1) == 0)
				dwptr = dwptr + 1;

		end

		//Check bit N for N=1...FEC_BITS:
		//Located at bit position 2^n of the message (CB0 is at bit 1, CB1 is at bit 2, CB2 is at bit 4, etc)
		for(integer i=0; i<FEC_BITS; i=i+1) begin
			check = 0;

			//XOR together all message bits with a 1 in this position
			for(integer j = 2**i + 1; j<BLOCK_BITS; j=j+1) begin
				if(j & 2**i)
					check = check ^ data_out[j];
			end

			data_out[2 ** i] = check;
		end

		//Bit 0 is global parity: XOR reduction of the entire code block including other check bits
		data_out[0] = ^data_out[BLOCK_BITS-1 : 1];

	end

endmodule
