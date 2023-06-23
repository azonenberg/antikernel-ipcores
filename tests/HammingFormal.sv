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
	@file
	@author Andrew D. Zonenberg
	@brief Formal verification testbench for HammingEncoder and HammingDecoder
 */
module HammingFormal(
	input wire			clk,
	input wire[63:0]	plaintext,
	input wire			plaintext_valid,

	input wire[1:0]		numbits,				//number of bits to corrupt
	input wire[6:0]		errpos0,				//index of first bit to corrupt
	input wire[6:0]		errpos1					//index of second bit to corrupt
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Encoder

	wire		codeword_valid;
	wire[71:0]	codeword;

	HammingEncoder #(
		.DATA_BITS(64)
	) encoder (
		.clk(clk),
		.data_in(plaintext),
		.valid_in(plaintext_valid),
		.valid_out(codeword_valid),
		.data_out(codeword)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Error injection

	logic		corrupted_valid = 0;
	logic[71:0]	corrupted = 0;

	logic[1:0]	numbits	= 0;
	logic[6:0]	bitpos	= 0;

	always_ff @(posedge clk) begin
		corrupted_valid	<= codeword_valid;
		corrupted		<= codeword;

		//Can corrupt 0-2 bits, but not more than that as this is a SEC-DED code
		//and more than two errors might go undetected
		assume(numbits != 2);

		//If we're corrupting two bits, they can't be at the same place
		if(numbits == 2)
			assume(errpos0 != errpos1);

		//Bit position to corrupt must be in range
		assume(errpos0 < 72);
		assume(errpos1 < 72);

		//Actually inject the errors
		if(numbits >= 1)
			corrupted[errpos0] <= !codeword[errpos0];
		if(numbits == 2)
			corrupted[errpos1] <= !codeword[errpos1];

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decoder

	wire		decoded_valid;
	wire[63:0]	decoded;

	wire		correctable_err;
	wire		uncorrectable_err;

	HammingDecoder #(
		.DATA_BITS(64)
	) decoder (
		.clk(clk),
		.data_in(corrupted),
		.valid_in(corrupted_valid),
		.valid_out(decoded_valid),
		.data_out(decoded),
		.correctable_err(correctable_err),
		.uncorrectable_err(uncorrectable_err)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Result verification

	always_ff @(posedge clk) begin

		if(decoded_valid) begin

			//Should have had valid input
			assert($past(corrupted_valid));

			//Should have a correctable error if we flipped 1 bit, and uncorrectable if 2
			assert(correctable_err == ($past(numbits, 2) == 1) );
			assert(uncorrectable_err == ($past(numbits, 2) == 2) );

			//If we did not have an uncorrectable error, the decoder should output the original message
			if(!uncorrectable_err)
				assert(decoded == $past(plaintext, 3));

		end

	end

endmodule
