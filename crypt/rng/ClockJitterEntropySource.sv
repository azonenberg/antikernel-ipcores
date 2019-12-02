`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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
	@brief Entropy source for RandomNumberGenerator based on clock jitter
 */
module ClockJitterEntropySource(

	//Main generator clock
	input wire			clk,

	//Second clock for jitter reference
	input wire			clk_ring,

	//Output
	output logic[31:0]	jitter_word			= 0,
	output logic		jitter_word_valid	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Generate some entropy from jitter between STARTUPE2 clock and the external clock

	logic	toggle_ring	= 0;
	always_ff @(posedge clk_ring) begin
		toggle_ring	<= !toggle_ring;
	end

	wire	toggle_sync;
	ThreeStageSynchronizer sync_toggle_ring(
		.clk_in(clk_ring),
		.din(toggle_ring),
		.clk_out(clk),
		.dout(toggle_sync)
	);

	logic[9:0]	count = 0;
	logic[1:0]	jitter_bits			= 0;
	logic		jitter_bit			= 0;
	logic		jitter_bit_valid	= 0;

	logic[4:0]	jitter_word_count	= 0;
	always_ff @(posedge clk) begin

		jitter_bit_valid		<= 0;
		jitter_word_valid		<= 0;

		//Sample one bit of clk_ring every 512 clk cycles
		count					<= count + 1;
		if(count == 0) begin
			jitter_bits[0]		<= toggle_sync;

			// Apply minimal whitening with a von Neumann corrector
			if(jitter_bits == 2'b10) begin
				jitter_bit_valid	<= 1;
				jitter_bit			<= 1;
			end
			else if(jitter_bits == 2'b01) begin
				jitter_bit_valid	<= 1;
				jitter_bit			<= 0;
			end

		end
		if(count == 512)
			jitter_bits[1]		<= toggle_sync;

		//Shift bitstream into 32-bit words
		if(jitter_bit_valid) begin
			jitter_word_count	<= jitter_word_count + 1;
			jitter_word			<= { jitter_word[30:0], jitter_bit };;
			if(jitter_word_count == 31)
				jitter_word_valid	<= 1;
		end
	end

endmodule
