`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
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
	@brief DDS squarewave oscillator for producing arbitrary frequencies (with some jitter) from a reference clock

	Note, real and fractional part are the *toggle* period (half the waveform period).
 */
module SquarewaveDDS(
	input wire			clk,
	input wire			sync_rst,

	input wire[31:0]	real_part,
	input wire[31:0]	frac_part,

	output logic		dout		= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	logic[31:0]	frac_accum	= 0;
	logic[31:0]	count		= 0;
	wire[32:0]	next_accum	= frac_accum + frac_part;

	always_ff @(posedge clk) begin

		//Synchronous reset to phase align the output predictably
		if(sync_rst) begin
			frac_accum	<= 0;
			count		<= 0;
			dout		<= 0;
		end

		//Bump real counter every clock. Fractional only increments on toggles
		count			<= count + 1'h1;

		//Add an extra cycle of delay if the fractional part has a carry out
		if(next_accum[32]) begin
			if(count > real_part) begin
				count		<= 0;
				frac_accum	<= next_accum[31:0];
				dout		<= ~dout;
			end
		end

		//Not carrying but still toggling
		else if(count >= real_part) begin
			count		<= 0;
			frac_accum	<= next_accum[31:0];
			dout		<= ~dout;
		end


	end

endmodule
