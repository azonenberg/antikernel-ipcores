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
	@brief Implements the Internet checksum.

	Input is fed to the checksum 32 bits at a time.
 */

module InternetChecksum32bit(
	input wire			clk,
	input wire			load,
	input wire			reset,
	input wire			process,
	input wire[31:0]	din,
	output reg[15:0]	sumout	= 0,
	output reg[15:0]	csumout	= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Checksum computation

	wire[17:0]	sumout_temp  = din[15:0] + din[31:16] + sumout;			//first stage of summation
	wire[16:0]	sumout_temp2 = sumout_temp[15:0] + sumout_temp[17:16];	//second stage, extra bit to catch carries
	wire[15:0]	sumout_temp3 = sumout_temp2[15:0] + sumout_temp2[16];	//add in the carry, if there is one
																		//(should not be possible to carry out here)

	always @(posedge clk) begin

		if(reset) begin
			sumout	<= 16'h0;
			csumout	<= 16'hffff;
		end

		else if(load) begin
			sumout	<= din[15:0];
			csumout	<= ~din[15:0];
		end

		else if(process) begin
			sumout	<= sumout_temp3;
			csumout	<= ~sumout_temp3;
		end

	end

endmodule
