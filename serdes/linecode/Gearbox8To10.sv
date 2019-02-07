`default_nettype none
`timescale 1ns/1ps

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
	@brief 8-to-10 gearbox

	Intended use case is recovering 10-bit symbols from an ISERDES that only provides 8:1 serialization.

	Assumes "printer friendly" bit ordering (bit 7 is first on wire).
	Note that this is mirrored from what most SERDES IPs expect!
 */
module Gearbox8To10(
	input wire			clk,

	input wire			din_valid,
	input wire[7:0]		din,

	input wire			bitslip,

	output logic		dout_valid	= 0,
	output logic[9:0]	dout
);

	logic[3:0]	temp_valid	= 0;
	logic[7:0]	temp_buf	= 0;	//max 8 bits, if we hit 10 we'd have sent last cycle

	//Figure out current capacity including the buffered and inbound data
	logic[15:0]	buffer_fwd;
	logic[4:0]	valid_fwd;
	always_comb begin

		if(bitslip) begin
			buffer_fwd	<= { 1'b0, temp_buf, din[7:1] };
			valid_fwd	<= temp_valid + 4'd7;
		end

		else begin
			buffer_fwd	<= { temp_buf, din };
			valid_fwd	<= temp_valid + 4'd8;
		end

	end

	//Emit a completed word as necessary, then save whatever is left
	always_ff @(posedge clk) begin
		dout_valid	<= 0;

		if(din_valid) begin

			//If we have at least ten bits to send, we're sending something
			if(valid_fwd >= 10) begin
				dout_valid	<= 1;
				temp_valid	<= valid_fwd - 5'd10;
			end
			else
				temp_valid	<= valid_fwd;

			case(valid_fwd)

				//Max capacity is 8 bits in temp buffer plus 8 arriving = 16
				16: begin
					dout		<= buffer_fwd[15:6];
					temp_buf	<= {10'h0, buffer_fwd[5:0]};
				end

				15: begin
					dout		<= buffer_fwd[14:5];
					temp_buf	<= {11'h0, buffer_fwd[4:0]};
				end

				14: begin
					dout		<= buffer_fwd[13:4];
					temp_buf	<= {12'h0, buffer_fwd[3:0]};
				end

				13: begin
					dout		<= buffer_fwd[12:3];
					temp_buf	<= {13'h0, buffer_fwd[2:0]};
				end

				12: begin
					dout		<= buffer_fwd[11:2];
					temp_buf	<= {14'h0, buffer_fwd[1:0]};
				end

				11: begin
					dout		<= buffer_fwd[10:1];
					temp_buf	<= {15'h0, buffer_fwd[0]};
				end

				10: begin
					dout		<= buffer_fwd[9:0];
					temp_buf	<= 0;
				end

				//Not enough data to send.
				default: begin
					temp_buf	<= buffer_fwd;
				end

			endcase

		end

	end

endmodule
