`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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
	@brief Descrambler for 10Gbase-R 64/66b line code
 */
module Descrambler64b66b(
	input wire		clk,

	input wire			valid_in,
	input wire[1:0]		header_in,
	input wire[63:0]	data_in,

	output logic		valid_out = 0,
	output logic[1:0]	header_out = 0,
	output logic[63:0]	data_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Descrambler

	logic[57:0]	rx_scramble			= 0;
	logic[63:0]	descrambled;

	always_ff @(posedge clk) begin

		valid_out	<= valid_in;
		header_out	<= header_in;

		if(valid_in) begin
			for(integer i=63; i>=0; i--) begin
				descrambled[i]	= data_in[i] ^ rx_scramble[38] ^ rx_scramble[57];
				rx_scramble		= { rx_scramble[56:0], data_in[i] };
			end
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Flip bits MSB-LSB within each byte

	always_comb begin
		for(integer nbyte=0; nbyte<8; nbyte = nbyte+1) begin
			for(integer nbit=0; nbit<8; nbit = nbit+1)
				data_out[nbyte*8 + nbit] = descrambled[nbyte*8 + (7-nbit)];
		end
	end

endmodule
