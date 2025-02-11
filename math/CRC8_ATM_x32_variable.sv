`timescale 1ns / 1ps
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
	@brief ATM/CCITT CRC-8 engine

	x^8 + x^2 + x^1 + 1

	Accepts 0 to 4 bytes per clock

	LEFT_ALIGN=1: data left aligned in din
		din_len				Action
		0					No-op
		1					Process din[31:24]
		2					Process din[31:16]
		3					Process din[31:8]
		4...7				Process din[31:0]

	LEFT_ALIGN=0: data right aligned in din
		din_len				Action
		0					No-op
		1					Process din[7:0]
		2					Process din[15:0]
		3					Process din[23:0]
		4...7				Process din[31:0]
 */
module CRC8_ATM_x32_variable #(
	parameter LEFT_ALIGN = 1
)(
	input wire			clk,

	input wire			reset,
	input wire[2:0]		din_len,
	input wire[31:0]	din,

	output wire[7:0]	crc_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ATM CRC32

	logic[7:0]	crc;

	//Bit twiddling for the output
	//Complement and flip bit ordering
	assign crc_out =
	{
		~crc[0],  ~crc[1],  ~crc[2],  ~crc[3],  ~crc[4],  ~crc[5],  ~crc[6],  ~crc[7]
	};


	logic[7:0]	current_byte;
	logic		lsb;

	always_ff @(posedge clk) begin

		if(reset)
			crc	= 8'hff;

		for(integer nbyte=0; nbyte<4; nbyte++) begin

			if(nbyte < din_len) begin

				if(LEFT_ALIGN) begin
					//Need to swap byte ordering to match canonical Ethernet ordering (LSB sent first)
					//rather than the "sensible" MSB-LSB order
					current_byte = din[(3-nbyte)*8 +: 8];
				end
				else begin
					current_byte = din[nbyte*8 +: 8];
				end

				for(integer nbit=0; nbit<8; nbit++) begin

					//Default to shifting left and mixing in the new bit
					lsb 	= current_byte[nbit] ^ crc[7];
					crc		= { crc[6:0], lsb };

					//XOR in the polynomial
					crc[1]	= lsb ^ crc[1];
					crc[2]	= lsb ^ crc[2];
				end

			end

		end

	end


endmodule
