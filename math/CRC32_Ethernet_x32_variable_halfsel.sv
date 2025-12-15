`timescale 1ns / 1ps
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
	@brief Ethernet CRC-32 engine

	Accepts 2 or 4 bytes per clock, left aligned in din
 */
module CRC32_Ethernet_x32_variable_halfsel(
	input wire			clk,

	input wire			reset,
	input wire			ce,
	input wire			halfwidth,
	input wire[31:0]	din,

	output wire[31:0]	crc_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet CRC32

	logic[31:0]	crc;
	logic[31:0]	crc_tmp;

	//Bit twiddling for the output
	//Complement and flip bit ordering
	assign crc_out =
	{
		~crc[24], ~crc[25], ~crc[26], ~crc[27], ~crc[28], ~crc[29], ~crc[30], ~crc[31],
		~crc[16], ~crc[17], ~crc[18], ~crc[19], ~crc[20], ~crc[21], ~crc[22], ~crc[23],
		~crc[8],  ~crc[9],  ~crc[10], ~crc[11], ~crc[12], ~crc[13], ~crc[14], ~crc[15],
		~crc[0],  ~crc[1],  ~crc[2],  ~crc[3],  ~crc[4],  ~crc[5],  ~crc[6],  ~crc[7]
	};


	logic[7:0]	current_byte;
	logic		lsb;

	always_ff @(posedge clk) begin

		crc_tmp	= crc;

		for(integer nbyte=0; nbyte<4; nbyte++) begin

			if( (nbyte < 2) || !halfwidth) begin

				//Need to swap byte ordering to match canonical Ethernet ordering (LSB sent first)
				//rather than the "sensible" MSB-LSB order
				current_byte = din[(3-nbyte)*8 +: 8];

				for(integer nbit=0; nbit<8; nbit++) begin

					//Default to shifting left and mixing in the new bit
					lsb 	= current_byte[nbit] ^ crc_tmp[31];
					crc_tmp		= { crc_tmp[30:0], lsb };

					//XOR in the polynomial
					crc_tmp[1]	= lsb ^ crc_tmp[1];
					crc_tmp[2]	= lsb ^ crc_tmp[2];
					crc_tmp[4]	= lsb ^ crc_tmp[4];
					crc_tmp[5]	= lsb ^ crc_tmp[5];
					crc_tmp[7]	= lsb ^ crc_tmp[7];
					crc_tmp[8]	= lsb ^ crc_tmp[8];
					crc_tmp[10] = lsb ^ crc_tmp[10];
					crc_tmp[11] = lsb ^ crc_tmp[11];
					crc_tmp[12] = lsb ^ crc_tmp[12];
					crc_tmp[16] = lsb ^ crc_tmp[16];
					crc_tmp[22] = lsb ^ crc_tmp[22];
					crc_tmp[23] = lsb ^ crc_tmp[23];
					crc_tmp[26] = lsb ^ crc_tmp[26];

				end

			end

		end

		if(ce) begin
			if(reset)
				crc	<= 32'hffffffff;
			else
				crc	<= crc_tmp;
		end

	end


endmodule
