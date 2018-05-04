`timescale 1ns / 1ps
`default_nettype none
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
	@brief Ethernet CRC-32 engine

	Accepts 0 to 4 bytes per clock, left aligned in din
		din_len				Action
		0					No-op
		1					Process din[31:24]
		2					Process din[31:16]
		3					Process din[31:8]
		4					Process din[31:0]
		5..7				Illegal, ignored

 */
module CRC32_Ethernet_x32_variable(
	input wire			clk,

	input wire			reset,
	input wire[2:0]		din_len,
	input wire[31:0]	din,

	output wire[31:0]	crc_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet CRC32

	integer 	nbyte;
	integer 	nbit;

	reg[31:0]	crc;

	//Bit twiddling for the output
	//Complement and flip bit ordering
	assign crc_out =
	{
		~crc[24], ~crc[25], ~crc[26], ~crc[27], ~crc[28], ~crc[29], ~crc[30], ~crc[31],
		~crc[16], ~crc[17], ~crc[18], ~crc[19], ~crc[20], ~crc[21], ~crc[22], ~crc[23],
		~crc[8],  ~crc[9],  ~crc[10], ~crc[11], ~crc[12], ~crc[13], ~crc[14], ~crc[15],
		~crc[0],  ~crc[1],  ~crc[2],  ~crc[3],  ~crc[4],  ~crc[5],  ~crc[6],  ~crc[7]
	};


	reg[7:0]	current_byte;
	reg			lsb;

	always @(posedge clk) begin

		if(reset)
			crc	= 32'hffffffff;				//equivalent to complementing the first 32 bits of the frame
											//as per 802.3 3.2.9 (a)

		for(nbyte=0; nbyte<4; nbyte = nbyte + 1) begin

			if(nbyte < din_len) begin

				//Need to swap byte ordering to match canonical Ethernet ordering (LSB sent first)
				//rather than the "sensible" MSB-LSB order
				current_byte = din[(3-nbyte)*8 +: 8];

				for(nbit=0; nbit<8; nbit = nbit + 1) begin

					//Default to shifting left and mixing in the new bit
					lsb = current_byte[nbit] ^ crc[31];
					crc	= { crc[30:0], lsb };

					//XOR in the polynomial
					crc[1]	 = lsb ^ crc[1];
					crc[2]	 = lsb ^ crc[2];
					crc[4]	 = lsb ^ crc[4];
					crc[5]	 = lsb ^ crc[5];
					crc[7]	 = lsb ^ crc[7];
					crc[8]	 = lsb ^ crc[8];
					crc[10] = lsb ^ crc[10];
					crc[11] = lsb ^ crc[11];
					crc[12] = lsb ^ crc[12];
					crc[16] = lsb ^ crc[16];
					crc[22] = lsb ^ crc[22];
					crc[23] = lsb ^ crc[23];
					crc[26] = lsb ^ crc[26];

				end

			end

		end

	end


endmodule
