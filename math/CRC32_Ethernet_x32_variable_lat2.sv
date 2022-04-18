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
	@brief Ethernet CRC-32 engine with 2-cycle latency

	Accepts 0 to 4 bytes per clock, left aligned in din

	din_len must be 0 or 4 for every cycle except the last.

		din_len				Action
		0					No-op
		1					Process din[31:24]
		2					Process din[31:16]
		3					Process din[31:8]
		4...7				Process din[31:0]
 */
module CRC32_Ethernet_x32_variable_lat2(
	input wire			clk,

	input wire			ce,
	input wire			reset,
	input wire[2:0]		din_len,
	input wire[31:0]	din,

	output logic[31:0]	crc_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet CRC32

	logic[31:0]	crc;

	logic[7:0]	current_byte;
	logic		lsb;

	logic[2:0]	len_ff	= 0;


	logic[31:0]	crc_temp[3:0];

	always_ff @(posedge clk) begin

		len_ff	<= din_len;

		if(reset)
			crc	= 32'hffffffff;				//equivalent to complementing the first 32 bits of the frame
											//as per 802.3 3.2.9 (a)

		else if(din_len != 0 && ce) begin

			for(integer nbyte=0; nbyte<4; nbyte++) begin

				//Need to swap byte ordering to match canonical Ethernet ordering (LSB sent first)
				//rather than the "sensible" MSB-LSB order
				current_byte = din[(3-nbyte)*8 +: 8];

				for(integer nbit=0; nbit<8; nbit++) begin

					//Default to shifting left and mixing in the new bit
					lsb 	= current_byte[nbit] ^ crc[31];
					crc		= { crc[30:0], lsb };

					//XOR in the polynomial
					crc[1]	= lsb ^ crc[1];
					crc[2]	= lsb ^ crc[2];
					crc[4]	= lsb ^ crc[4];
					crc[5]	= lsb ^ crc[5];
					crc[7]	= lsb ^ crc[7];
					crc[8]	= lsb ^ crc[8];
					crc[10] = lsb ^ crc[10];
					crc[11] = lsb ^ crc[11];
					crc[12] = lsb ^ crc[12];
					crc[16] = lsb ^ crc[16];
					crc[22] = lsb ^ crc[22];
					crc[23] = lsb ^ crc[23];
					crc[26] = lsb ^ crc[26];

				end

				//Save the intermediate CRC after each byte of input
				crc_temp[nbyte]	<= crc;

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output muxing, bit renumbering, and inversion

	logic[31:0] fcrc;

	always_ff @(posedge clk) begin

		if(ce) begin

			case(len_ff)
				1:			fcrc	= ~crc_temp[0];
				2:			fcrc	= ~crc_temp[1];
				3:			fcrc	= ~crc_temp[2];
				4:			fcrc	= ~crc_temp[3];
				default:	fcrc	= ~crc;
			endcase

			crc_out <=
			{
				fcrc[24], fcrc[25], fcrc[26], fcrc[27], fcrc[28], fcrc[29], fcrc[30], fcrc[31],
				fcrc[16], fcrc[17], fcrc[18], fcrc[19], fcrc[20], fcrc[21], fcrc[22], fcrc[23],
				fcrc[8],  fcrc[9],  fcrc[10], fcrc[11], fcrc[12], fcrc[13], fcrc[14], fcrc[15],
				fcrc[0],  fcrc[1],  fcrc[2],  fcrc[3],  fcrc[4],  fcrc[5],  fcrc[6],  fcrc[7]
			};
		end

	end


endmodule
