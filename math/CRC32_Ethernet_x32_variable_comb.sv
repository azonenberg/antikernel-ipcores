`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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
	@brief Ethernet CRC-32 (derived from easics.com generator)

	Original license:

	Copyright (C) 1999-2008 Easics NV.
	 This source file may be used and distributed without restriction
	 provided that this copyright statement is not removed from the file
	 and that any derivative work contains the original copyright notice
	 and the associated disclaimer.

	 THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS
	 OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
	 WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.

	 Purpose : synthesizable CRC function
	   * polynomial: (0 1 2 4 5 7 8 10 11 12 16 22 23 26 32)
	   * data width: 8

	 Info : tools@easics.be
	        http://www.easics.com

	Assumes input bit ordering is already reversed!
 */
module CRC32_Ethernet_x32_variable_comb(
	input wire[31:0] 	d,
	input wire[2:0]		len,
	input wire[31:0] 	c,
	output logic[31:0]	crc);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual CRC function

	logic[7:0]	current_byte;
	logic		lsb;

	always_comb begin
		crc = c;

		for(integer nbyte=0; nbyte<4; nbyte++) begin

			if(nbyte < len) begin

				//Need to swap byte ordering to match canonical Ethernet ordering (LSB sent first)
				//rather than the "sensible" MSB-LSB order
				current_byte = d[(3-nbyte)*8 +: 8];

				for(integer nbit=0; nbit<8; nbit++) begin

					//Default to shifting left and mixing in the new bit
					lsb 	= current_byte[7-nbit] ^ crc[31];
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

			end

		end
	end

endmodule
