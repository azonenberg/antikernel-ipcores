`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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
	@brief ATM-polynomial CRC-8 (derived from easics.com generator)

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
	   * polynomial: x^8 + x^2 + x^1 + 1
	   * data width: 32

	 Info : tools@easics.be
	        http://www.easics.com
 */
module CRC8_ATM(clk, reset, update, din, crc, crc_first24);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O declarations

	input wire clk;
	input wire reset;

	input wire update;
	input wire[31:0] din;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual CRC function

	//Internal CRC state (assuming we're processing all 32 input bits)
	output reg[7:0] crc = 8'h00;

	//Output CRC value (processing din[31:8] only)
	output reg[7:0] crc_first24 = 0;

	wire[23:0] din_first24 = din[31:8];

	always @(posedge clk) begin
		if(reset)
			crc <= 'h00;
		if(update) begin

			crc[0] <= din[31] ^ din[30] ^ din[28] ^ din[23] ^ din[21] ^ din[19] ^ din[18] ^ din[16] ^ din[14] ^ din[12] ^ din[8] ^ din[7] ^ din[6] ^ din[0] ^ crc[4] ^ crc[6] ^ crc[7];
			crc[1] <= din[30] ^ din[29] ^ din[28] ^ din[24] ^ din[23] ^ din[22] ^ din[21] ^ din[20] ^ din[18] ^ din[17] ^ din[16] ^ din[15] ^ din[14] ^ din[13] ^ din[12] ^ din[9] ^ din[6] ^ din[1] ^ din[0] ^ crc[0] ^ crc[4] ^ crc[5] ^ crc[6];
			crc[2] <= din[29] ^ din[28] ^ din[25] ^ din[24] ^ din[22] ^ din[17] ^ din[15] ^ din[13] ^ din[12] ^ din[10] ^ din[8] ^ din[6] ^ din[2] ^ din[1] ^ din[0] ^ crc[0] ^ crc[1] ^ crc[4] ^ crc[5];
			crc[3] <= din[30] ^ din[29] ^ din[26] ^ din[25] ^ din[23] ^ din[18] ^ din[16] ^ din[14] ^ din[13] ^ din[11] ^ din[9] ^ din[7] ^ din[3] ^ din[2] ^ din[1] ^ crc[1] ^ crc[2] ^ crc[5] ^ crc[6];
			crc[4] <= din[31] ^ din[30] ^ din[27] ^ din[26] ^ din[24] ^ din[19] ^ din[17] ^ din[15] ^ din[14] ^ din[12] ^ din[10] ^ din[8] ^ din[4] ^ din[3] ^ din[2] ^ crc[0] ^ crc[2] ^ crc[3] ^ crc[6] ^ crc[7];
			crc[5] <= din[31] ^ din[28] ^ din[27] ^ din[25] ^ din[20] ^ din[18] ^ din[16] ^ din[15] ^ din[13] ^ din[11] ^ din[9] ^ din[5] ^ din[4] ^ din[3] ^ crc[1] ^ crc[3] ^ crc[4] ^ crc[7];
			crc[6] <= din[29] ^ din[28] ^ din[26] ^ din[21] ^ din[19] ^ din[17] ^ din[16] ^ din[14] ^ din[12] ^ din[10] ^ din[6] ^ din[5] ^ din[4] ^ crc[2] ^ crc[4] ^ crc[5];
			crc[7] <= din[30] ^ din[29] ^ din[27] ^ din[22] ^ din[20] ^ din[18] ^ din[17] ^ din[15] ^ din[13] ^ din[11] ^ din[7] ^ din[6] ^ din[5] ^ crc[3] ^ crc[5] ^ crc[6];

			crc_first24[0] <= din_first24[23] ^ din_first24[21] ^ din_first24[19] ^ din_first24[18] ^ din_first24[16] ^ din_first24[14] ^ din_first24[12] ^ din_first24[8] ^ din_first24[7] ^ din_first24[6] ^ din_first24[0] ^ crc[0] ^ crc[2] ^ crc[3] ^ crc[5] ^ crc[7];
			crc_first24[1] <= din_first24[23] ^ din_first24[22] ^ din_first24[21] ^ din_first24[20] ^ din_first24[18] ^ din_first24[17] ^ din_first24[16] ^ din_first24[15] ^ din_first24[14] ^ din_first24[13] ^ din_first24[12] ^ din_first24[9] ^ din_first24[6] ^ din_first24[1] ^ din_first24[0] ^ crc[0] ^ crc[1] ^ crc[2] ^ crc[4] ^ crc[5] ^ crc[6] ^ crc[7];
			crc_first24[2] <= din_first24[22] ^ din_first24[17] ^ din_first24[15] ^ din_first24[13] ^ din_first24[12] ^ din_first24[10] ^ din_first24[8] ^ din_first24[6] ^ din_first24[2] ^ din_first24[1] ^ din_first24[0] ^ crc[1] ^ crc[6];
			crc_first24[3] <= din_first24[23] ^ din_first24[18] ^ din_first24[16] ^ din_first24[14] ^ din_first24[13] ^ din_first24[11] ^ din_first24[9] ^ din_first24[7] ^ din_first24[3] ^ din_first24[2] ^ din_first24[1] ^ crc[0] ^ crc[2] ^ crc[7];
			crc_first24[4] <= din_first24[19] ^ din_first24[17] ^ din_first24[15] ^ din_first24[14] ^ din_first24[12] ^ din_first24[10] ^ din_first24[8] ^ din_first24[4] ^ din_first24[3] ^ din_first24[2] ^ crc[1] ^ crc[3];
			crc_first24[5] <= din_first24[20] ^ din_first24[18] ^ din_first24[16] ^ din_first24[15] ^ din_first24[13] ^ din_first24[11] ^ din_first24[9] ^ din_first24[5] ^ din_first24[4] ^ din_first24[3] ^ crc[0] ^ crc[2] ^ crc[4];
			crc_first24[6] <= din_first24[21] ^ din_first24[19] ^ din_first24[17] ^ din_first24[16] ^ din_first24[14] ^ din_first24[12] ^ din_first24[10] ^ din_first24[6] ^ din_first24[5] ^ din_first24[4] ^ crc[0] ^ crc[1] ^ crc[3] ^ crc[5];
			crc_first24[7] <= din_first24[22] ^ din_first24[20] ^ din_first24[18] ^ din_first24[17] ^ din_first24[15] ^ din_first24[13] ^ din_first24[11] ^ din_first24[7] ^ din_first24[6] ^ din_first24[5] ^ crc[1] ^ crc[2] ^ crc[4] ^ crc[6];

		end
	end

endmodule
