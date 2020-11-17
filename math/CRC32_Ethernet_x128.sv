`timescale 1ns / 1ps
`default_nettype none
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
 */
module CRC32_Ethernet_x128(
	input wire			clk,
	input wire			reset,
	input wire 			update,
	input wire			last,
	input wire[4:0]		din_len,
	input wire[127:0]	din,
	output wire[31:0]	crc_flipped);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O shuffling

	logic[31:0] crc_out = 0;

	wire[31:0] crc_not = ~crc_out;
	assign crc_flipped =
	{
		crc_not[24], crc_not[25], crc_not[26], crc_not[27],
		crc_not[28], crc_not[29], crc_not[30], crc_not[31],

		crc_not[16], crc_not[17], crc_not[18], crc_not[19],
		crc_not[20], crc_not[21], crc_not[22], crc_not[23],

		crc_not[8], crc_not[9], crc_not[10], crc_not[11],
		crc_not[12], crc_not[13], crc_not[14], crc_not[15],

		crc_not[0], crc_not[1], crc_not[2], crc_not[3],
		crc_not[4], crc_not[5], crc_not[6], crc_not[7]
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input preprocessing

	logic[127:0] din_flipped;

	logic[31:0] crc_in;

	logic[31:0] crc_x128_ff	= 0;

	always_comb begin

		//Start new CRC if requested
		if(reset)
			crc_in = 32'hffffffff;
		else
			crc_in = crc_x128_ff;

		for(integer i=0; i<4; i++) begin
			din_flipped[i*32 +: 32] <=
			{
				din[i*32 + 24], din[i*32 + 25], din[i*32 + 26], din[i*32 + 27],
				din[i*32 + 28], din[i*32 + 29], din[i*32 + 30], din[i*32 + 31],

				din[i*32 + 16], din[i*32 + 17], din[i*32 + 18], din[i*32 + 19],
				din[i*32 + 20], din[i*32 + 21], din[i*32 + 22], din[i*32 + 23],

				din[i*32 + 8], din[i*32 + 9], din[i*32 + 10], din[i*32 + 11],
				din[i*32 + 12], din[i*32 + 13], din[i*32 + 14], din[i*32 + 15],

				din[i*32 + 0], din[i*32 + 1], din[i*32 + 2], din[i*32 + 3],
				din[i*32 + 4], din[i*32 + 5], din[i*32 + 6], din[i*32 + 7]
			};
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CRC calculation blocks

	wire[31:0]	crc_x128;

	//Full blocks
	CRC32_Ethernet_x128_comb crc128(
		.d(din_flipped),
		.c(crc_in),
		.crc(crc_x128)
	);

	logic[127:0]	final_data_in 	= 0;
	logic[31:0]		final_crc_in	= 0;
	logic[3:0]		final_crc_len	= 0;

	wire[31:0] crc_x64;
	wire[31:0] crc_x32;

	CRC32_Ethernet_x64_comb crc64(
		.d(final_data_in[127:64]),
		.c(final_crc_in),
		.crc(crc_x64)
	);

	CRC32_Ethernet_x32_variable_comb crc32(
		.d(final_data_in[127:96]),
		.len(final_crc_len[2:0]),
		.c(final_crc_in),
		.crc(crc_x32)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline registers

	enum logic[1:0]
	{
		FINISH_STATE_IDLE,
		FINISH_STATE_X64,
		FINISH_STATE_X32,
		FINISH_STATE_LAST
	} finish_state = FINISH_STATE_IDLE;

	logic done		= 0;
	logic last_ff	= 0;

	always_ff @(posedge clk) begin

		crc_x128_ff	<= crc_x128;
		last_ff		<= last;

		case(finish_state)

			FINISH_STATE_IDLE: begin

				final_data_in	<= din_flipped;
				final_crc_in	<= crc_x128_ff;
				final_crc_len	<= din_len;
				done			<= 0;

				//Start the completion process on the last word of data
				if(last_ff || (update && !din_len[4] )) begin
					finish_state	<= FINISH_STATE_X64;

					if(last_ff && din_len[4]) begin
						crc_out	<= crc_x128_ff;
						done	<= 1;
					end

				end

			end	//end FINISH_STATE_IDLE

			FINISH_STATE_X64: begin

				//If we have >= 8 bytes left, we want the 64 bit CRC
				if(final_crc_len[3]) begin
					final_crc_in	<= crc_x64;
					final_crc_len	<= final_crc_len[2:0];

					final_data_in	<= { final_data_in[63:0], 64'h0 };

					//If this is the end, save the result
					if(!done && final_crc_len[2:0] == 0) begin
						crc_out		<=  crc_x64;
						done		<= 1;
					end
				end

				finish_state	<= FINISH_STATE_X32;

			end	//end FINISH_STATE_X64

			FINISH_STATE_X32: begin

				//If we have >= 4 bytes left, we want the full 32 bit CRC
				if(final_crc_len[2]) begin

					final_data_in	<= { final_data_in[95:0], 32'h0 };

					final_crc_in	<= crc_x32;
					final_crc_len	<= final_crc_len[1:0];
				end

				//If this is the end, save the result
				if(!done && final_crc_len[1:0] == 0) begin
					crc_out		<= crc_x32;
					done		<= 1;
				end

				finish_state	<= FINISH_STATE_LAST;

			end	//end FINISH_STATE_X32

			FINISH_STATE_LAST: begin
				if(!done)
					crc_out		<= crc_x32;

				finish_state	<= FINISH_STATE_IDLE;
			end	//end FINISH_STATE_LAST

		endcase

	end

endmodule
