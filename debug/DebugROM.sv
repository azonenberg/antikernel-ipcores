`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg                                                                          *
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
	@brief A ROM describing the attached debug IPs (up to 16)
 */
module DebugROM #(
	parameter DEVICE_0_TYPE		= 32'h0,
	parameter DEVICE_0_ADDR		= 32'h0,

	parameter DEVICE_1_TYPE		= 32'h0,
	parameter DEVICE_1_ADDR		= 32'h0,

	parameter DEVICE_2_TYPE		= 32'h0,
	parameter DEVICE_2_ADDR		= 32'h0,

	parameter DEVICE_3_TYPE		= 32'h0,
	parameter DEVICE_3_ADDR		= 32'h0,

	parameter DEVICE_4_TYPE		= 32'h0,
	parameter DEVICE_4_ADDR		= 32'h0,

	parameter DEVICE_5_TYPE		= 32'h0,
	parameter DEVICE_5_ADDR		= 32'h0,

	parameter DEVICE_6_TYPE		= 32'h0,
	parameter DEVICE_6_ADDR		= 32'h0,

	parameter DEVICE_7_TYPE		= 32'h0,
	parameter DEVICE_7_ADDR		= 32'h0,

	parameter DEVICE_8_TYPE		= 32'h0,
	parameter DEVICE_8_ADDR		= 32'h0,

	parameter DEVICE_9_TYPE		= 32'h0,
	parameter DEVICE_9_ADDR		= 32'h0,

	parameter DEVICE_10_TYPE	= 32'h0,
	parameter DEVICE_10_ADDR	= 32'h0,

	parameter DEVICE_11_TYPE	= 32'h0,
	parameter DEVICE_11_ADDR	= 32'h0,

	parameter DEVICE_12_TYPE	= 32'h0,
	parameter DEVICE_12_ADDR	= 32'h0,

	parameter DEVICE_13_TYPE	= 32'h0,
	parameter DEVICE_13_ADDR	= 32'h0,

	parameter DEVICE_14_TYPE	= 32'h0,
	parameter DEVICE_14_ADDR	= 32'h0,

	parameter DEVICE_15_TYPE	= 32'h0,
	parameter DEVICE_15_ADDR	= 32'h0
)(
	//The APB bus
	APB.completer 					apb
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Endianness-swap the FOURCC strings

	//For some reason vivado doesn't like this for synthesis
	localparam DEVICE_0_TYPE_BSWAP  = {<<8{DEVICE_0_TYPE}};
	localparam DEVICE_1_TYPE_BSWAP  = {<<8{DEVICE_1_TYPE}};
	localparam DEVICE_2_TYPE_BSWAP  = {<<8{DEVICE_2_TYPE}};
	localparam DEVICE_3_TYPE_BSWAP  = {<<8{DEVICE_3_TYPE}};
	localparam DEVICE_4_TYPE_BSWAP  = {<<8{DEVICE_4_TYPE}};
	localparam DEVICE_5_TYPE_BSWAP  = {<<8{DEVICE_5_TYPE}};
	localparam DEVICE_6_TYPE_BSWAP  = {<<8{DEVICE_6_TYPE}};
	localparam DEVICE_7_TYPE_BSWAP  = {<<8{DEVICE_7_TYPE}};
	localparam DEVICE_8_TYPE_BSWAP  = {<<8{DEVICE_8_TYPE}};
	localparam DEVICE_9_TYPE_BSWAP  = {<<8{DEVICE_9_TYPE}};
	localparam DEVICE_10_TYPE_BSWAP = {<<8{DEVICE_10_TYPE}};
	localparam DEVICE_11_TYPE_BSWAP = {<<8{DEVICE_11_TYPE}};
	localparam DEVICE_12_TYPE_BSWAP = {<<8{DEVICE_12_TYPE}};
	localparam DEVICE_13_TYPE_BSWAP = {<<8{DEVICE_13_TYPE}};
	localparam DEVICE_14_TYPE_BSWAP = {<<8{DEVICE_14_TYPE}};
	localparam DEVICE_15_TYPE_BSWAP = {<<8{DEVICE_15_TYPE}};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Fill the ROM

	logic[31:0] rom [63:0];

	initial begin
		rom[0]		<= 32'd16;	//16 rom table entries supported for now
		rom[1]		<= 32'h0;	//reserved

		rom[2]		<= DEVICE_0_TYPE_BSWAP;
		rom[3]		<= DEVICE_0_ADDR;
		rom[4]		<= DEVICE_1_TYPE_BSWAP;
		rom[5]		<= DEVICE_1_ADDR;
		rom[6]		<= DEVICE_2_TYPE_BSWAP;
		rom[7]		<= DEVICE_2_ADDR;
		rom[8]		<= DEVICE_3_TYPE_BSWAP;
		rom[9]		<= DEVICE_3_ADDR;
		rom[10]		<= DEVICE_4_TYPE_BSWAP;
		rom[11]		<= DEVICE_4_ADDR;
		rom[12]		<= DEVICE_5_TYPE_BSWAP;
		rom[13]		<= DEVICE_5_ADDR;
		rom[14]		<= DEVICE_6_TYPE_BSWAP;
		rom[15]		<= DEVICE_6_ADDR;
		rom[16]		<= DEVICE_7_TYPE_BSWAP;
		rom[17]		<= DEVICE_7_ADDR;
		rom[18]		<= DEVICE_8_TYPE_BSWAP;
		rom[19]		<= DEVICE_8_ADDR;
		rom[20]		<= DEVICE_9_TYPE_BSWAP;
		rom[21]		<= DEVICE_9_ADDR;
		rom[22]		<= DEVICE_10_TYPE_BSWAP;
		rom[23]		<= DEVICE_10_ADDR;
		rom[24]		<= DEVICE_11_TYPE_BSWAP;
		rom[25]		<= DEVICE_11_ADDR;
		rom[26]		<= DEVICE_12_TYPE_BSWAP;
		rom[27]		<= DEVICE_12_ADDR;
		rom[28]		<= DEVICE_13_TYPE_BSWAP;
		rom[29]		<= DEVICE_13_ADDR;
		rom[30]		<= DEVICE_14_TYPE_BSWAP;
		rom[31]		<= DEVICE_14_ADDR;
		rom[32]		<= DEVICE_15_TYPE_BSWAP;
		rom[33]		<= DEVICE_15_ADDR;

		for(integer i=34; i<64; i++)
			rom[i]	<= 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Registered reads to allow for ROM to be implemented as a block RAM if it's big enough

	always_ff @(posedge apb.pclk) begin

		//Clear flags
		apb.pready	<= 0;
		apb.prdata	<= 0;
		apb.pslverr	<= 0;

		if(apb.psel && apb.penable) begin

			apb.pready		<= 1;

			//Reject all writes
			if(apb.pwrite)
				apb.pslverr <= 1;

			//Reads
			else
				apb.prdata <= rom[ apb.paddr[apb.ADDR_WIDTH-1 : 2] ];

		end

	end

endmodule
