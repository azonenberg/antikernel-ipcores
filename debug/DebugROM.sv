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
	// Combinatorial reads

	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//Reject all writes
			if(apb.pwrite)
				apb.pslverr = 1;

			//Reads
			else begin
				case(apb.paddr)

					'h00: apb.prdata = DEVICE_0_TYPE;
					'h04: apb.prdata = DEVICE_0_ADDR;
					'h08: apb.prdata = DEVICE_1_TYPE;
					'h0c: apb.prdata = DEVICE_1_ADDR;
					'h10: apb.prdata = DEVICE_2_TYPE;
					'h14: apb.prdata = DEVICE_2_ADDR;
					'h18: apb.prdata = DEVICE_3_TYPE;
					'h1c: apb.prdata = DEVICE_3_ADDR;
					'h20: apb.prdata = DEVICE_4_TYPE;
					'h24: apb.prdata = DEVICE_4_ADDR;
					'h28: apb.prdata = DEVICE_5_TYPE;
					'h2c: apb.prdata = DEVICE_5_ADDR;
					'h30: apb.prdata = DEVICE_6_TYPE;
					'h34: apb.prdata = DEVICE_6_ADDR;
					'h38: apb.prdata = DEVICE_7_TYPE;
					'h3c: apb.prdata = DEVICE_7_ADDR;
					'h40: apb.prdata = DEVICE_8_TYPE;
					'h44: apb.prdata = DEVICE_8_ADDR;
					'h48: apb.prdata = DEVICE_9_TYPE;
					'h4c: apb.prdata = DEVICE_9_ADDR;
					'h50: apb.prdata = DEVICE_10_TYPE;
					'h54: apb.prdata = DEVICE_10_ADDR;
					'h58: apb.prdata = DEVICE_11_TYPE;
					'h5c: apb.prdata = DEVICE_11_ADDR;
					'h60: apb.prdata = DEVICE_12_TYPE;
					'h64: apb.prdata = DEVICE_12_ADDR;
					'h68: apb.prdata = DEVICE_13_TYPE;
					'h6c: apb.prdata = DEVICE_13_ADDR;
					'h70: apb.prdata = DEVICE_14_TYPE;
					'h74: apb.prdata = DEVICE_14_ADDR;
					'h78: apb.prdata = DEVICE_15_TYPE;
					'h7c: apb.prdata = DEVICE_15_ADDR;

					//Invalid address
					default: apb.pslverr = 1;

				endcase
			end

		end

	end

endmodule
