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
	parameter DEVICE_3_ADDR		= 32'h0
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

					//Invalid address
					default: apb.pslverr = 1;

				endcase
			end

		end

	end

endmodule
