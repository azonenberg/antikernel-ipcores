`default_nettype none
`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2022 Andrew D. Zonenberg                                                                          *
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
	@brief A clock buffer
 */
module ClockBuffer(clkin, ce, clkout);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O / parameter declarations

	parameter TYPE = "LOCAL";	//Set to LOCAL, GLOBAL, REGIONAL, or IO
								//LOCAL is a hint and may not always be possible
	parameter CE = "YES";		//Set to YES or NO
								//If NO, ce input is ignored

	input wire	clkin;
	input wire	ce;

	output wire	clkout;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual primitive

	wire ce_internal = (CE == "YES") ? ce : 1'b1;

	generate

		//Local clock (one region of the device)
		if(TYPE == "LOCAL") begin

			//For Xilinx Spartan-6 or 7 series: Use a BUFH (TODO: Support other FPGAs)
			if(CE == "NO")
				BUFH clk_buf(.I(clkin), .O(clkout));

			//If we have a clock enable, we have to use a BUFG for Spartan-6 since it lacks BUFHCE
			else if(CE == "YES") begin
				`ifdef XILINX_SPARTAN6
					BUFGCE clk_buf(.I(clkin), .O(clkout), .CE(ce));
					initial begin
						$warning("Using BUFGCE instead of BUFHCE for ClockBuffer TYPE=\"LOCAL\", CE=\"YES\" since S6 has no BUFHCE");
					end
				`else
					BUFHCE clk_buf(.I(clkin), .O(clkout), .CE(ce));
				`endif
			end

			//Parameter error
			else begin
				initial begin
					$fatal(0, "ERROR: ClockBuffer CE argument must be \"YES\" or \"NO\"");
				end
			end

		end

		//Global clock (entire device)
		else if(TYPE == "GLOBAL") begin

			//For Xilinx Spartan-6 or 7 series: Use a BUFG (TODO: Support other FPGAs)
			if(CE == "NO") begin
				(* DONT_TOUCH = "true" *)	//force the buffer to not get optimized out
				BUFG clk_buf(.I(clkin), .O(clkout));
			end

			//Use a BUFG for all Xilinx FPGAs
			else if(CE == "YES") begin
				(* DONT_TOUCH = "true" *)	//force the buffer to not get optimized out
				BUFGCE clk_buf(.I(clkin), .O(clkout), .CE(ce));
			end

			//Parameter error
			else begin
				initial begin
					$fatal(0, "ERROR: ClockBuffer CE argument must be \"YES\" or \"NO\"");
				end
			end

		end

		//High-speed I/O clock
		else if(TYPE == "IO") begin

			if(CE == "NO") begin
				(* DONT_TOUCH = "true" *)	//force the buffer to not get optimized out
				BUFIO clk_buf(.I(clkin), .O(clkout));
			end

			//Parameter error
			else begin
				initial begin
					$fatal(0, "ERROR: ClockBuffer CE argument must be \"NO\" for TYPE == \"IO\"");
				end
			end

		end

		//Regional clock (7 series)
		else if(TYPE == "REGIONAL") begin

			(* DONT_TOUCH = "true" *)	//force the buffer to not get optimized out
			BUFR #(
				.BUFR_DIVIDE("BYPASS")
			) clk_buf (
				.I(clkin),
				.O(clkout),
				.CE(ce_internal),
				.CLR(1'b0)
				);

		end

		//Parameter error
		else begin
			initial begin
				$fatal(0, "ERROR: ClockBuffer TYPE argument must be \"GLOBAL\" or \"LOCAL\"");
			end
		end

	endgenerate

endmodule

