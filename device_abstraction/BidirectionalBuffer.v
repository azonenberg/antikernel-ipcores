`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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
	@brief Tristate I/O to two bidirectional buses

	OE_INVERT (default true) means OE=1 output, OE=0 input
 */
module BidirectionalBuffer #(
	parameter WIDTH 	= 1,
	parameter OE_INVERT	= 1
) (
	output wire[WIDTH-1:0]	fabric_in,
	input wire[WIDTH-1:0]	fabric_out,
	inout wire[WIDTH-1:0]	pad,
	input wire				oe
);

	////////////////////////////////////////////////////////////////////////////////////////////////
	// The IO buffers

	genvar i;
	generate
		for(i=0; i<WIDTH; i = i+1) begin: buffers

			`ifdef XILINX
				IOBUF iobuf(
					.I(fabric_out[i]),
					.IO(pad[i]),
					.O(fabric_in[i]),
					.T(OE_INVERT ? (!oe) : (oe))
				);
			`endif

			`ifdef EFINIX
				EFX_IO_BUF #(
					.PULL_OPTION("WEAK_PULLDOWN")	//TODO decide what we want
				) iobuf (
					.I(fabric_out[i]),
					.IO(pad[i]),
					.O(fabric_in[i]),
					.OE(OE_INVERT ? !oe : oe));
			`endif

		end
	endgenerate

endmodule
