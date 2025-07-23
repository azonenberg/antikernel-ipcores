`timescale 1ns/1ps
`default_nettype none
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
	@brief X25519 subtractor. Two cycle pipelined implementation.

	Derived from sub() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)

	Additive inverse modulo 2^(255-19) with no reduction.
 */
(* USE_DSP="no" *)
module X25519_Sub(
	input wire			clk,
	input wire			en,
	input wire[263:0]	a,
	input wire[263:0]	b,
	output logic		`ifdef XILINX out_valid = 0 `endif ,
	output logic[263:0]	`ifdef XILINX out = 0 `endif
	);

	//output initialization for efinix toolchain compatibility
	`ifndef XILINX
	initial begin
		out_valid = 0;
		out = 0;
	end
	`endif

	logic	en_ff	= 0;

	logic[128:0]	sum_low = 0;
	logic[135:0]	sum_high = 0;

	always_ff @(posedge clk) begin
		en_ff		<= en;
		out_valid	<= en_ff;

		//calculate a-b+p rather than a-b to avoid possibility of underflow
		//we may get some carry out, that's fine though
		if(en) begin
			sum_low		<= a[127:0] - b[127:0] + 128'hffffffffffffffffffffffffffffffed;
			sum_high	<= a[263:128] - b[263:128] + 128'h7fffffffffffffffffffffffffffffff;
		end

		if(en_ff)
			out			<= { sum_high + sum_low[128], sum_low[127:0] };

	end

endmodule
