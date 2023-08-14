`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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
	@brief A DSP48 based 48-bit performance counter

	TODO: offer non-DSP version too
 */
module PerformanceCounter(
	input wire			clk,
	input wire			en,
	input wire[2:0]		delta,

	input wire			rst,

	output wire[47:0]	count);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//we want Y mux to select C
	//and Z mux to select P

	logic	en_ff = 0;
	always_ff @(posedge clk) begin
		en_ff	<= en;
	end

	DSP48E1 #(
		.ACASCREG(1),
		.ADREG(1),
		.ALUMODEREG(0),
		.AREG(1),
		.BCASCREG(1),
		.BREG(1),
		.CARRYINREG(1),
		.CARRYINSELREG(1),
		.CREG(1),
		.DREG(1),
		.INMODEREG(0),
		.MREG(0),
		.OPMODEREG(1),
		.PREG(1),
		.A_INPUT("DIRECT"),
		.B_INPUT("DIRECT"),
		.USE_DPORT("FALSE"),
		.USE_MULT("NONE"),
		.USE_SIMD("ONE48"),
		.AUTORESET_PATDET("NO_RESET"),
		.MASK(48'h0),
		.PATTERN(48'h0),
		.SEL_MASK("MASK"),
		.SEL_PATTERN("PATTERN"),
		.USE_PATTERN_DETECT("NO_PATDET")
	) slice (

		.CLK(clk),

		//zillions of resets
		.RSTA(rst),
		.RSTALLCARRYIN(rst),
		.RSTALUMODE(rst),
		.RSTB(rst),
		.RSTC(rst),
		.RSTCTRL(rst),
		.RSTD(rst),
		.RSTINMODE(rst),
		.RSTM(rst),
		.RSTP(rst),

		.ALUMODE(4'h0),					//Z+X+Y+CIN, CIN=0, X=P, Y=delta, Z=0, X=0
		.CEALUMODE(1'b0),
		.CEINMODE(1'b0),
		.CEC(en),
		.CEP(en_ff),
		.CECTRL(en),
		.INMODE(5'h0a),					//not using A so anything goes
		.OPMODE({7'b000_11_10}),		//zmux=0, ymux=c, xmux=p

		.C({45'h0, delta}),
		.P(count),

		//multiplier and pre-adder not used
		.A(30'h3fffffff),
		.B(18'h3ffff),
		.CEA1(1'b0),
		.CEA2(1'b0),
		.CEAD(1'b0),
		.CEB1(1'b0),
		.CEB2(1'b0),
		.CED(1'b0),
		.CEM(1'b0),
		.D(25'h0),

		//Cascade ports (not used)
		.ACIN(30'h0),
		.ACOUT(),
		.BCIN(18'h0),
		.BCOUT(),
		.CARRYCASCIN(1'h0),
		.CARRYCASCOUT(),
		.CARRYIN(1'b0),
		.CARRYINSEL(3'b0),
		.CARRYOUT(),
		.CECARRYIN(1'b0),
		.MULTSIGNIN(1'b0),
		.MULTSIGNOUT(),
		.OVERFLOW(),
		.PCIN(48'h0),
		.PCOUT(),
		.UNDERFLOW(),

		//Pattern detect (not used)
		.PATTERNBDETECT(),
		.PATTERNDETECT()
	);

endmodule
