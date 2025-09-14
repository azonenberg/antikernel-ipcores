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
	@brief Ganged collection of DDR I/O buffers for a parallel DDR input bus

	Supports 7 series only for now.
 */
module DDRInputBuffer #(
	parameter WIDTH = 16
) (
	input wire				clk_p,
	input wire				clk_n,

	input wire[WIDTH-1:0]	din,

	output wire[WIDTH-1:0]	dout0,
	output wire[WIDTH-1:0]	dout1

	`ifdef EFINIX
	, input wire			clk_pipe
	`endif
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The IO buffers

	for(genvar i=0; i<WIDTH; i++) begin: buffers

		`ifdef XILINX_ULTRASCALEPLUS

			IDDRE1 #
			(
				.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
				.IS_C_INVERTED(0),
				.IS_CB_INVERTED(0)
			) ddr_ibuf (
				.C(clk_p),
				.CB(clk_n),
				.Q1(dout0[i]),
				.Q2(dout1[i]),
				.R(1'b0),
				.D(din[i])
			);

			`define OK

		`endif

		`ifdef XILINX_7SERIES

			IDDR #
			(
				.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
				.SRTYPE("ASYNC"),
				.INIT_Q1(0),
				.INIT_Q2(0)
			) ddr_ibuf (
				.C(clk_p),
				.Q1(dout0[i]),
				.Q2(dout1[i]),
				.CE(1'b1),
				.R(1'b0),
				.S(1'b0),
				.D(din[i])
			);

			`define OK

		`endif

		`ifdef EFINIX

			EFX_IDDIO #(
				.IS_CLK_INVERTED(0),
				.PULL_OPTION("NONE"),
				.MODE("DDIO_RESYNC")
			) ddr_ibuf (
				.O_HI(dout0_tmp[i]),
				.O_LO(dout1[i]),
				.CLK(clk_p),
				.I(din[i])
			);

			`define OK

		`endif

		`ifndef OK
			$fatal(1, "DDRInputBuffer: unrecognized device family (did you forget to define XILINX_7SERIES or XILINX_ULTRASCALEPLUS?)");
		`endif

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Extra pipeline registers for Trion

	`ifdef EFINIX
		//DDIO_PIPE isn't working (is it even there??), need to manually pipeline
		wire[WIDTH-1:0]		dout0_tmp;
		logic[WIDTH-1:0]	dout0_tmp_ff;

		always_ff @(posedge clk_pipe) begin
			dout0_tmp_ff	<= dout0_tmp;
		end

		assign dout0 	= dout0_tmp_ff;
	`endif

endmodule
