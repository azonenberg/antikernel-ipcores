`default_nettype none
`timescale 1ns/1ps
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
	@brief Generate clocks for use by the oversampling module
 */
module OversamplingClocking(

	//Inputs from global clock tree
	input wire	clk_125mhz,

	//Clock outputs
	output wire clk_156p25mhz,		//only used for transmit, but convenient to get from same PLL
	output wire	clk_312p5mhz,
	output wire clk_625mhz_fabric,
	output wire	clk_625mhz_io_0,
	output wire	clk_625mhz_io_90,

	output wire	pll_lock,
	output wire	align_done
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main PLL

	wire	clk_unused;

	wire	phase_shift_en;
	wire	phase_shift_inc;
	wire	phase_shift_done;

	ReconfigurablePLL #(
		.IN0_PERIOD(8),			//125 MHz input
		.IN1_PERIOD(8),

		.OUTPUT_BUF_GLOBAL( 6'b011100),
		.OUTPUT_BUF_IO(		6'b000011),
		.OUTPUT_GATE(		6'b011100),

		.OUT0_MIN_PERIOD(1.6),	//625 MHz output to IO clock network
		.OUT1_MIN_PERIOD(1.6),	//625 MHz output to IO clock network
		.OUT2_MIN_PERIOD(1.6),	//625 MHz output to fabric
		.OUT3_MIN_PERIOD(3.2),	//312.5 MHz output to fabric
		.OUT4_MIN_PERIOD(6.4),	//156.25 MHz output to fabric (for transmit side coding)
		.OUT5_MIN_PERIOD(3.2),	//312.5 MHz output (unused)

		.OUT0_DEFAULT_PHASE(0),
		.OUT1_DEFAULT_PHASE(90),
		.OUT2_DEFAULT_PHASE(0),
		.OUT3_DEFAULT_PHASE(0),
		.OUT4_DEFAULT_PHASE(0),
		.OUT5_DEFAULT_PHASE(0),

		.FINE_PHASE_SHIFT(6'b001100),

		.ACTIVE_ON_START(1)		//Start PLL on power up
	) pll (
		.clkin({clk_125mhz, clk_125mhz}),
		.clksel(1'b0),

		.clkout({clk_unused, clk_156p25mhz, clk_312p5mhz, clk_625mhz_fabric, clk_625mhz_io_90, clk_625mhz_io_0}),

		.reset(1'b0),
		.locked(pll_lock),

		.busy(),
		.reconfig_clk(clk_125mhz),
		.reconfig_start(1'b0),
		.reconfig_finish(1'b0),
		.reconfig_cmd_done(),

		.reconfig_vco_en(1'b0),
		.reconfig_vco_mult(7'b0),
		.reconfig_vco_indiv(7'b0),
		.reconfig_vco_bandwidth(1'b0),

		.reconfig_output_en(1'b0),
		.reconfig_output_idx(3'b0),
		.reconfig_output_div(8'b0),
		.reconfig_output_phase(9'b0),

		.phase_shift_clk(clk_312p5mhz),
		.phase_shift_en(phase_shift_en),
		.phase_shift_inc(phase_shift_inc),
		.phase_shift_done(phase_shift_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Phase control

	OversamplingPhaseAlignment phase_ctl(
		.clk_312p5mhz(clk_312p5mhz),
		.clk_625mhz_fabric(clk_625mhz_fabric),
		.clk_625mhz_io_0(clk_625mhz_io_0),

		.phase_shift_en(phase_shift_en),
		.phase_shift_inc(phase_shift_inc),
		.phase_shift_done(phase_shift_done),

		.done(align_done)
	);

endmodule
