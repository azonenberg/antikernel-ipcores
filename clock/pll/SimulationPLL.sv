`timescale 1ps / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
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
	@brief A very simple "PLL" that synthesizes a clock locked to the frequency of an incoming signal
 */
module SimulationPLL(
	input wire			refclk,
	input wire[31:0]	multiplier,

	output wire			clkout
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Measure the speed of the incoming clock

	wire[31:0]	incoming_period;
	SimulationClockPeriodCounter counter(
		.clk(refclk),
		.period_ps(incoming_period)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Calculate period of the outbound clock

	real			incoming_period_f;
	real 			toggle_delay;
	logic[31:0] 	toggle_real;
	real			toggle_delta;
	real			toggle_frac_scaled;
	logic[31:0]		toggle_frac;
	always_comb begin
		incoming_period_f	= $itor(incoming_period);
		toggle_delay		= incoming_period_f / (2 * multiplier);
		toggle_real			= $floor(toggle_delay);
		toggle_delta		= toggle_delay - $floor(toggle_delay);
		toggle_frac_scaled	= toggle_delta * $pow(2, 32);
		toggle_frac			= $rtoi(toggle_frac_scaled);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DDS the up-scaled waveform

	UnclockedSimulationSquarewaveDDS dds(
		.sync_rst(0),
		.real_part(toggle_real),
		.frac_part(toggle_frac),
		.dout(clkout)
	);

endmodule
