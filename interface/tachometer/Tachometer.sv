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
	@brief Reads a standard PC fan tachometer and gives an output in RPM

	Assumes 2 pulses per revolution
 */
module Tachometer#(
	parameter REFCLK_HZ 		= 125000000,

	parameter MIN_PERIOD_TICKS	= 1000,			//threshold for debouncing
	parameter INTEGRATION		= 16			//number of cycles to average for period calculations
) (
	input wire	clk,
	input wire	tach,

	output logic[15:0] rpm = 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize input

	wire	tach_sync;

	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_in (
		.clk_in(clk),
		.din(tach),
		.clk_out(clk),
		.dout(tach_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Measure period from each rising edge to the next

	//Use a X-bit counter
	//Assuming min RPM 60 (1 Hz), 250 MHz max refclk freq, a 28-bit counter is sufficient

	logic		tach_ff			= 0;
	logic[27:0]	count 			= 0;
	logic		period_valid;

	logic		stuck			= 0;

	always_comb begin
		period_valid	= (tach_ff != tach_sync);
	end

	always_ff @(posedge clk) begin
		count		<= count + 1;

		//start looking for toggles after dead time is over
		if(count > MIN_PERIOD_TICKS) begin
			tach_ff 		<= tach_sync;

			if(period_valid) begin
				count		<= 0;
				stuck		<= 0;
			end

		end

		if(count == 28'hFFFFFFF)
			stuck	<= 1;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sum blocks of pulses to get better accuracy

	logic[7:0]	totalcount = 0;
	logic[31:0]	pulsetotal = 0;
	logic		total_valid	= 0;

	always_ff @(posedge clk) begin

		total_valid		<= 0;

		if(period_valid) begin

			//Integrate
			pulsetotal	<= pulsetotal + count;

			//Last pulse in block?
			if(totalcount == (INTEGRATION - 1)) begin
				totalcount	<= 0;
				total_valid	<= 1;
			end
			else
				totalcount	<= totalcount + 1;
		end

		if(total_valid)
			pulsetotal	<= 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Convert integrated period to RPM

	localparam[35:0] FREQ_NUMER = 15 * REFCLK_HZ * INTEGRATION;

	/*
		Average pulse period is (pulsetotal / INTEGRATION) timer ticks
		* 1 Hz / REFCLK_HZ ticks

		so pulse frequency is REFCLK_HZ / (pulsetotal / INTEGRATION)

		4 toggles per revolution
		so rotation frequency is REFCLK_HZ / 4*(pulsetotal / INTEGRATION) Hz
		= (REFCLK_HZ * INTEGRATION) / (4 * pulsetotal)

		60 RPM / 1 Hz
		so RPM is (60 * REFCLK_HZ * INTEGRATION) / (4 * pulsetotal)
		or (15 * REFCLK_HZ * INTEGRATION) / pulsetotal

		Numerator is a synthesis time constant that can be >32 bits

		Pre-divide both sides to get things in range at the cost of losing some precision
	 */
	wire		divdone;
	wire[31:0]	divquot;
	NonPipelinedDivider divider(
		.clk(clk),
		.start(total_valid),
		.dend(FREQ_NUMER / 16),
		.dvsr(pulsetotal / 16),
		.quot(divquot),
		.rem(),
		.busy(),
		.done(divdone),
		.sign(1'b0)
	);

	always_ff @(posedge clk) begin

		if(stuck)
			rpm	<= 0;

		if(divdone)
			rpm	<= divquot;

	end

endmodule
