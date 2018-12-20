`timescale 1ns / 1ps
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
	@brief DDS squarewave oscillator for producing arbitrary frequencies (with some jitter) in simulation.

	Periods may be specified with resolution smaller than the simulation time scale. This may be useful for generating
	extremely high frequency clocks.

	Instead of delays being measured in clock cycles as with SquarewaveDDS, they're measured in picoseconds.
 */
module UnclockedSimulationSquarewaveDDS(
	input wire			sync_rst,

	input wire[31:0]	real_part,
	input wire[31:0]	frac_part,

	output logic		dout		= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	logic[32:0]	frac_accum	= 0;

	always begin

		#(0.001 * real_part);
		frac_accum = frac_accum + frac_part;
		if(frac_accum[32]) begin
			frac_accum[32]	= 0;
			#0.001;
		end
		dout = 0;

		#(0.001 * real_part);
		frac_accum = frac_accum + frac_part;
		if(frac_accum[32]) begin
			frac_accum[32]	= 0;
			#0.001;
		end
		dout = 1;

	end

endmodule
