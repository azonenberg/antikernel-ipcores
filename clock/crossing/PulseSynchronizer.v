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
	@brief Unidirectional pulse synchronizer for sharing single cycle pulses across clock domains

	Note that it takes several clocks for the pulse to propagate. If clk_a is faster than clk_b, there is a "dead time"
	window in which two consecutive pulses may be read as one.
 */
module PulseSynchronizer(
	input wire	clk_a,
	input wire	pulse_a,

	input wire	clk_b,
	output reg	pulse_b = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit side

	reg		tx_a	= 0;

	//Toggle every time we get a pulse
	always @(posedge clk_a) begin
		if(pulse_a)
			tx_a	<= ~tx_a;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The synchronizer

	wire	rx_a;

	ThreeStageSynchronizer sync
		(.clk_in(clk_a), .din(tx_a), .clk_out(clk_b), .dout(rx_a));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive side

	reg		rx_a_ff	= 0;

	//Pulse every time we get a toggle
	always @(posedge clk_b) begin
		rx_a_ff	<= rx_a;
		pulse_b	<= (rx_a_ff != rx_a);
	end

endmodule
