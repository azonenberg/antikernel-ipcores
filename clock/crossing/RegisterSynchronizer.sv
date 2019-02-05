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
	@brief Wrapper around PulseSynchronizer for synchronizing a register from a management/JTAG domain to
	a SoC internal domain.
 */
module RegisterSynchronizer #(
	parameter WIDTH = 16,
	parameter INIT = 0
) (
	input wire				clk_a,
	input wire				en_a,
	output wire				ack_a,
	input wire[WIDTH-1:0]	reg_a,

	input wire				clk_b,
	output logic			updated_b 	= 0,

	(* DONT_TOUCH *)
	output logic[WIDTH-1:0]	reg_b		= INIT
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control plane

	wire	update_b;

	PulseSynchronizer sync_en(
		.clk_a(clk_a),
		.pulse_a(en_a),
		.clk_b(clk_b),
		.pulse_b(update_b)
	);

	PulseSynchronizer sync_ack(
		.clk_a(clk_b),
		.pulse_a(update_b),
		.clk_b(clk_a),
		.pulse_b(ack_a)
	);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Data plane

	always_ff @(posedge clk_b) begin
		updated_b	<= update_b;
		if(update_b)
			reg_b	<= reg_a;
	end

endmodule
