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
	@brief Wrapper around PulseSynchronizer for moving a multi-bit value between clock domains

	Often used for synchronizing a register from a management/JTAG domain to a SoC internal domain.

	If IN_REG is set to 1, the input is registered prior to the clock domain crossing.

	If IN_REG is set to 0, the A domain logic must not change reg_a's value between assertion of en_a and receipt of
	ack_a.
 */
module RegisterSynchronizer #(
	parameter WIDTH 	= 16,
	parameter INIT		= 0,
	parameter IN_REG	= 1
) (
	input wire				clk_a,
	input wire				en_a,
	output wire				ack_a,
	input wire[WIDTH-1:0]	reg_a,

	input wire				clk_b,
	output logic			updated_b 	= 0,
	input wire				reset_b,

	(* ASYNC_REG = "TRUE" *)
	output logic[WIDTH-1:0]	reg_b		= INIT
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control plane

	wire				update_b;

	//Newer vivado seems to not like having an always_ff and always_comb driving the same block
	//even if mutually exclusive at synthesis time, it causes problems in simulation
	//so we have to jump through some hoops here...

	logic[WIDTH-1:0]	reg_a_muxed;
	logic				en_a_muxed;

	//Registered path
	logic[WIDTH-1:0]	reg_a_ff = INIT;
	always_ff @(posedge clk_a) begin
		if(en_a)
			reg_a_ff	<= reg_a;
	end

	//Mux it
	always_comb begin
		if(IN_REG)
			reg_a_muxed	= reg_a_ff;
		else
			reg_a_muxed	= reg_a;
	end

	PulseSynchronizer #(
		.SYNC_IN_REG(IN_REG)
	) sync_en (
		.clk_a(clk_a),
		.pulse_a(en_a),
		.clk_b(clk_b),
		.pulse_b(update_b)
	);

	PulseSynchronizer #(
		.SYNC_IN_REG(IN_REG)
	) sync_ack (
		.clk_a(clk_b),
		.pulse_a(update_b),
		.clk_b(clk_a),
		.pulse_b(ack_a)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Data plane

	//Add one extra full cycle on the output side of the sync
	//to make sure we get a clean capture (i.e. data path is always min of one cycle longer than control path)
	logic	update_b_ff	= 0;

	always_ff @(posedge clk_b or posedge reset_b) begin

		if(reset_b) begin
			updated_b	<= 0;
			reg_b		<= INIT;
			update_b_ff	<= 0;
		end

		else begin
			update_b_ff	<= update_b;

			updated_b	<= update_b_ff;
			if(update_b_ff)
				reg_b	<= reg_a_muxed;
		end

	end

endmodule
