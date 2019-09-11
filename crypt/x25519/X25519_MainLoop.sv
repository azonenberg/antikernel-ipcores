`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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
	@brief X25519 multiplication

	Derived from mainloop() in NaCl crypto_scalarmult/curve25519/ref/smult.c (public domain)
 */
module X25519_MainLoop(
	input wire			clk,
	input wire			en,
	input wire[255:0]	work_in,
	input wire[255:0]	e,
	output logic		out_valid	= 0,
	output logic[511:0]	work_out	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loop contents

	logic			iter_en		= 0;
	logic			b			= 0;

	logic[511:0]	xzm1		= 0;
	logic[511:0]	xzm			= 0;

	wire			iter_valid;
	wire[511:0]		iter_xzm_out;
	wire[511:0]		iter_xzm1_out;

	X25519_MainLoopIteration iter(
		.clk(clk),
		.en(iter_en),
		.xzm1_in(xzm1),
		.xzm_in(xzm),
		.b(b),
		.work_low({8'h0, work_in}),
		.out_valid(iter_valid),
		.xzm_out(iter_xzm_out),
		.xzm1_out(iter_xzm1_out)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sequencing

	logic[7:0] round = 0;

	enum logic[2:0]
	{
		STATE_IDLE,
		STATE_START,
		STATE_WAIT,
		STATE_DONE
	} state = STATE_IDLE;

	always_ff @(posedge clk) begin

		iter_en		<= 0;
		out_valid	<= 0;

		case(state)

			STATE_IDLE: begin

				//When starting a new multiply, go from the highest bit
				if(en) begin
					iter_en			<= 1;
					round			<= 254;
					b				<= e[254];
					xzm1[511:256]	<= 256'h1;
					xzm1[255:0]		<= work_in;
					xzm[511:0]		<= 512'h1;
					state			<= STATE_WAIT;
				end

			end	//end STATE_IDLE

			STATE_START: begin
				b					<= e[round];
				iter_en				<= 1;
				state				<= STATE_WAIT;
			end

			STATE_WAIT: begin
				if(iter_valid) begin
					xzm				<= iter_xzm_out;
					xzm1			<= iter_xzm1_out;
					round			<= round - 1;

					if(round == 0)
						state		<= STATE_DONE;
					else
						state		<= STATE_START;

				end
			end	//end STATE_WAIT

			STATE_DONE: begin
				out_valid	<= 1;
				work_out	<= iter_xzm_out;
				state		<= STATE_IDLE;
			end	//end STATE_DONE

		endcase

	end

endmodule
