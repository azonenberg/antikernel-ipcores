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
	@brief Three-stage flipflop-based synchronizer
 */
module ThreeStageSynchronizer #(
	parameter INIT		= 0,
	parameter IN_REG	= 1
)(
	input wire clk_in,
	input wire din,
	input wire clk_out,

	(* ASYNC_REG = "TRUE" *)
	output logic dout	`ifndef EFINIX = INIT `endif
    );

    `ifdef EFINIX
    initial dout = INIT;
    `endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input stage

	logic dout0;

	//First stage: FF in the transmitting domain
	//TODO: Figure out why vivado doesn't let me initialize dout0!
	if(IN_REG) begin
		always_ff @(posedge clk_in) begin
			dout0	<= din;
		end
	end

	//Assume first stage is registered already in the sending module
	else begin
		always_comb
			dout0	= din;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Two stages in the receiving clock domain
	(* ASYNC_REG = "TRUE" *) logic dout1;
	always_ff @(posedge clk_out) begin
		dout1	<= dout0;
		dout	<= dout1;
	end

endmodule
