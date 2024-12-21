`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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

`include "APBTypes.sv"

/**
	@brief An APBv5 clock domain crossing

	This module essentially functions as an APBRegisterSlice but with separate clocks on each port.

	There is no requirement for any phase or frequency relationship between the clocks.
 */
module APB_CDC(

	(* ASYNC_REG = "TRUE" *)
	APB.completer	upstream,

	input wire		downstream_pclk,

	(* ASYNC_REG = "TRUE" *)
	APB.requester	downstream
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock path

	assign downstream.pclk = downstream_pclk;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset path

	ResetSynchronizer sync_rst(
		.rst_in_n(upstream.preset_n),
		.clk(downstream.pclk),
		.rst_out_n(downstream.preset_n));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control handshaking

	wire	dupdate;
	logic	ready = 0;

	HandshakeSynchronizer sync_ctl(
		.clk_a(upstream.pclk),
		.en_a(upstream.penable && upstream.psel && !upstream.pready),
		.ack_a(upstream.pready),
		.busy_a(),

		.clk_b(downstream.pclk),
		.en_b(dupdate),
		.ack_b(ready),
		.busy_b()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Upstream -> downstream control path

	logic	dupdate_ff	= 0;
	logic	done		= 0;
	always_ff @(posedge downstream_pclk) begin

		dupdate_ff				<= dupdate;

		if(dupdate_ff)
			downstream.psel		<= 1;
		if(downstream.pready || done) begin
			downstream.psel		<= 0;
			downstream.penable	<= 0;
		end

		if(dupdate) begin
			done				<= 0;
			downstream.penable	<= 1;

			//forward data downstream
			downstream.paddr	<=	upstream.paddr;
			downstream.pwrite	<=	upstream.pwrite;
			downstream.pwdata	<=	upstream.pwdata;
			downstream.pprot	<=	upstream.pprot;
			downstream.pstrb	<=	upstream.pstrb;
			downstream.pwakeup	<=	upstream.pwakeup;
			downstream.pauser	<=	upstream.pauser;
			downstream.pwuser	<=	upstream.pwuser;
		end

		if(downstream.pready)
			done				<= 1;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Downstream -> upstream path

	always_ff @(posedge downstream_pclk) begin

		ready		<= 0;

		if(downstream.pready && !ready)
			ready	<= 1;

		if(downstream.pready) begin
			upstream.prdata		<= downstream.prdata;
			upstream.pslverr	<= downstream.pslverr;
			upstream.pruser		<= downstream.pruser;
			upstream.pbuser		<= downstream.pbuser;
		end

	end

endmodule
