`timescale 1ns/1ps
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

module DRPClockDomainShifting(

	//Management clock domain
	input wire			mgmt_clk,

	input wire			mgmt_en,
	input wire			mgmt_wr,
	input wire[8:0]		mgmt_addr,
	input wire[15:0]	mgmt_wdata,
	output wire[15:0]	mgmt_rdata,
	output wire			mgmt_done,

	//DRP clock domain
	input wire			drp_clk,
	output wire			drp_en,
	output wire			drp_we,
	output wire[8:0]	drp_addr,
	output wire[15:0]	drp_di,
	input wire[15:0]	drp_do,
	input wire			drp_rdy
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write path synchronizer

	RegisterSynchronizer #(
		.WIDTH(26),
		.INIT(0),
		.IN_REG(1)
	) wr_sync (
		.clk_a(mgmt_clk),
		.en_a(mgmt_en),
		.ack_a(),
		.reg_a({mgmt_wr, mgmt_addr, mgmt_wdata}),

		.clk_b(drp_clk),
		.updated_b(drp_en),
		.reset_b(1'b0),
		.reg_b({drp_we, drp_addr, drp_di})
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read path synchronizer

	RegisterSynchronizer #(
		.WIDTH(16),
		.INIT(0),
		.IN_REG(1)
	) rd_sync (
		.clk_a(drp_clk),
		.en_a(drp_rdy),
		.ack_a(),
		.reg_a(drp_do),

		.clk_b(mgmt_clk),
		.updated_b(mgmt_done),
		.reset_b(1'b0),
		.reg_b(mgmt_rdata)
	);

endmodule
