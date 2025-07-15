`timescale 1ns / 1ps
`default_nettype none
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
	@brief Pipeline stage for AXI4-Stream with no flow control
 */
module AXIS_PipelineStage(
	AXIStream.receiver			axi_rx,
	AXIStream.transmitter		axi_tx
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Validate buses are the same size

	if(axi_rx.DATA_WIDTH != axi_tx.DATA_WIDTH)
		axi_bus_width_inconsistent();
	if(axi_rx.USER_WIDTH != axi_tx.USER_WIDTH)
		axi_bus_width_inconsistent();
	if(axi_rx.ID_WIDTH != axi_tx.ID_WIDTH)
		axi_bus_width_inconsistent();
	if(axi_rx.DEST_WIDTH != axi_tx.DEST_WIDTH)
		axi_bus_width_inconsistent();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pass-through on control signals

	assign axi_tx.aclk		= axi_rx.aclk;
	assign axi_tx.twakeup	= axi_rx.twakeup;
	assign axi_tx.areset_n	= axi_rx.areset_n;

	//Ignore flow control
	assign axi_rx.tready	= 1;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline register on control signals

	always_ff @(posedge axi_rx.aclk) begin
		axi_tx.tdata	<= axi_rx.tdata;
		axi_tx.tuser	<= axi_rx.tuser;
		axi_tx.tid		<= axi_rx.tid;
		axi_tx.tdest	<= axi_rx.tdest;
		axi_tx.tstrb	<= axi_rx.tstrb;
		axi_tx.tkeep	<= axi_rx.tkeep;
		axi_tx.tlast	<= axi_rx.tlast;
	end

endmodule
