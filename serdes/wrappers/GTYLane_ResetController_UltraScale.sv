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
	@brief Reset controller for a GTYE4_CHANNEL
 */
module GTYLane_ResetController_UltraScale(

	//Clock inputs
	input wire	clk_system_in,		//system clock for power-on reset (must not stop when GTY clocks do)

	//Reset inputs
	input wire	rst_async_tx_in,
	input wire	rst_async_rx_in,

	//Synchronized resets out
	output wire	rst_gt_tx_out,
	output wire	rst_gt_rx_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Hold lane in reset for a short time after configuration

	logic[3:0] rst_init_count = 1;
	logic rst_init_ff = 1;

	always_ff @(posedge clk_system_in) begin
		if(rst_init_count)
			rst_init_count <= rst_init_count + 1;
		rst_init_ff	<= (rst_init_count > 0);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Combine all reset sources for TX

	logic	rst_tx_ored;
	always_comb begin
		if(rst_init_ff || rst_async_tx_in)
			rst_tx_ored = 1;
		else
			rst_tx_ored = 0;
	end

	assign rst_gt_tx_out = rst_tx_ored;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Combine all reset sources for RX

	logic	rst_rx_ored;
	always_comb begin
		if(rst_init_ff || rst_async_rx_in)
			rst_rx_ored = 1;
		else
			rst_rx_ored = 0;
	end

	assign rst_gt_rx_out = rst_rx_ored;

endmodule
