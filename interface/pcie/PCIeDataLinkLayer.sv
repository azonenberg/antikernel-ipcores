`timescale 1ns/1ps
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
	@brief PCI Express data link layer
 */
module PCIeDataLinkLayer(
	input wire			clk,
	input wire			rst_n,

	input wire[15:0]	rx_data,
	input wire[1:0]		rx_charisk,
	input wire[1:0]		rx_err,

	output logic[15:0]	tx_data			= 0,
	output logic[1:0]	tx_charisk		= 0,

	input wire			tx_skip_req,
	output logic		tx_skip_ack		= 0,
	input wire			tx_skip_done

	//todo: data link specific ports
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main logic

	logic	skip_pending	= 0;

	always_ff @(posedge clk) begin

		tx_skip_ack		<= 0;

		//If we have a pending skip request, ack it
		if(tx_skip_req) begin
			tx_skip_ack			<= 1;
			skip_pending		<= 1;
		end

		//wait for skip to finish
		if(skip_pending) begin
			if(tx_skip_done) begin
				skip_pending	<= 0;
			end
		end

		//Normal mode
		else begin
			tx_data				<= 0;
			tx_charisk			<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	/*
	ila_0 ila(
		.clk(tx_clk),

		.probe0(rx_data_cdc),
		.probe1(rx_charisk_cdc),
		.probe2(rx_err_cdc),

		.probe3(rx_ts1_valid),
		.probe4(rx_ts2_valid),
		.probe5(rx_ts_link_valid),
		.probe6(rx_ts_link),
		.probe7(rx_ts_lane_valid),
		.probe8(rx_ts_lane),
		.probe9(rx_ts_num_fts),
		.probe10(rx_ts_5g_supported),
		.probe11(tx_data),
		.probe12(tx_charisk),
		.probe13(tsgen.skip_count),
		.probe14(ts_type_is_ts2),
		.probe15(tx_ts_link_valid),
		.probe16(tx_ts_link),
		.probe17(tx_ts_lane_valid),
		.probe18(tx_ts_lane),
		.probe19(tsSentCount),
		.probe20(tsRecvCount),
		.probe21(ltssm_state),
		.probe22(tx_train_data),
		.probe23(tx_train_charisk),
		.probe24(tx_train_skip_ack),
		.probe25(tx_skip_req),
		.probe26(tx_skip_done),
		.probe27(link_up),
		.probe28(1'b0)
	);
	*/

endmodule
