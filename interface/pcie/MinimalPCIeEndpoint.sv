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
	@brief Bare-bones PCIe gen1/2 x1 endpoint

	Supports just enough of the spec to link up with a DW_pcie and allow bare metal code to read and write

	Input: two 8b10b symbols
 */
module MinimalPCIeEndpoint(
	input wire			rst_n,

	input wire			tx_clk,
	output wire[15:0]	tx_data,
	output wire[1:0]	tx_charisk,

	input wire			rx_clk,
	input wire[15:0]	rx_data,
	input wire[1:0]		rx_charisk,
	input wire[1:0]		rx_disperr,
	input wire[1:0]		rx_symbolerr
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize reset into PHY clock domains

	wire	rst_rx_n;
	wire	rst_tx_n;

	ResetSynchronizer sync_rst_rx( .rst_in_n(rst_n), .clk(rx_clk), .rst_out_n(rst_rx_n));
	ResetSynchronizer sync_rst_tx( .rst_in_n(rst_n), .clk(tx_clk), .rst_out_n(rst_tx_n));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Shift the incoming data into the TX clock domain (the entire protocol stack runs in this clock)

	logic[1:0] rx_err;
	always_comb begin
		rx_err = rx_disperr | rx_symbolerr;
	end

	wire[1:0]	rx_charisk_cdc;
	wire[1:0]	rx_err_cdc;
	wire[15:0]	rx_data_cdc;

	wire	rd_empty;
	CrossClockFifo #(
		.WIDTH(20),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) rx_cdc (
		.wr_clk(rx_clk),
		.wr_en(1),
		.wr_data({rx_charisk, rx_err, rx_data}),
		.wr_size(),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(!rst_rx_n),

		.rd_clk(tx_clk),
		.rd_en(!rd_empty),
		.rd_data({rx_charisk_cdc, rx_err_cdc, rx_data_cdc}),
		.rd_size(),
		.rd_empty(rd_empty),
		.rd_underflow(),
		.rd_reset(!rst_tx_n)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Post-training link and lane IDs

	wire[4:0]	link_id;
	wire[4:0]	lane_id;
	wire		link_up;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output mux

	wire[15:0]	tx_train_data;
	wire[1:0]	tx_train_charisk;
	wire		tx_train_skip_ack;

	wire[15:0]	tx_ll_data;
	wire[1:0]	tx_ll_charisk;
	wire		tx_ll_skip_ack;

	wire		tx_skip_req;
	wire		tx_skip_done;

	PCIeOutputMux outmux(
		.clk(tx_clk),
		.rst_n(rst_tx_n),

		.tx_data(tx_data),
		.tx_charisk(tx_charisk),

		.tx_train_data(tx_train_data),
		.tx_train_charisk(tx_train_charisk),
		.tx_train_skip_ack(tx_train_skip_ack),

		.tx_ll_data(tx_ll_data),
		.tx_ll_charisk(tx_ll_charisk),
		.tx_ll_skip_ack(tx_ll_skip_ack),

		.tx_link_up(link_up),

		.tx_skip_req(tx_skip_req),
		.tx_skip_done(tx_skip_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Link training and status state machine

	PCIeLTSSM ltssm(
		.clk(tx_clk),
		.rst_n(rst_tx_n),

		.rx_data(rx_data_cdc),
		.rx_charisk(rx_charisk_cdc),
		.rx_err(rx_err_cdc),

		.tx_data(tx_train_data),
		.tx_charisk(tx_train_charisk),
		.tx_skip_ack(tx_train_skip_ack),
		.tx_skip_req(tx_skip_req),
		.tx_skip_done(tx_skip_done),

		.link_id(link_id),
		.lane_id(lane_id),
		.link_up(link_up)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Link layer

	wire	dl_link_up;

	PCIeDataLinkLayer linklayer(
		.clk(tx_clk),
		.rst_n(rst_tx_n),

		.link_up(link_up),

		.rx_data(rx_data_cdc),
		.rx_charisk(rx_charisk_cdc),
		.rx_err(rx_err_cdc),

		.tx_data(tx_ll_data),
		.tx_charisk(tx_ll_charisk),

		.tx_skip_req(tx_skip_req),
		.tx_skip_ack(tx_ll_skip_ack),
		.tx_skip_done(tx_skip_done),

		.dl_link_up(dl_link_up)
	);

endmodule
