`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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

`include "EthernetBus.svh"
`include "GmiiBus.svh"

/**
	@brief Performance counters for an EthernetBus
 */
module EthernetPerformanceCounters #(
	parameter PIPELINED_INCREMENT = 0
)(
	input wire							rx_clk,
	input wire EthernetRxBus			rx_bus,

	input wire							tx_clk,
	input wire EthernetTxBus			tx_bus,

	input wire							rst_rx,
	input wire							rst_tx,

	output EthernetMacPerformanceData	counters
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side

	PerformanceCounter count_rx_frames(
		.clk(rx_clk), .rst(rst_rx), .en(rx_bus.commit), .delta(1), .count(counters.rx_frames));

	PerformanceCounter count_rx_crc_err(
		.clk(rx_clk), .rst(rst_rx), .en(rx_bus.drop), .delta(1), .count(counters.rx_crc_err));

	PerformanceCounter count_rx_bytes(
		.clk(rx_clk), .rst(rst_rx), .en(rx_bus.data_valid), .delta(rx_bus.bytes_valid), .count(counters.rx_bytes));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side

	PerformanceCounter count_tx_frames(
		.clk(tx_clk), .rst(rst_tx), .en(tx_bus.start), .delta(1), .count(counters.tx_frames));

	PerformanceCounter count_tx_bytes(
		.clk(tx_clk), .rst(rst_tx), .en(tx_bus.data_valid), .delta(tx_bus.bytes_valid), .count(counters.tx_bytes));


endmodule
