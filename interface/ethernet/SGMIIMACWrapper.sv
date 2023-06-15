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

`include "GmiiBus.svh"
`include "EthernetBus.svh"
`include "SGMIIToGMIIBridge.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief Convenience wrapper around SGMIIToGMIIBridge and TriSpeedEthernetMAC
 */
module SGMIIMACWrapper #(
	parameter RX_INVERT = 0,
	parameter TX_INVERT = 0
)(
	//Main clocks
	input wire								clk_125mhz,
	input wire								clk_312p5mhz,

	//Oversampling clocks for receiver
	input wire								clk_625mhz_0,
	input wire								clk_625mhz_90,

	input wire								sgmii_rx_data_p,
	input wire								sgmii_rx_data_n,

	output wire								sgmii_tx_data_p,
	output wire								sgmii_tx_data_n,

	//MAC signals to TCP/IP stack
	output wire								mac_rx_clk,		//echo of clk_125mhz
	output EthernetRxBus					mac_rx_bus,

	input wire EthernetTxBus				mac_tx_bus,
	output wire 							mac_tx_ready,

	//clk_125mhz domain
	output wire								link_up,
	output lspeed_t							link_speed,

	input wire								rst_stat,
	output SGMIIPerformanceCounters			sgmii_perf,
	output GigabitMacPerformanceCounters	mac_perf,

	output wire								rx_error
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Bridge to GMII

	GmiiBus		gmii_rx_bus;
	GmiiBus		gmii_tx_bus;

	SGMIIToGMIIBridge #(
		.RX_INVERT(RX_INVERT),
		.TX_INVERT(TX_INVERT)
	) bridge (
		.clk_125mhz(clk_125mhz),
		.clk_312p5mhz(clk_312p5mhz),

		.clk_625mhz_0(clk_625mhz_0),
		.clk_625mhz_90(clk_625mhz_90),

		.sgmii_rx_data_p(sgmii_rx_data_p),
		.sgmii_rx_data_n(sgmii_rx_data_n),

		.sgmii_tx_data_p(sgmii_tx_data_p),
		.sgmii_tx_data_n(sgmii_tx_data_n),

		.gmii_rx_bus(gmii_rx_bus),
		.gmii_tx_bus(gmii_tx_bus),

		.link_up(link_up),
		.link_speed(link_speed),

		.rst_stat(rst_stat),
		.perf(sgmii_perf),

		.rx_error(rx_error)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual MAC

	TriSpeedEthernetMAC mac(
		.gmii_rx_clk(clk_125mhz),
		.gmii_rx_bus(gmii_rx_bus),

		.gmii_tx_clk(clk_125mhz),
		.gmii_tx_bus(gmii_tx_bus),

		.link_up(link_up),
		.link_speed(link_speed),

		.rx_bus(mac_rx_bus),
		.tx_bus(mac_tx_bus),
		.tx_ready(mac_tx_ready),

		.perf(mac_perf)
		);


endmodule
