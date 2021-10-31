`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2021 Andrew D. Zonenberg                                                                          *
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief Converter from EthernetBus to RMII (at fixed 100 Mbps)
 */
module RMIIToEthernetBusBridge(

	//Clocks
	input wire					clk_50mhz,
	input wire					clk_125mhz,
	input wire					gmii_rxc,

	//Host side data bus
	input wire EthernetRxBus	host_rx_bus,
	output EthernetTxBus		host_tx_bus,
	input wire					host_tx_ready,

	//RMII bus
	output wire					rmii_refclk,
	output wire					rmii_rx_en,
	output wire[1:0]			rmii_rxd,
	input wire					rmii_tx_en,
	input wire[1:0]				rmii_txd
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RGMII -> RMII conversion

	wire			rmii_tx_ready;
	EthernetTxBus	rmii_tx_bus;

	EthernetCrossoverClockCrossing_spaced gtoc(
		.rx_clk(gmii_rxc),
		.rx_bus(host_rx_bus),

		.tx_clk(clk_50mhz),
		.tx_ready(rmii_tx_ready),
		.tx_bus(rmii_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RMII -> RGMII conversion

	EthernetRxBus	rmii_rx_bus;

	EthernetCrossoverClockCrossing_x8 ctog(
		.rx_clk(clk_50mhz),
		.rx_bus(rmii_rx_bus),

		.tx_clk(clk_125mhz),
		.tx_ready(host_tx_ready),
		.tx_bus(host_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The MAC

	/*
		Invert the transmitted reference clock since we drive data on the positive edge

		Inversion leads to data toggling on the falling edge, providing close to 10ns of setup and hold margin for
		the MAC on the MCU. RMII v1.2 spec requires 4ns setup / 2ns hold which should be trivial.
	 */
	assign rmii_refclk = !clk_50mhz;

	SimpleRMIIMAC mac(
		.clk_50mhz(clk_50mhz),

		.mac_tx_bus(rmii_tx_bus),
		.mac_tx_ready(rmii_tx_ready),
		.mac_rx_bus(rmii_rx_bus),

		.rmii_rx_en(rmii_rx_en),
		.rmii_rxd(rmii_rxd),

		.rmii_tx_en(rmii_tx_en),
		.rmii_txd(rmii_txd)
	);

endmodule
