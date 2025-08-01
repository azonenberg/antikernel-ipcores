`timescale 1ns / 1ps
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

import EthernetBus::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief Convenience wrapper around RGMIIToGMIIBridge and AXIS_TriSpeedEthernetMAC
 */
module AXIS_RGMIIMACWrapper #(
	parameter PHY_INTERNAL_DELAY_RX = 1,
	parameter CLK_BUF_TYPE = "GLOBAL"
)(

	//Clocking
	input wire					clk_125mhz,
	input wire					clk_250mhz,

	//RGMII signals to off-chip device
	input wire					rgmii_rxc,
	input wire[3:0]				rgmii_rxd,
	input wire					rgmii_rx_ctl,

	output wire					rgmii_txc,
	output wire[3:0]			rgmii_txd,
	output wire					rgmii_tx_ctl,

	//MAC signals to TCP/IP stack
	AXIStream.receiver			axi_tx,
	AXIStream.transmitter		axi_rx,

	//axi_rx aclk domain
	output wire					link_up,
	output lspeed_t				link_speed

	//TODO: performance counters
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Bridge to GMII

	wire		gmii_rxc;
	GmiiBus		gmii_rx_bus;
	GmiiBus		gmii_tx_bus;

	assign		mac_rx_clk	= gmii_rxc;

	RGMIIToGMIIBridge #(
		.PHY_INTERNAL_DELAY_RX(PHY_INTERNAL_DELAY_RX),
		.CLK_BUF_TYPE(CLK_BUF_TYPE)
	) rgmii_bridge(
		.rgmii_rxc(rgmii_rxc),
		.rgmii_rxd(rgmii_rxd),
		.rgmii_rx_ctl(rgmii_rx_ctl),

		.rgmii_txc(rgmii_txc),
		.rgmii_txd(rgmii_txd),
		.rgmii_tx_ctl(rgmii_tx_ctl),

		.gmii_rxc(gmii_rxc),
		.gmii_rx_bus(gmii_rx_bus),

		.gmii_txc(clk_125mhz),
		.clk_250mhz(clk_250mhz),
		.gmii_tx_bus(gmii_tx_bus),

		.link_up(link_up),
		.link_speed(link_speed)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual MAC

	AXIS_TriSpeedEthernetMAC mac(
		.gmii_rx_clk(gmii_rxc),
		.gmii_rx_bus(gmii_rx_bus),

		.gmii_tx_clk(clk_125mhz),
		.gmii_tx_bus(gmii_tx_bus),

		.link_up(link_up),
		.link_speed(link_speed),

		.axi_rx(axi_rx),
		.axi_tx(axi_tx)
		);


endmodule
