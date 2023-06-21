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

/**
	@file
	@author Andrew D. Zonenberg
	@brief	Convenience wrapper around QSGMIIToSGMIIBridge, GigBaseXPCS, and TriSpeedEthernetMAC
 */
module QSGMIIMACWrapper(

	//SERDES interface: 125 MHz, 32 bit data plus control flags
	//(8B/10B coding handled by transceiver)
	input wire									rx_clk,
	input wire									rx_data_valid,
	input wire[3:0]								rx_data_is_ctl,
	input wire[31:0]							rx_data,
	input wire[3:0]								rx_disparity_err,
	input wire[3:0]								rx_symbol_err,

	input wire									tx_clk,
	output wire[3:0]							tx_data_is_ctl,
	output wire[31:0]							tx_data,
	output wire[3:0]							tx_force_disparity_negative,

	//MAC interfaces (tx_clk domain for *both*)
	output EthernetRxBus[3:0]					mac_rx_bus,
	output wire[3:0]							link_up,
	output lspeed_t[3:0]						link_speed,
	output GigabitMacPerformanceCounters[3:0]	mac_perf,

	input wire EthernetTxBus[3:0]				mac_tx_bus,
	output wire[3:0]							mac_tx_ready
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// QSGMII splitter

	wire		sgmii_rx_data_valid;
	wire[3:0]	sgmii_rx_data_is_ctl;
	wire[31:0]	sgmii_rx_data;
	wire[3:0]	sgmii_rx_disparity_err;
	wire[3:0]	sgmii_rx_symbol_err;

	wire[3:0]	sgmii_tx_data_is_ctl;
	wire[31:0]	sgmii_tx_data;
	wire[3:0]	sgmii_tx_force_disparity_negative;
	wire[3:0]	sgmii_tx_disparity_negative;

	QSGMIIToSGMIIBridge quadsplit(
		.rx_clk(rx_clk),
		.rx_data_valid(rx_data_valid),
		.rx_data_is_ctl(rx_data_is_ctl),
		.rx_data(rx_data),
		.rx_disparity_err(rx_disparity_err),
		.rx_symbol_err(rx_symbol_err),

		.tx_clk(tx_clk),
		.tx_data_is_ctl(tx_data_is_ctl),
		.tx_data(tx_data),
		.tx_force_disparity_negative(tx_force_disparity_negative),

		.sgmii_rx_data_valid(sgmii_rx_data_valid),
		.sgmii_rx_data_is_ctl(sgmii_rx_data_is_ctl),
		.sgmii_rx_data(sgmii_rx_data),
		.sgmii_rx_disparity_err(sgmii_rx_disparity_err),
		.sgmii_rx_symbol_err(sgmii_rx_symbol_err),

		.sgmii_tx_data_is_ctl(tx_data_is_ctl),
		.sgmii_tx_data(tx_data),
		.sgmii_tx_force_disparity_negative(sgmii_tx_force_disparity_negative),
		.sgmii_tx_disparity_negative(sgmii_tx_disparity_negative)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Per channel stuff

	for(genvar g=0; g<4; g=g+1) begin : lanes

		GmiiBus gmii_rx_bus;
		GmiiBus gmii_tx_bus;

		//Line coding
		GigBaseXPCS pcs(
			.clk_125mhz(tx_clk),
			.sgmii_mode(1'b1),

			.rx_clk(rx_clk),
			.rx_data_valid(sgmii_rx_data_valid),
			.rx_data_is_ctl(sgmii_rx_data_is_ctl[g]),
			.rx_data(sgmii_rx_data[g*8 +: 8]),
			.rx_disparity_err(sgmii_rx_disparity_err[g]),
			.rx_symbol_err(sgmii_rx_symbol_err[g]),

			.link_up(link_up[g]),
			.link_speed(link_speed[g]),

			.gmii_rx_bus(gmii_rx_bus),
			.gmii_tx_bus(gmii_tx_bus),

			.tx_clk(),
			.tx_data_is_ctl(sgmii_tx_data_is_ctl[g]),
			.tx_data(sgmii_tx_data[g*8 +: 8]),
			.tx_force_disparity_negative(sgmii_tx_force_disparity_negative[g]),
			.tx_disparity_negative(sgmii_tx_disparity_negative[g])
		);

		//MAC
		//(note that we cross to tx_clk domain in the PCS)
		TriSpeedEthernetMAC mac(
			.gmii_rx_clk(tx_clk),
			.gmii_rx_bus(gmii_rx_bus),

			.gmii_tx_clk(tx_clk),
			.gmii_tx_bus(gmii_tx_bus),

			.link_up(link_up[g]),
			.link_speed(link_speed[g]),

			.rx_bus(mac_rx_bus[g]),
			.tx_bus(mac_tx_bus[g]),

			.tx_ready(mac_tx_ready[g]),
			.perf(mac_perf[g])
		);

	end

endmodule
