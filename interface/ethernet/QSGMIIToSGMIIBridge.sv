`default_nettype none
`timescale 1ns/1ps
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief QSGMII - 4x SGMII splitter block
 */
module QSGMIIToSGMIIBridge(

	//SERDES interface: 125 MHz, 32 bit data plus control flags
	//(8B/10B coding handled by transceiver)
	input wire			rx_clk,
	input wire			rx_data_valid,
	input wire[3:0]		rx_data_is_ctl,
	input wire[31:0]	rx_data,
	input wire[3:0]		rx_disparity_err,
	input wire[3:0]		rx_symbol_err,

	input wire			tx_clk,
	output logic[3:0]	tx_data_is_ctl	= 0,
	output logic[31:0]	tx_data	= 0,
	output logic[3:0]	tx_force_disparity_negative	= 0,

	//SGMII interfaces (feed to GigBaseXPCS)
	output logic		sgmii_rx_data_valid	= 0,
	output logic[3:0]	sgmii_rx_data_is_ctl = 0,
	output logic[31:0]	sgmii_rx_data = 0,
	output logic[3:0]	sgmii_rx_disparity_err = 0,
	output logic[3:0]	sgmii_rx_symbol_err = 0,

	input wire[3:0]		sgmii_tx_data_is_ctl,
	input wire[31:0]	sgmii_tx_data,
	input wire[3:0]		sgmii_tx_force_disparity_negative,
	output wire[3:0]	sgmii_tx_disparity_negative
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX disparity tracking

	for(genvar g=0; g<4; g=g+1) begin
		Encode8b10b tx_encoder(
			.clk(tx_clk),

			.data_is_ctl(sgmii_tx_data_is_ctl[g]),
			.data(sgmii_tx_data[g*8 +: 8]),
			.force_disparity_negative(sgmii_tx_force_disparity_negative[g]),

			//Don't connect output codeword
			//We're not actually doing fabric encoding, just tracking disparity
			.codeword(),

			.tx_disparity_negative(sgmii_tx_disparity_negative[g])
		);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side splitter logic

	logic[1:0]			baseLane = 0;
	logic[1:0]			isrc = 0;
	always_ff @(posedge rx_clk) begin

		sgmii_rx_data_valid	<= rx_data_valid;

		for(integer i=0; i<4; i=i+1) begin

			//Source lane is rotated by wherever we last saw a K28.1
			isrc = i + baseLane;

			//Pass data along
			sgmii_rx_data_is_ctl[i]		<= rx_data_is_ctl[isrc];
			sgmii_rx_data[i]			<= rx_data[isrc*8 +: 8];
			sgmii_rx_disparity_err[i]	<= rx_disparity_err[isrc];
			sgmii_rx_symbol_err[i]		<= rx_symbol_err[i];

			//Look for K28.1. If found, we are lane zero (and this is actually a K28.5)
			if(rx_data_is_ctl[isrc] && (rx_data[isrc*8 +: 8] == 8'h3c) ) begin
				baseLane <= isrc;
				sgmii_rx_data[isrc*8 +: 8] <= 8'hbc;
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side mux logic

	always_ff @(posedge tx_clk) begin

		tx_data_is_ctl				<= sgmii_tx_data_is_ctl;
		tx_data						<= sgmii_tx_data;
		tx_force_disparity_negative	<= sgmii_tx_force_disparity_negative;

		//Convert K28.5 in lane 0 to K28.1
		if(tx_data_is_ctl[0] && (tx_data[7:0] == 8'hbc) )
			tx_data[7:0]			<= 8'h3c;

	end

endmodule

