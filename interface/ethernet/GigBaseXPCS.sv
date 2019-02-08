`default_nettype none
`timescale 1ns/1ps

/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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
	@brief	1000base-X / SGMII PCS
 */
module GigBaseXPCS(

	input wire			clk_125mhz,

	//1 = SGMII, 0 = 1000base-X
	input wire			sgmii_mode,

	//RX SERDES interface.
	//Typical usage: 156.25 MHz clock, but average of 125 MHz valid data rate.
	//May also be 125 MHz with rx_data_valid tied high.
	input wire			rx_clk,
	input wire			rx_data_valid,
	input wire			rx_data_is_ctl,
	input wire[7:0]		rx_data,

	//RX status signals
	output logic		link_up		= 0,
	output lspeed_t		link_speed	= LINK_SPEED_1000M,

	//RX GMII interface. Clock is always 125 MHz regardless of link speed.
	output GmiiBus		rx_gmii_bus,

	//TX SERDES interface. 125 MHz.
	output wire			tx_clk,
	output logic		tx_data_is_ctl	= 0,
	output logic[7:0]	tx_data	= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Forward the transmit clock

	assign	tx_clk = clk_125mhz;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX autonegotiation

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX autonegotiation

	//Dummy state machine for now
	logic[3:0] count = 0;
	always_ff @(posedge tx_clk) begin

		count		<= count + 1'h1;

		tx_data_is_ctl	<= 0;

		case(count)

			//K28.5
			0: begin
				tx_data	<= 8'hbc;
				tx_data_is_ctl	<= 1;
			end

			//D21.5
			1: 	tx_data	<= 8'hb5;

			//Status register
			2:	tx_data	<= 0;
			3:	tx_data	<= 0;

			//K28.5
			4: begin
				tx_data	<= 8'hbc;
				tx_data_is_ctl	<= 1;
			end

			//D2.2
			5: 	tx_data	<= 8'h42;

			//Status register
			6:	tx_data	<= 0;
			7:	tx_data	<= 0;

		endcase

	end

endmodule
