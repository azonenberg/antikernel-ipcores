`timescale 1ns / 1ps
`default_nettype none
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

`include "SSP21.svh"
`include "UDPv4Bus.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief Responder for SSP-21 protocol, implemented as a UDP socket server
 */
module SSP21UDPServer(
	input wire							clk,

	input wire ssp21_handshakemode_t	crypto_mode,	//Crypto mode
	input wire[255:0]					crypto_psk,		//Pre-shared key to use in SHARED_SECRET mode

	input wire UDPv4RxBus				udp_rx_bus,
	output UDPv4TxBus					udp_tx_bus
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main RX state machine

	enum logic[3:0]
	{
		RX_STATE_IDLE	= 4'h0,
		RX_STATE_FIRST	= 4'h1
	} rx_state = RX_STATE_IDLE;

	always_ff @(posedge clk) begin

		case(rx_state)

			//Wait for a new packet to show up
			RX_STATE_IDLE: begin

				//Ready for first byte of data
				if(udp_rx_bus.start)
					rx_state	<= RX_STATE_FIRST;

			end	//end RX_STATE_IDLE

			//
			RX_STATE_FIRST: begin

			end	//end RX_STATE_FIRST

		endcase

		//Go back to idle after the packet is done
		if(udp_rx_bus.commit || udp_rx_bus.drop)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	ila_0 ila(
		.clk(clk),
		.probe0(rx_state),
		.probe1(udp_rx_bus)
	);

endmodule
