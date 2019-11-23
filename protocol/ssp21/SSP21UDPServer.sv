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
	// HMAC-SHA256 crypto block

	StreamingHMACSHA256 hmac(
		.clk(clk)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// X25519 crypto block

	X25519_ScalarMult smult(
		.clk(clk)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main RX state machine

	enum logic[3:0]
	{
		RX_STATE_IDLE			= 4'h0,
		RX_STATE_DROP			= 4'h1,
		RX_STATE_FIRST			= 4'h2,
		RX_STATE_HANDSHAKE_1	= 4'h3,
		RX_STATE_HANDSHAKE_2	= 4'h4,
		RX_STATE_HANDSHAKE_3	= 4'h5,
		RX_STATE_HANDSHAKE_4	= 4'h6
	} rx_state = RX_STATE_IDLE;

	//Type of an incoming packet
	ssp21_function_t				rx_function;

	//Request-Handshake-Begin
	logic[15:0]						rx_version;
	ssp21_handshake_ephemeral_mode	rx_ephemeral_mode;
	ssp21_handshake_hash			rx_hash;
	ssp21_handshake_kdf				rx_kdf;
	ssp21_nonce_mode				rx_nonce_mode;
	ssp21_session_crypto_mode		rx_crypto_mode;
	logic[15:0]						rx_max_nonce;
	logic[31:0]						rx_max_session_duration;
	ssp21_handshakemode_t			rx_handshake_mode;
	logic[255:0]					rx_handshake_nonce;

	logic[7:0]						rx_wcount;

	always_ff @(posedge clk) begin

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for a new packet to show up
			RX_STATE_IDLE: begin

				//Ready for first byte of data
				if(udp_rx_bus.start)
					rx_state	<= RX_STATE_FIRST;

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read first word of the packet and decide how to handle it

			RX_STATE_FIRST: begin
				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4)
						rx_state	<= RX_STATE_DROP;

					//Parse the first word
					else begin
						rx_function	<= ssp21_function_t'(udp_rx_bus.data[31:24]);

						//Parse header fields depending on function
						case(udp_rx_bus.data[31:24])

							//Function plus start of crypto spec (version, ephemeral mode)
							REQUEST_HANDSHAKE_BEGIN: begin
								rx_version			<= udp_rx_bus.data[23:8];
								rx_ephemeral_mode	<= ssp21_handshake_ephemeral_mode'(udp_rx_bus.data[7:0]);
								rx_state			<= RX_STATE_HANDSHAKE_1;
							end	//end REQUEST_HANDSHAKE_BEGIN

						endcase

					end
				end
			end	//end RX_STATE_FIRST

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Drop corrupted/damaged frames

			RX_STATE_DROP: begin
				//do nothing until the frame ends
			end	//end RX_STATE_DROP

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Parse a Request-Handshake-Begin message

			//Rest of crypto spec (hash, KDF, nonce mode, crypto mode)
			RX_STATE_HANDSHAKE_1: begin
				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4)
						rx_state	<= RX_STATE_DROP;

					//Nope, good data
					else begin
						rx_hash			<= ssp21_handshake_hash'(udp_rx_bus.data[31:24]);
						rx_kdf			<= ssp21_handshake_kdf'(udp_rx_bus.data[23:16]);
						rx_nonce_mode	<= ssp21_nonce_mode'(udp_rx_bus.data[15:8]);
						rx_crypto_mode	<= ssp21_session_crypto_mode'(udp_rx_bus.data[7:0]);

						rx_state		<= RX_STATE_HANDSHAKE_2;
					end

				end
			end	//end RX_STATE_HANDSHAKE_1

			//First half of constraints (max_nonce, start of max_session_duration)
			RX_STATE_HANDSHAKE_2: begin

				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4)
						rx_state	<= RX_STATE_DROP;

					//Nope, good data
					else begin
						rx_max_nonce					<= udp_rx_bus.data[31:16];
						rx_max_session_duration[31:16]	<= udp_rx_bus.data[15:0];

						rx_state						<= RX_STATE_HANDSHAKE_3;
					end

				end

			end	//end RX_STATE_HANDSHAKE_2

			//Second half of max session duration, handshake mode, first byte of ephemeral data
			RX_STATE_HANDSHAKE_3: begin
				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4)
						rx_state	<= RX_STATE_DROP;

					//Nope, good data
					else begin
						rx_max_session_duration[15:0]		<= udp_rx_bus.data[31:16];
						rx_handshake_mode					<= ssp21_handshakemode_t'(udp_rx_bus.data[15:8]);

						//Read ephemeral data
						case(rx_ephemeral_mode)
							NONCE: begin
								rx_handshake_nonce[255:248] <= udp_rx_bus.data[7:0];
							end
						endcase

						rx_state							<= RX_STATE_HANDSHAKE_4;
						rx_wcount							<= 1;
					end

				end
			end	//end RX_STATE_HANDSHAKE_3

			//Rest of ephemeral data
			RX_STATE_HANDSHAKE_4: begin
				rx_wcount	<= rx_wcount + 1;
			end	//end RX_STATE_HANDSHAKE_4

		endcase

		//Go back to idle after the packet is done
		if(udp_rx_bus.commit || udp_rx_bus.drop)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA
	/*
	ila_0 ila(
		.clk(clk),
		.probe0(rx_state),
		.probe1(udp_rx_bus),
		.probe2(rx_function),
		.probe3(rx_version),
		.probe4(rx_ephemeral_mode),
		.probe5(rx_hash),
		.probe6(rx_kdf),
		.probe7(rx_nonce_mode),
		.probe8(rx_crypto_mode),
		.probe9(rx_max_nonce),
		.probe10(rx_max_session_duration),
		.probe11(rx_handshake_mode),

		.probe12(rx_wcount),
		.probe13(rx_handshake_nonce)
	);*/

endmodule
