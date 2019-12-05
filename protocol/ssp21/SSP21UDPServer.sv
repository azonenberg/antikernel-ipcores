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

	//Configuration
	input wire ssp21_handshakemode_t	crypto_mode,	//Crypto mode
	input wire[255:0]					crypto_psk,		//Pre-shared key to use in SHARED_SECRET mode

	//Interface to the RNG
	input wire							rng_gen_ready,
	output logic						rng_gen_en	= 0,
	input wire							rng_valid,
	input wire[31:0]					rng_out,

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
		RX_STATE_FIRST			= 4'h1,
		RX_STATE_HANDSHAKE_1	= 4'h2,
		RX_STATE_HANDSHAKE_2	= 4'h3,
		RX_STATE_HANDSHAKE_3	= 4'h4,
		RX_STATE_HANDSHAKE_4	= 4'h5,
		RX_STATE_HANDSHAKE_5	= 4'h6,
		RX_STATE_DROP			= 4'h7
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

	//Communications from RX to TX side
	logic							rx_handshake_begin_en		= 0;	//handshake-begin
	logic							rx_handshake_begin_busy		= 0;
	logic							rx_handshake_begin_done		= 0;

	logic							rx_err_en		= 0;
	logic							rx_err_busy		= 0;
	logic[7:0]						rx_err_id		= 0;
	logic							rx_err_done		= 0;

	logic[7:0]						rx_wcount;

	localparam						OUR_PORT	= 16'd8888;

	always_ff @(posedge clk) begin

		rx_handshake_begin_en	<= 0;
		rx_err_en		<= 0;

		if(rx_handshake_begin_done)
			rx_handshake_begin_busy	<= 0;
		if(rx_err_done)
			rx_err_busy	<= 0;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for a new packet to show up

			RX_STATE_IDLE: begin

				//Ready for first byte of data
				if(udp_rx_bus.start) begin

					//Silently drop if going to the wrong port (anything but 8888 for now)
					if(udp_rx_bus.dst_port != OUR_PORT) begin
					end

					//all good
					else
						rx_state	<= RX_STATE_FIRST;
				end

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read first word of the packet and decide how to handle it

			RX_STATE_FIRST: begin

				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4)  begin
						rx_err_id	<= BAD_MESSAGE_FORMAT;
						rx_state	<= RX_STATE_DROP;
					end

					//Parse the first word
					else begin
						rx_function	<= ssp21_function_t'(udp_rx_bus.data[31:24]);

						//Parse header fields depending on function
						case(udp_rx_bus.data[31:24])

							//Function plus start of crypto spec (version, ephemeral mode)
							REQUEST_HANDSHAKE_BEGIN: begin

								//If it's a handshake-begin and we're still processing the previous handshake-begin,
								//silently drop it.
								if(rx_handshake_begin_busy)
									rx_state			<= RX_STATE_DROP;
								else begin
									rx_version			<= udp_rx_bus.data[23:8];
									rx_ephemeral_mode	<= ssp21_handshake_ephemeral_mode'(udp_rx_bus.data[7:0]);
									rx_state			<= RX_STATE_HANDSHAKE_1;
								end
							end	//end REQUEST_HANDSHAKE_BEGIN

							//Unknown message type (discard with an error)
							default: begin
								rx_err_id	<= BAD_MESSAGE_FORMAT;
								rx_state	<= RX_STATE_DROP;
							end	//unknown

						endcase

					end
				end

				//Abort if the packet ends with no content
				if(udp_rx_bus.drop) begin
					rx_err_id	<= BAD_MESSAGE_FORMAT;
					rx_state	<= RX_STATE_DROP;
				end

			end	//end RX_STATE_FIRST

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Drop corrupted/damaged frames

			RX_STATE_DROP: begin

				//If already sending a drop message, drop silently. Otherwise send the reply
				if(!rx_err_busy) begin
					rx_err_en	<= 1;
					rx_err_busy	<= 1;
				end

				rx_state		<= RX_STATE_IDLE;

			end	//end RX_STATE_DROP

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Parse a Request-Handshake-Begin message

			//Rest of crypto spec (hash, KDF, nonce mode, crypto mode)
			RX_STATE_HANDSHAKE_1: begin

				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4) begin
						rx_err_id		<= BAD_MESSAGE_FORMAT;
						rx_state		<= RX_STATE_DROP;
					end

					//Nope, good data
					else begin
						rx_hash			<= ssp21_handshake_hash'(udp_rx_bus.data[31:24]);
						rx_kdf			<= ssp21_handshake_kdf'(udp_rx_bus.data[23:16]);
						rx_nonce_mode	<= ssp21_nonce_mode'(udp_rx_bus.data[15:8]);
						rx_crypto_mode	<= ssp21_session_crypto_mode'(udp_rx_bus.data[7:0]);

						//Validate configuration
						if(udp_rx_bus.data[31:24] != SHA256) begin
							rx_err_id	<= UNSUPPORTED_HANDSHAKE_HASH;
							rx_state	<= RX_STATE_DROP;
						end
						else if(udp_rx_bus.data[23:16] != HKDF_SHA256) begin
							rx_err_id	<= UNSUPPORTED_HANDSHAKE_KDF;
							rx_state	<= RX_STATE_DROP;
						end
						else if(udp_rx_bus.data[15:8] != INCREMENT_LAST_RX) begin
							rx_err_id	<= UNSUPPORTED_NONCE_MODE;
							rx_state	<= RX_STATE_DROP;
						end
						else
							rx_state	<= RX_STATE_HANDSHAKE_2;
					end

				end

				//Abort if the packet ends
				if(udp_rx_bus.drop) begin
					rx_err_id			<= BAD_MESSAGE_FORMAT;
					rx_state			<= RX_STATE_DROP;
				end

			end	//end RX_STATE_HANDSHAKE_1

			//First half of constraints (max_nonce, start of max_session_duration)
			RX_STATE_HANDSHAKE_2: begin

				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4) begin
						rx_err_id		<= BAD_MESSAGE_FORMAT;
						rx_state		<= RX_STATE_DROP;
					end

					//Nope, good data
					else begin
						rx_max_nonce					<= udp_rx_bus.data[31:16];
						rx_max_session_duration[31:16]	<= udp_rx_bus.data[15:0];

						rx_state						<= RX_STATE_HANDSHAKE_3;
					end

				end

				//Abort if the packet ends
				if(udp_rx_bus.drop) begin
					rx_err_id		<= BAD_MESSAGE_FORMAT;
					rx_state		<= RX_STATE_DROP;
				end

			end	//end RX_STATE_HANDSHAKE_2

			//Second half of max session duration, handshake mode, length of ephemeral data
			//(should always be 32 for both pubkey and nonce modes)
			RX_STATE_HANDSHAKE_3: begin

				if(udp_rx_bus.data_valid) begin

					//Drop the frame if truncated
					if(udp_rx_bus.bytes_valid != 4) begin
						rx_err_id	<= BAD_MESSAGE_FORMAT;
						rx_state	<= RX_STATE_DROP;
					end

					//Nope, good data
					else begin
						rx_max_session_duration[15:0]	<= udp_rx_bus.data[31:16];
						rx_handshake_mode				<= ssp21_handshakemode_t'(udp_rx_bus.data[15:8]);

						rx_state						<= RX_STATE_HANDSHAKE_4;
						rx_wcount						<= 0;

						//For now, we only support SHARED_SECRET
						if(udp_rx_bus.data[15:8] != SHARED_SECRET) begin
							rx_err_id			<= UNSUPPORTED_HANDSHAKE_MODE;
							rx_state			<= RX_STATE_DROP;
						end

						//Read ephemeral data
						case(rx_ephemeral_mode)

							NONCE: begin
								if(udp_rx_bus.data[7:0] != 'd32) begin
									rx_err_id	<= BAD_MESSAGE_FORMAT;
									rx_state	<= RX_STATE_DROP;
								end
							end

							default: begin
								rx_err_id		<= UNSUPPORTED_HANDSHAKE_EPHEMERAL;
								rx_state		<= RX_STATE_DROP;
							end

						endcase

					end

				end

				//Abort if the packet ends
				if(udp_rx_bus.drop) begin
					rx_err_id	<= BAD_MESSAGE_FORMAT;
					rx_state	<= RX_STATE_DROP;
				end

			end	//end RX_STATE_HANDSHAKE_3

			//Rest of ephemeral data
			RX_STATE_HANDSHAKE_4: begin
				if(udp_rx_bus.data_valid) begin

					rx_wcount		<= rx_wcount + udp_rx_bus.bytes_valid;

					if( (rx_wcount + udp_rx_bus.bytes_valid) >= 32)
						rx_state	<= RX_STATE_HANDSHAKE_5;

					case(udp_rx_bus.bytes_valid)
						1: rx_handshake_nonce	<= { rx_handshake_nonce[247:0], udp_rx_bus.data[31:24]};
						2: rx_handshake_nonce	<= { rx_handshake_nonce[239:0], udp_rx_bus.data[31:16]};
						3: rx_handshake_nonce	<= { rx_handshake_nonce[231:0], udp_rx_bus.data[31:8]};
						4: rx_handshake_nonce	<= { rx_handshake_nonce[223:0], udp_rx_bus.data[31:0]};
					endcase

				end

				//Abort if the packet ends
				if(udp_rx_bus.drop) begin
					rx_err_id		<= BAD_MESSAGE_FORMAT;
					rx_state		<= RX_STATE_DROP;
				end

			end	//end RX_STATE_HANDSHAKE_4

			//Done receiving the handshake
			RX_STATE_HANDSHAKE_5: begin

				//Wait for packet to complete with a valid checksum, then act on the handshake
				if(udp_rx_bus.commit) begin
					rx_handshake_begin_en	<= 1;
					rx_handshake_begin_busy	<= 1;
					rx_state				<= RX_STATE_IDLE;
				end

				//Abort if the packet gets dropped
				if(udp_rx_bus.drop) begin
					rx_err_id		<= BAD_MESSAGE_FORMAT;
					rx_state		<= RX_STATE_DROP;
				end
			end	//end RX_STATE_HANDSHAKE_5

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main TX state machine

	logic[255:0]	tx_handshake_nonce	= 0;
	logic[3:0]		tx_count			= 0;

	enum logic[3:0]
	{
		TX_STATE_IDLE				= 4'h0,
		TX_STATE_ERROR				= 4'h1,
		TX_STATE_ERROR_COMMIT		= 4'h2,
		TX_STATE_HANDSHAKE_1		= 4'h3,
		TX_STATE_HANDSHAKE_2		= 4'h4,
		TX_STATE_HANDSHAKE_3		= 4'h5,
		TX_STATE_HANDSHAKE_4		= 4'h6,
		TX_STATE_HANDSHAKE_COMMIT	= 4'h7
	} tx_state = TX_STATE_IDLE;

	always_ff @(posedge clk) begin

		udp_tx_bus.start		<= 0;
		udp_tx_bus.data_valid	<= 0;
		udp_tx_bus.bytes_valid	<= 0;
		udp_tx_bus.commit		<= 0;
		udp_tx_bus.drop			<= 0;

		rx_handshake_begin_done	<= 0;
		rx_err_done				<= 0;

		rng_gen_en				<= 0;

		case(tx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Idle, wait for something to send

			TX_STATE_IDLE: begin

				//Always prepare to send to the source of the incoming packet
				udp_tx_bus.dst_ip	<= udp_rx_bus.src_ip;
				udp_tx_bus.dst_port	<= udp_rx_bus.src_port;
				udp_tx_bus.src_port	<= OUR_PORT;

				//Send an error message
				if(rx_err_en) begin

					udp_tx_bus.start	<= 1;

					//Message is only 2 bytes long
					udp_tx_bus.payload_len	<= 2;

					tx_state				<= TX_STATE_ERROR;

				end

				//Process a valid handshake by sending a Reply-Handshake-Begin message
				else if( (rx_handshake_begin_en || rx_handshake_begin_busy) &&
					!rx_handshake_begin_done && rng_gen_ready) begin

					rng_gen_en		<= 1;
					tx_count		<= 0;
					tx_state		<= TX_STATE_HANDSHAKE_1;

				end

			end	//end TX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//Send a Reply-Handshake-Begin message

			/*
				Save the session start time
				Hash the entire request
				Reply-Handshake-Begin message format:
					REPLY_HANDSHAKE_BEGIN (1 byte)
					Ephemeral data (32-byte nonce)
					Mode data (none)
			 */

			//Generate a nonce
			TX_STATE_HANDSHAKE_1: begin

				if(rng_valid) begin
					tx_handshake_nonce	<= {tx_handshake_nonce[223:0], rng_out };
					tx_count			<= tx_count + 1'h1;

					if(tx_count == 7)
						tx_state		<= TX_STATE_HANDSHAKE_2;

					else
						rng_gen_en		<= 1;
				end

			end	//end TX_STATE_HANDSHAKE_1

			//Prepare to send the reply
			TX_STATE_HANDSHAKE_2: begin
				udp_tx_bus.start		<= 1;
				udp_tx_bus.payload_len	<= 'd35;	//1 byte opcode
													//1 byte sequence length, 32 byte nonce
													//1 byte sequence length
				tx_count				<= 0;
				tx_state				<= TX_STATE_HANDSHAKE_3;
			end	//end TX_STATE_HANDSHAKE_2

			//Send the header, length of nonce, and first 16 bits of nonce
			TX_STATE_HANDSHAKE_3: begin
				udp_tx_bus.data_valid	<= 1;
				udp_tx_bus.bytes_valid	<= 4;
				udp_tx_bus.data[31:24]	<= REPLY_HANDSHAKE_BEGIN;
				udp_tx_bus.data[23:16]	<= 8'd32;
				udp_tx_bus.data[15:0]	<= tx_handshake_nonce[255:240];

				tx_handshake_nonce		<= {tx_handshake_nonce[239:0], 16'h0};

				tx_state				<= TX_STATE_HANDSHAKE_4;
			end	//end TX_STATE_HANDSHAKE_3

			//Send the rest of the nonce
			TX_STATE_HANDSHAKE_4: begin
				tx_count				<= tx_count + 1'h1;

				udp_tx_bus.data_valid	<= 1;
				udp_tx_bus.bytes_valid	<= 4;
				udp_tx_bus.data			<= tx_handshake_nonce[255:224];

				tx_handshake_nonce		<= {tx_handshake_nonce[223:0], 32'h0};

				if(tx_count == 7) begin

					//Last 2 bytes of nonce, plus length of mode_data (always zero)
					//and a padding byte that isn't actually sent
					udp_tx_bus.bytes_valid	<= 3;
					udp_tx_bus.data[15:0]	<= 0;

					tx_state				<= TX_STATE_HANDSHAKE_COMMIT;
				end
			end	//end TX_STATE_HANDSHAKE_4

			TX_STATE_HANDSHAKE_COMMIT: begin
				udp_tx_bus.commit		<= 1;
				rx_handshake_begin_done	<= 1;
				tx_state				<= TX_STATE_IDLE;
			end	//end TX_STATE_COMMIT

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Send a Reply-Handshake-Error message

			TX_STATE_ERROR: begin
				udp_tx_bus.data_valid	<= 1;
				udp_tx_bus.bytes_valid	<= 2;
				udp_tx_bus.data[31:24]	<= REPLY_HANDSHAKE_ERROR;
				udp_tx_bus.data[23:16]	<= rx_err_id;
				udp_tx_bus.data[15:0]	<= 0;
				tx_state				<= TX_STATE_ERROR_COMMIT;
			end	//end TX_STATE_ERROR

			TX_STATE_ERROR_COMMIT: begin
				udp_tx_bus.commit		<= 1;
				rx_err_done				<= 1;
				tx_state				<= TX_STATE_IDLE;
			end	//end TX_STATE_ERROR_COMMIT

		endcase
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	/*
	ila_0 ila(
		.clk(clk),
		.probe0(rx_state),
		.probe1(udp_tx_bus),
		.probe2(rng_gen_en),
		.probe3(rng_gen_ready),
		.probe4(rng_valid),
		.probe5(rng_out),
		.probe6(rx_handshake_begin_en),
		.probe7(tx_count),
		.probe8(tx_state)
	);
	*/

endmodule
