`timescale 1ns / 1ps
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

/**
	@brief Streaming HMAC-SHA-256 core (reference: RFC 4231)

	The key must be <= 512 bits in length, right-padded with zeroes if necessary. If the desired key is >512 bits long,
	pass SHA256(key) as the key.

	After asserting "start", wait until "ready" goes high before sending input to the HMAC core.
 */
module StreamingHMACSHA256(
	input wire			clk,

	input wire			key_update,
	input wire[511:0]	key,

	input wire			start,
	output logic		ready		= 0,
	input wire			update,
	input wire[31:0]	data_in,
	input wire[2:0]		bytes_valid,
	input wire			finalize,

	output logic		hash_valid	= 0,
	output logic[255:0]	hash		= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Key padding

	logic[511:0] o_key_pad	= 0;
	logic[511:0] i_key_pad	= 0;

	always_ff @(posedge clk) begin
		if(key_update) begin
			for(integer i=0; i<64; i++) begin
				o_key_pad[i*8 +: 8]	<= key[(63-i)*8 +: 8] ^ 8'h5c;
				i_key_pad[i*8 +: 8]	<= key[(63-i)*8 +: 8] ^ 8'h36;
			end
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual hash core

	logic		hash_start 			= 0;

	logic		hash_update 		= 0;
	logic[31:0]	hash_data_in		= 0;
	logic[2:0]	hash_bytes_valid	= 0;
	logic		hash_finalize		= 0;

	wire		hash_int_valid;
	wire[255:0]	hash_int;

	StreamingSHA256 hasher(
		.clk(clk),
		.start(hash_start),
		.update(hash_update),
		.data_in(hash_data_in),
		.bytes_valid(hash_bytes_valid),
		.finalize(hash_finalize),
		.hash_valid(hash_int_valid),
		.hash(hash_int)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// HMAC state machine

	enum logic[2:0]
	{
		STATE_IDLE		= 0,
		STATE_IN_PAD	= 1,
		STATE_MESSAGE	= 2,
		STATE_IN_WAIT	= 3,
		STATE_OUT_PAD	= 4,
		STATE_REHASH	= 5,
		STATE_FINALIZE	= 6,
		STATE_OUT_WAIT	= 7
	} state = STATE_IDLE;

	logic[3:0] count = 0;

	always_ff @(posedge clk) begin
		hash_start		<= 0;
		hash_update		<= 0;
		hash_finalize	<= 0;
		ready			<= 0;
		hash_valid		<= 0;

		case(state)

			//Wait for a new hash to start
			STATE_IDLE: begin
				if(start) begin
					hash_start	<= 1;
					count		<= 0;
					state		<= STATE_IN_PAD;
				end
			end	//end STATE_IDLE

			//Hash i_key_pad
			//TODO: cache state so we don't have to recompute every message!
			STATE_IN_PAD: begin
				hash_update			<= 1;
				hash_bytes_valid	<= 4;
				hash_data_in		<= i_key_pad[32*count +: 32];
				count				<= count + 1;
				if(count == 15) begin
					state			<= STATE_MESSAGE;
					ready			<= 1;
				end
			end	//end STATE_IN_PAD

			//Hash the message proper
			STATE_MESSAGE: begin

				if(update) begin
					hash_update			<= 1;
					hash_bytes_valid	<= bytes_valid;
					hash_data_in		<= data_in;
				end

				if(finalize) begin
					hash_finalize		<= 1;
					state				<= STATE_IN_WAIT;
				end

			end	//end STATE_MESSAGE

			//Wait for the inner hash to complete
			STATE_IN_WAIT: begin
				if(hash_int_valid) begin
					hash_start			<= 1;
					count				<= 0;
					state				<= STATE_OUT_PAD;
				end
			end	//end STATE_IN_WAIT

			//Hash o_key_pad
			STATE_OUT_PAD: begin
				hash_update			<= 1;
				hash_bytes_valid	<= 4;
				hash_data_in		<= o_key_pad[32*count +: 32];
				count				<= count + 1;
				if(count == 15) begin
					state			<= STATE_REHASH;
					count			<= 0;
				end
			end	//end STATE_OUT_PAD

			//Hash H(i_key_pad || message)
			STATE_REHASH: begin
				hash_update			<= 1;
				hash_bytes_valid	<= 4;
				hash_data_in		<= hash_int[32*(7-count) +: 32];
				count				<= count + 1;
				if(count == 7) begin
					state			<= STATE_FINALIZE;
					count			<= 0;
				end
			end	//end STATE_REHASH

			STATE_FINALIZE: begin
				hash_finalize		<= 1;
				state				<= STATE_OUT_WAIT;
			end	//end STATE_FINALIZE

			//Wait for the outer hash to complete
			STATE_OUT_WAIT: begin
				if(hash_int_valid) begin
					hash_valid		<= 1;
					hash			<= hash_int;
					state			<= STATE_IDLE;
				end
			end	//end STATE_OUT_WAIT

		endcase

	end

endmodule
