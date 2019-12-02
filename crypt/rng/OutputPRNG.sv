`timescale 1ns/1ps
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief CSPRNG for RandomNumberGenerator output stage
 */
module OutputPRNG(
	input wire			clk,

	//Keying
	input wire			rng_key_update,
	input wire[255:0]	rng_key,
	output logic		gen_rekey_en	= 0,
	output wire[255:0]	gen_rekey_value,

	//API interface
	input wire			gen_en,
	output logic		gen_ready		= 0,
	output logic		rng_valid		= 0,
	output logic[31:0]	rng_out			= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// HMAC-SHA256 based PRNG for now.
	// TODO: AES instead (likely smaller and faster)

	//Initialize count to zero (9.4.1)
	logic[127:0]	rng_count		= 0;

	logic			hmac_start		= 0;
	wire			hmac_ready;
	logic			hmac_update		= 0;
	logic[31:0]		hmac_data_in	= 0;
	logic			hmac_finalize	= 0;

	wire			hmac_valid;
	wire[255:0]		hmac_hash;

	assign gen_rekey_value = hmac_hash;

	StreamingHMACSHA256 hmac(
		.clk(clk),

		.key_update(rng_key_update),
		.key(rng_key),
		.start(hmac_start),
		.ready(hmac_ready),
		.update(hmac_update),
		.data_in(hmac_data_in),
		.bytes_valid(3'd4),
		.finalize(hmac_finalize),

		.hash_valid(hmac_valid),
		.hash(hmac_hash)
	);

	logic			rng_gen_block	= 0;
	logic[1:0]		gen_count		= 0;

	logic[2:0]		out_words_valid	= 0;
	logic			read_pending	= 0;

	enum logic[3:0]
	{
		GEN_STATE_IDLE		= 4'h0,
		GEN_STATE_INIT		= 4'h1,
		GEN_STATE_INPUT		= 4'h2,
		GEN_STATE_FINALIZE	= 4'h3,
		GEN_STATE_WAIT		= 4'h4,
		GEN_STATE_REKEY_1	= 4'h5,
		GEN_STATE_REKEY_2	= 4'h6,
		GEN_STATE_REKEY_3	= 4'h7,
		GEN_STATE_REKEY_4	= 4'h8
	} gen_state = GEN_STATE_IDLE;

	logic			rng_gen_pending	= 0;

	always_ff @(posedge clk) begin

		hmac_start		<= 0;
		hmac_update		<= 0;
		hmac_finalize	<= 0;

		rng_valid		<= 0;
		rng_gen_block	<= 0;

		gen_rekey_en	<= 0;

		if(rng_gen_block)
			rng_gen_pending	<= 1;

		//Bump the counter when we rekey
		if(rng_key_update) begin
			rng_count	<= rng_count + 1;
			gen_ready	<= 1;
		end

		//GenerateRandomData (9.4.4)
		//We differer from Schneier's design here in that output can only be requested in 32-bit chunks,
		//rather than arbitrary sizes.
		if(gen_en || read_pending) begin

			//If output ready, send it
			if(out_words_valid != 0) begin
				rng_valid		<= 1;
				out_words_valid	<= out_words_valid - 1'h1;
				rng_out			<= hmac_hash[out_words_valid*32 - 1 -: 32];
			end

			//If no output ready, generate a new block
			else if(!read_pending) begin
				rng_gen_block	<= 1;
				read_pending	<= 1;
			end
		end

		//GenerateBlocks (9.4.3)
		//Special cased to only generate a single HMAC block at a time.
		//Must be called repeatedly to generate additional data.
		case(gen_state)

			//Wait for a request to generate data
			GEN_STATE_IDLE: begin
				if((rng_gen_block || rng_gen_pending) && gen_ready) begin
					rng_gen_pending	<= 0;
					hmac_start		<= 1;
					gen_count		<= 0;
					gen_state		<= GEN_STATE_INIT;
				end
			end	//end GEN_STATE_IDLE

			//Wait for initial padding
			GEN_STATE_INIT: begin
				if(hmac_ready)
					gen_state	<= GEN_STATE_INPUT;
			end

			//Hash the counter value
			GEN_STATE_INPUT: begin
				hmac_data_in	<= rng_count[gen_count*32 +: 32];
				gen_count		<= gen_count + 1;
				hmac_update		<= 1;
				if(gen_count == 3)
					gen_state	<= GEN_STATE_FINALIZE;
			end	//end GEN_STATE_INPUT

			//Finish hashing
			GEN_STATE_FINALIZE: begin
				hmac_finalize	<= 1;
				gen_state		<= GEN_STATE_WAIT;
			end //end GEN_STATE_FINALIZE

			//Wait for the hash to complete then bump the counter
			GEN_STATE_WAIT: begin

				if(hmac_valid) begin
					gen_state		<= GEN_STATE_IDLE;
					rng_count		<= rng_count + 1;

					//Rekey after 1024 hash blocks
					if(rng_count[9:0] == 0) begin
						gen_state	<= GEN_STATE_REKEY_1;
						hmac_start	<= 1;
					end

					//All good
					else
						out_words_valid	<= 8;
				end

			end	//end GEN_STATE_WAIT

			GEN_STATE_REKEY_1: begin
				if(hmac_ready)
					gen_state	<= GEN_STATE_REKEY_2;
			end	//end GEN_STATE_REKEY_1

			GEN_STATE_REKEY_2: begin
				hmac_data_in	<= rng_count[gen_count*32 +: 32];
				gen_count		<= gen_count + 1;
				hmac_update		<= 1;
				if(gen_count == 3)
					gen_state	<= GEN_STATE_REKEY_3;
			end	//end GEN_STATE_REKEY_2

			GEN_STATE_REKEY_3: begin
				hmac_finalize	<= 1;
				gen_state		<= GEN_STATE_REKEY_4;
			end	//end GEN_STATE_REKEY_3

			GEN_STATE_REKEY_4: begin
				if(hmac_valid) begin
					gen_state		<= GEN_STATE_IDLE;
					rng_count		<= rng_count + 1;
					gen_rekey_en	<= 1;
				end
			end	//end GEN_STATE_REKEY_4

		endcase

	end

endmodule
