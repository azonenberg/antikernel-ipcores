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

`include "I2CTransceiver.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief CSPRNG using the Fortuna architecture with HMAC-SHA256 as the generator function
 */
module RandomNumberGenerator #(
	parameter				ADDR_PINS	= 3'h0		//eeprom low address bits
)(

	//Main generator clock (from PLL / external oscillator, must NOT be from on die source)
	input wire				clk,

	//On die ring oscillator clock from STARTUPE2 or similar
	input wire				clk_ring,

	//API interface
	input wire				gen_en,
	output logic			gen_ready	= 0,
	output logic			rng_valid	= 0,
	output logic[31:0]		rng_out		= 0,

	//Die serial number (from DeviceInfo_7series)
	//Tie to zero if not available in target FPGA
	input wire				die_serial_valid,
	input wire[63:0]		die_serial,

	//EEPROM serial number (from I2CMACAddressReader)
	input wire				eeprom_serial_valid,
	input wire[127:0]		eeprom_serial,

	//I2C bus to EEPROM for persisting PRNG state
	output wire				i2c_driver_req,
	input wire				i2c_driver_ack,
	output wire				i2c_driver_done,
	output i2c_in_t			i2c_driver_cin,
	input wire i2c_out_t	i2c_driver_cout,

	//XADC input for runtime entropy accumulation (from OnDieSensors_7series or similar)
	//Tie to zero if no sensors are available, but any sensor data is better than none.
	input wire[15:0]		die_temp,
	input wire[15:0]		volt_core,
	input wire[15:0]		volt_ram,
	input wire[15:0]		volt_aux,
	input wire				sensors_update,

	//Additional entropy injection from user-supplied events. Can be anything.
	input wire				entropy_en,
	input wire[31:0]		entropy_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// HMAC-SHA256 core for RNG output

	//Initialize key/count to zero (9.4.1)
	logic			rng_key_update	= 0;
	logic[255:0]	rng_key			= 0;
	logic[127:0]	rng_count		= 0;

	logic			hmac_start			= 0;
	wire			hmac_ready;
	logic			hmac_update			= 0;
	logic[31:0]		hmac_data_in		= 0;
	logic			hmac_finalize		= 0;

	wire			hmac_valid;
	wire[255:0]		hmac_hash;

	StreamingHMACSHA256 hmac(
		.clk(clk),

		.key_update(rng_key_update),
		.key(rng_key),
		.start(hmac_start),
		.ready(hmac_ready),
		.update(hmac_update),
		.data_in(hmac_data_in),
		.bytes_valid(3'd4),
		.finalize(finalize),

		.hash_valid(hmac_valid),
		.hash(hmac_hash)
	);

	logic			rng_gen_block	= 0;
	logic[1:0]		gen_count	= 0;

	enum logic[3:0] gen_state
	{
		GEN_STATE_IDLE		= 4'h0,
		GEN_STATE_INIT		= 4'h1,
		GEN_STATE_INPUT		= 4'h2,
		GEN_STATE_FINALIZE	= 4'h3,
		GEN_STATE_WAIT		= 4'h4
	} = GEN_STATE_IDLE;

	always_ff @(posedge clk) begin

		hmac_start		<= 0;
		hmac_update		<= 0;
		hmac_finalize	<= 0;

		//Bump the counter when we rekey
		if(rng_key_update) begin
			rng_count	<= rng_count + 1;
			gen_ready	<= 1;
		end

		//GenerateBlocks (9.4.3)
		//Special cased to only generate a single HMAC block at a time.
		//Must be called repeatedly to generate additional data.
		case(gen_state)

			//Wait for a request to generate data
			GEN_STATE_IDLE: begin
				if(rng_gen_block && gen_ready) begin
					hmac_start	<= 1;
					gen_count	<= 0;
					gen_state	<= GEN_STATE_INIT;
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
					gen_state	<= GEN_STATE_IDLE;
					rng_count	<= rng_count + 1;

					//TODO: save results somewhere etc
				end

			end	//end GEN_STATE_WAIT

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SHA256 core for reseeding (9.4.2)

	logic		sha_start		= 0;
	logic		sha_update		= 0;
	logic[31:0]	sha_data_in		= 0;
	logic[2:0]	sha_bytes_valid	= 0;
	logic		sha_finalize	= 0;
	wire		sha_hash_valid;
	wire[255:0]	sha_hash;

	StreamingSHA256 sha(
		.clk(clk),
		.start(sha_start),
		.update(sha_update),
		.data_in(sha_data_in),
		.bytes_valid(sha_bytes_valid),
		.finalize(sha_finalize),
		.hash_valid(sha_hash_valid),
		.hash(sha_hash)
	);

	//To start a reseed operation, assert reseed_en with reseed_data set to the additional data being mixed in.
	logic			reseed_en		= 0;
	logic[255:0]	reseed_input	= 0;

	logic[2:0]		reseed_count	= 0;

	enum logic[3:0]
	{
		RESEED_STATE_IDLE		= 4'h0,
		RESEED_STATE_KEY		= 4'h1,
		RESEED_STATE_DATA		= 4'h2,
		RESEED_STATE_FINALIZE	= 4'h3,
		RESEED_STATE_REKEY		= 4'h4
	} reseed_state = RESEED_STATE_IDLE;

	always_ff @(posedge clk) begin

		reseed_en		<= 0;
		sha_start		<= 0;
		sha_update		<= 0;
		sha_finalize	<= 0;
		rng_key_update	<= 0;

		case(reseed_state)

			//Wait for a new reseed request
			RESEED_STATE_IDLE: begin

				if(reseed_en) begin
					sha_start		<= 1;
					reseed_count	<= 0;
					reseed_state	<= RESEED_STATE_KEY;
				end

			end	//end RESEED_STATE_IDLE

			//Hash in the old key
			RESEED_STATE_KEY: begin
				sha_data_in		<= rng_key[reseed_count*32 +: 32];
				sha_bytes_valid	<= 1;
				reseed_count	<= reseed_count + 1'h1;

				if(reseed_count == 7)
					reseed_state	<= RESEED_STATE_DATA;
			end	//end RESEED_STATE_KEY

			//Hash in the new data
			RESEED_STATE_DATA: begin
				sha_data_in		<= reseed_input[reseed_count*32 +: 32];
				sha_bytes_valid	<= 1;
				reseed_count	<= reseed_count + 1'h1;

				if(reseed_count == 7)
					reseed_state	<= RESEED_STATE_FINALIZE;
			end	//end RESEED_STATE_DATA

			//Finish hashing
			RESEED_STATE_FINALIZE: begin
				sha_finalize	<= 1;
				reseed_state	<= RESEED_STATE_REKEY;
			end	//end RESEED_STATE_FINALIZE

			//Rekey the HMAC core when hashing completes
			RESEED_STATE_REKEY: begin
				if(sha_hash_valid) begin
					rng_key			<= sha_hash;
					rng_key_valid	<= 1;
					reseed_state	<= RESEED_STATE_IDLE;
				end
			end	//end RESEED_STATE_REKEY

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Generate some entropy from jitter between STARTUPE2 clock and the external clock
	// Apply minimal whitening with a von Neumann corrector

	logic	toggle_ring	= 0;
	always_ff @(posedge clk_ring) begin
		toggle_ring	<= !toggle_ring;
	end

	wire	toggle_sync;
	ThreeStageSynchronizer sync_toggle_ring(
		.clk_in(clk_ring),
		.din(toggle_ring),
		.clk_out(clk),
		.dout(toggle_sync)
	);

	logic[9:0]	count = 0;
	logic[1:0]	jitter_bits			= 0;
	logic		jitter_bit			= 0;
	logic		jitter_bit_valid	= 0;

	logic[31:0]	jitter_word			= 0;
	logic[4:0]	jitter_word_count	= 0;
	logic		jitter_word_valid	= 0;
	always_ff @(posedge clk) begin

		jitter_bit_valid		<= 0;
		jitter_word_valid		<= 0;

		count					<= count + 1;

		if(count == 0) begin
			jitter_bits[0]		<= toggle_sync;

			if(jitter_bits == 2'b10) begin
				jitter_bit_valid	<= 1;
				jitter_bit			<= 1;
			end
			else if(jitter_bits == 2'b01) begin
				jitter_bit_valid	<= 1;
				jitter_bit			<= 0;
			end

		end
		if(count == 512)
			jitter_bits[1]		<= toggle_sync;

		if(jitter_bit_valid) begin
			jitter_word_count	<= jitter_word_count + 1;
			jitter_word			<= { jitter_word[30:0], jitter_bit };;
			if(jitter_word_count == 31)
				jitter_word_valid	<= 1;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register helper block for the EEPROM

	logic		open 				= 0;
	wire		reg_ready;
	logic		select 				= 0;
	logic		close 				= 0;
	wire		rdata_valid;
	wire[7:0]	rdata;
	wire		burst_done;
	wire		err;
	logic[7:0]	eeprom_addr			= 0;
	logic[7:0]	eeprom_burst_len	= 0;
	logic		eeprom_we			= 0;
	logic		eeprom_wdata_valid	= 0;
	logic[7:0]	eeprom_wdata;
	wire		eeprom_need_wdata;

	I2CRegisterHelper #(
		.ADDR_BYTES(1)		//2 Kbit = 256 byte eeprom
	) helper (
		.clk(clk),
		.slave_addr({4'ha, ADDR_PINS[2:0], 1'h1}),

		.open(open),
		.ready(reg_ready),
		.select(select),
		.addr(eeprom_addr),
		.we(eeprom_we),
		.burst_len(eeprom_burst_len),
		.close(close),
		.rdata_valid(rdata_valid),
		.rdata(rdata),
		.err(err),
		.wdata_valid(eeprom_wdata_valid),
		.wdata(eeprom_wdata),
		.need_wdata(eeprom_need_wdata),
		.burst_done(burst_done),

		.request(i2c_driver_req),
		.done(i2c_driver_done),
		.ack(i2c_driver_ack),
		.cin(i2c_driver_cin),
		.cout(i2c_driver_cout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Seed loading / persisting

	wire	trig_out;

	enum logic[3:0]
	{
		SEED_STATE_BOOT_0		= 4'h0,
		SEED_STATE_BOOT_1		= 4'h1,
		SEED_STATE_BOOT_READ	= 4'h2,
		SEED_STATE_IDLE			= 4'h3
	} seed_state = SEED_STATE_BOOT_0;

	logic[255:0]	eeprom_seed;

	always_ff @(posedge clk) begin

		open		<= 0;
		close		<= 0;
		select		<= 0;

		case(seed_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Initialization

			SEED_STATE_BOOT_0: begin

				//wait until the initial eeprom read is done, this confirms everything is initialized
				if(trig_out && eeprom_serial_valid) begin
					open		<= 1;
					seed_state	<= SEED_STATE_BOOT_1;
				end

			end	//end SEED_STATE_BOOT_0

			SEED_STATE_BOOT_1: begin
				if(reg_ready && !open) begin
					select				<= 1;
					eeprom_addr			<= 8'ha0;
					eeprom_burst_len	<= 8'h10;
					seed_state			<= SEED_STATE_BOOT_READ;
				end
			end	//end SEED_STATE_BOOT_1

			SEED_STATE_BOOT_READ: begin
				if(rdata_valid)
					eeprom_seed			<= { eeprom_seed[247:0], rdata };

				if(burst_done)
					seed_state			<= SEED_STATE_IDLE;
			end	//end SEED_STATE_BOOT_READ

			SEED_STATE_IDLE: begin
			end	//end SEED_STATE_IDLE

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual RNG engine

	enum logic[3:0] rng_state
	{
		RNG_STATE_INIT		= 4'h0
	} = RNG_STATE_INIT;

	always_ff @(posedge clk) begin

		case(rng_state)

			RNG_STATE_INIT: begin
			end

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The LA

	ila_1 ila(
		.clk(clk),
		.probe0(toggle_sync),
		.probe1(jitter_bit),
		.probe2(jitter_bit_valid),
		.probe3(jitter_word_valid),
		.probe4(jitter_word),
		.trig_out(trig_out),
		.trig_out_ack(trig_out)
	);

endmodule
