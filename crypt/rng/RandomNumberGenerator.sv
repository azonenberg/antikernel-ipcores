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

	Early prototype implementation, needs third party review!!!

	Reference: https://www.schneier.com/academic/paperfiles/fortuna.pdf
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
	output wire				gen_ready,
	output wire				rng_valid,
	output wire[31:0]		rng_out,

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
	// PRNG for the output stage

	//Initialize key to zero (9.4.1)
	logic			rng_key_update	= 0;
	logic[255:0]	rng_key			= 0;

	wire			gen_rekey_en;
	wire[255:0]		gen_rekey_value;

	OutputPRNG prng(
		.clk(clk),

		.rng_key_update(rng_key_update),
		.rng_key(rng_key),
		.gen_rekey_en(gen_rekey_en),
		.gen_rekey_value(gen_rekey_value),

		.gen_en(gen_en),
		.gen_ready(gen_ready),
		.rng_valid(rng_valid),
		.rng_out(rng_out)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Internal entropy source

	wire		jitter_word_valid;
	wire[31:0]	jitter_word;

	ClockJitterEntropySource jitter_source(
		.clk(clk),
		.clk_ring(clk_ring),
		.jitter_word(jitter_word),
		.jitter_word_valid(jitter_word_valid)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Seed persistence logic

	wire		trig_out;

	wire		eeprom_seed_valid;
	wire[255:0]	eeprom_seed;

	SeedPersistenceManager #(
		.ADDR_PINS(ADDR_PINS)
	) persistence_mgr(
		.clk(clk),

		.i2c_driver_req(i2c_driver_req),
		.i2c_driver_ack(i2c_driver_ack),
		.i2c_driver_done(i2c_driver_done),
		.i2c_driver_cin(i2c_driver_cin),
		.i2c_driver_cout(i2c_driver_cout),

		.load_en(trig_out && eeprom_serial_valid),

		.eeprom_seed_valid(eeprom_seed_valid),
		.eeprom_seed(eeprom_seed)
	);

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Entropy pools (9.5.2)

	//Index of the pool currently being written to by new entropy
	logic[4:0]		pool_wptr			= 0;
	logic[255:0]	entropy_pools[31:0];

	//Number of reseeds we've done
	logic[31:0]		num_reseeds			= 0;

	//Old data of the pool we're writing to
	wire[255:0]		old_poolhash	= entropy_pools[pool_wptr];

	initial begin
		for(integer i=0; i<32; i++)
			entropy_pools[i]	<= 0;
	end

	logic[31:0] 	reseed_timer		= 1;
	logic			reseed_pending		= 0;

	logic[3:0]		reseed_count		= 0;

	logic			jitter_word_pending	= 0;
	logic			sensor_word_pending	= 0;
	logic			entropy_pending		= 0;

	logic			booting				= 1;

	enum logic[3:0]
	{
		POOL_STATE_IDLE_0			= 4'h0,
		POOL_STATE_IDLE_1			= 4'h1,
		POOL_STATE_DIE_SERIAL		= 4'h2,
		POOL_STATE_EEPROM_SERIAL	= 4'h3,
		POOL_STATE_SAVED_SEED		= 4'h4,
		POOL_STATE_OLDHASH			= 4'h5,
		POOL_STATE_FINALIZE			= 4'h6,
		POOL_STATE_WAIT				= 4'h7,
		POOL_STATE_RESEED_0			= 4'h8
	} pool_state = POOL_STATE_IDLE_0;

	always_ff @(posedge clk) begin

		sha_start		<= 0;
		sha_update		<= 0;
		sha_finalize	<= 0;
		rng_key_update	<= 0;

		if(jitter_word_valid)
			jitter_word_pending	<= 1;
		if(sensors_update)
			sensor_word_pending	<= 1;
		if(entropy_en)
			entropy_pending		<= 1;

		//Reseed every ~100ms (assuming 100 MHz clk)
		reseed_timer		<= reseed_timer + 1;
		if(reseed_timer == 32'd9999999) begin
			reseed_pending	<= 1;
			reseed_timer	<= 1;
		end

		//Key switch (9.4.4)
		if(gen_rekey_en) begin
			rng_key			<= gen_rekey_value;
			rng_key_update	<= 1;
		end

		//Keep track of how many reseeds we've done
		if(rng_key_update)
			num_reseeds			<= num_reseeds + 1'h1;

		case(pool_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Idle - constantly hash stuff in a loop

			POOL_STATE_IDLE_0: begin
				sha_start	<= 1;
				pool_state	<= POOL_STATE_IDLE_1;
			end	//end POOL_STATE_IDLE_0

			POOL_STATE_IDLE_1: begin

				//If we're done booting, hash in the boot-time data
				if(booting && die_serial_valid && eeprom_serial_valid && eeprom_seed_valid) begin
					pool_state		<= POOL_STATE_DIE_SERIAL;
					reseed_count	<= 0;
				end

				//External entropy gets highest precedence
				else if(entropy_en || entropy_pending) begin
					sha_update		<= 1;
					sha_data_in		<= entropy_data;
					sha_bytes_valid	<= 4;
					entropy_pending	<= 0;
				end

				//Then sensors (XOR together the 4 readings to get a single 32-bit word)
				else if(sensors_update || sensor_word_pending) begin
					sha_update		<= 1;
					sha_data_in		<= { die_temp ^ volt_aux, volt_core ^ volt_ram };
					sha_bytes_valid	<= 4;
					sensor_word_pending	<= 0;
				end

				//Add jitter if we have nothing left to hash
				else if(jitter_word_valid || jitter_word_pending) begin
					sha_update			<= 1;
					sha_data_in			<= jitter_word;
					jitter_word_pending	<= 0;
				end

				//If we're due to reseed, handle that
				else if(reseed_pending) begin
					reseed_pending		<= 0;
					reseed_count		<= 0;
					pool_state			<= POOL_STATE_RESEED_0;
				end

			end	//end POOL_STATE_IDLE_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Boot sequence - mix in die and EEPROM serial numbers plus the saved state from last boot

			POOL_STATE_DIE_SERIAL: begin
				reseed_count		<= reseed_count + 1;

				sha_update			<= 1;
				sha_bytes_valid		<= 4;

				if(reseed_count == 0)
					sha_data_in		<= die_serial[31:0];
				else begin
					sha_data_in		<= die_serial[63:32];
					reseed_count	<= 0;
					pool_state		<= POOL_STATE_EEPROM_SERIAL;
				end
			end	//end POOL_STATE_DIE_SERIAL

			POOL_STATE_EEPROM_SERIAL: begin
				reseed_count		<= reseed_count + 1;

				sha_update			<= 1;
				sha_bytes_valid		<= 4;

				case(reseed_count)
					0:	sha_data_in	<= eeprom_serial[31:0];
					1:	sha_data_in	<= eeprom_serial[63:32];
					2:	sha_data_in	<= eeprom_serial[95:64];
					3: begin
						sha_data_in		<= eeprom_serial[31:0];
						reseed_count	<= 0;
						pool_state		<= POOL_STATE_SAVED_SEED;
					end
				endcase
			end	//end POOL_STATE_EEPROM_SERIAL

			POOL_STATE_SAVED_SEED: begin
				reseed_count		<= reseed_count + 1;

				sha_update			<= 1;
				sha_bytes_valid		<= 4;

				sha_data_in			<= eeprom_seed[reseed_count*32 +: 32];

				if(reseed_count == 15) begin
					reseed_count	<= 0;
					pool_state		<= POOL_STATE_OLDHASH;
				end
			end	//end POOL_STATE_SAVED_SEED

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Re-seed the generator (hash the working hash plus one or more entropy pools)
			//TODO

			POOL_STATE_RESEED_0: begin
			end	//end POOL_STATE_RESEED_0

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Finalize the hash

			//Mix in the old pool hash
			POOL_STATE_OLDHASH: begin
				reseed_count		<= reseed_count + 1;

				sha_update			<= 1;
				sha_bytes_valid		<= 4;

				sha_data_in			<= old_poolhash[reseed_count*32 +: 32];

				if(reseed_count == 15) begin
					reseed_count	<= 0;
					pool_state		<= POOL_STATE_FINALIZE;
				end
			end	//end POOL_STATE_OLDHASH

			POOL_STATE_FINALIZE: begin
				sha_finalize		<= 1;
				pool_state			<= POOL_STATE_WAIT;
			end	//end POOL_STATE_FINALIZE

			POOL_STATE_WAIT: begin
				if(sha_hash_valid) begin
					entropy_pools[pool_wptr]	<= sha_hash;
					pool_state			<= POOL_STATE_IDLE_0;

					//Start writing to the next pool
					pool_wptr			<= pool_wptr + 1;

					//If we were waiting for initialization, key the generator at this point
					if(booting) begin
						booting			<= 0;

						rng_key			<= sha_hash;
						rng_key_update	<= 1;
					end
				end
			end	//end POOL_STATE_WAIT

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The LA

	ila_1 ila(
		.clk(clk),
		.probe0(jitter_word_valid),
		.probe1(jitter_word),
		.probe2(pool_state),
		.probe3(sha_start),
		.probe4(sha_finalize),
		.probe5(sha_data_in),
		.probe6(sha_bytes_valid),
		.probe7(reseed_count),
		.probe8(die_serial[15:0]),
		.probe9(eeprom_seed[15:0]),
		.probe10(eeprom_serial[15:0]),
		.probe11(sha_hash_valid),
		.probe12(booting),
		.probe13(reseed_pending),
		.probe14(gen_en),
		.probe15(gen_ready),
		.probe16(rng_valid),
		.probe17(rng_out),

		.probe18(sha_update),
		.probe19(entropy_en),
		.probe20(die_serial_valid),
		.probe21(eeprom_serial_valid),
		.probe22(eeprom_seed_valid),
		.trig_out(trig_out),
		.trig_out_ack(trig_out)
	);

endmodule
