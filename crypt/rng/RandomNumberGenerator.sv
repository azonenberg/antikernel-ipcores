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

	wire			rng_key_update;
	wire[255:0]		rng_key;

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

	wire		sha_start;
	wire		sha_update;
	wire[31:0]	sha_data_in;
	wire[2:0]	sha_bytes_valid;
	wire		sha_finalize;
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

	EntropyPools pools(
		.clk(clk),

		.gen_rekey_en(gen_rekey_en),
		.gen_rekey_value(gen_rekey_value),
		.rng_key_update(rng_key_update),
		.rng_key(rng_key),

		.sha_start(sha_start),
		.sha_update(sha_update),
		.sha_data_in(sha_data_in),
		.sha_bytes_valid(sha_bytes_valid),
		.sha_finalize(sha_finalize),
		.sha_hash_valid(sha_hash_valid),
		.sha_hash(sha_hash),

		.die_serial_valid(die_serial_valid),
		.die_serial(die_serial),
		.eeprom_serial_valid(eeprom_serial_valid),
		.eeprom_serial(eeprom_serial),
		.eeprom_seed_valid(eeprom_seed_valid),
		.eeprom_seed(eeprom_seed),

		.die_temp(die_temp),
		.volt_core(volt_core),
		.volt_ram(volt_ram),
		.volt_aux(volt_aux),
		.sensors_update(sensors_update),

		.jitter_word(jitter_word),
		.jitter_word_valid(jitter_word_valid),

		.entropy_en(entropy_en),
		.entropy_data(entropy_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The LA

	ila_1 ila(
		.clk(clk),
		.probe0(jitter_word_valid),
		.probe1(jitter_word),
		.probe2(pools.pool_state),
		.probe3(sha_start),
		.probe4(sha_finalize),
		.probe5(sha_data_in),
		.probe6(sha_bytes_valid),
		.probe7(pools.reseed_count),
		.probe8(die_serial[15:0]),
		.probe9(eeprom_seed[15:0]),
		.probe10(eeprom_serial[15:0]),
		.probe11(sha_hash_valid),
		.probe12(pools.booting),
		.probe13(pools.reseed_pending),
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
