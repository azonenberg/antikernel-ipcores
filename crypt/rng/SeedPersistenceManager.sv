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
	@brief Handles saving and loading RandomNumberGenerator seed state via an external 24C-compatible EEPROM.
 */
module SeedPersistenceManager #(
	parameter				ADDR_PINS	= 3'h0		//eeprom low address bits
)(
	input wire	clk,

	//I2C bus to EEPROM for persisting PRNG state
	output wire				i2c_driver_req,
	input wire				i2c_driver_ack,
	output wire				i2c_driver_done,
	output i2c_in_t			i2c_driver_cin,
	input wire i2c_out_t	i2c_driver_cout,

	//Load/store commands
	input wire				load_en,

	//Input seed
	output logic			eeprom_seed_valid	= 0,
	output logic[255:0]		eeprom_seed			= 0
	);

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

	enum logic[3:0]
	{
		SEED_STATE_BOOT_0		= 4'h0,
		SEED_STATE_BOOT_1		= 4'h1,
		SEED_STATE_BOOT_READ	= 4'h2,
		SEED_STATE_IDLE			= 4'h3
	} seed_state = SEED_STATE_BOOT_0;

	always_ff @(posedge clk) begin

		open		<= 0;
		close		<= 0;
		select		<= 0;

		case(seed_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Initialization

			SEED_STATE_BOOT_0: begin

				if(load_en) begin
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

				if(burst_done) begin
					seed_state			<= SEED_STATE_IDLE;
					eeprom_seed_valid	<= 1;
				end
			end	//end SEED_STATE_BOOT_READ

			SEED_STATE_IDLE: begin
			end	//end SEED_STATE_IDLE

		endcase

	end

endmodule


