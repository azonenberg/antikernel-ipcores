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

`include "I2CTransceiver.svh"

/**
	@brief Controller for a TI INA226 instrumentation amp / ADC

	Calibration example: 50 mR shunt
	I = V / 0.05 = V * 20. This is in volts/amps units
	One LSB of Vshunt is 2.5 uV or 50 uA.
	Cal = 50 gives current reading in uA
	Cal = 5 gives reading in tens of uA
 */
module INA226 #(
	parameter SLAVE_ADDR 		= 8'h80,
	parameter CFG_AVERAGE		= 1,	//must be 0...7, see table 3 of INA226 datasheet
	parameter CFG_VBUS_TIME		= 4,	//must be 0...7, see table 4 of INA226 datasheet
	parameter CFG_VSHUNT_TIME	= 4,	//must be 0...7, see table 5 of INA226 datasheet
	parameter CALIBRATION		= 50
) (
	input wire				clk,
	output logic			booting			= 1,

	input wire				poll_en,
	output logic			poll_done		= 0,

	output logic[15:0]		bus_voltage		= 0,	//1.25 mV per LSB
	output logic[15:0]		current_scaled	= 0,	//shunt voltage * cal constant

	output wire				driver_request,
	output wire				driver_done,
	input wire				driver_ack,
	output i2c_in_t			driver_cin,
	input wire i2c_out_t	driver_cout
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register access

	logic		open 		= 0;
	wire		ready;
	logic		select 		= 0;

	enum logic[7:0]
	{
		REG_CONFIG	= 8'h00,
		REG_VSHUNT	= 8'h01,
		REG_VBUS	= 8'h02,
		REG_POWER	= 8'h03,
		REG_CURRENT	= 8'h04,
		REG_CAL		= 8'h05
	} addr					= REG_CONFIG;

	logic		we			= 0;
	logic		close		= 0;
	wire		rdata_valid;
	wire[7:0]	rdata;
	wire		err;
	wire		need_wdata;
	logic		wdata_valid	= 0;
	logic[7:0]	wdata		= 0;

	I2CRegisterHelper #(
		.ADDR_BYTES(1)
	) rhelper (
		.clk(clk),
		.slave_addr(SLAVE_ADDR),

		.open(open),
		.ready(ready),
		.select(select),
		.addr(addr),
		.we(we),
		.burst_len(8'h2),	//all registers are 16 bits
		.close(close),
		.rdata_valid(rdata_valid),
		.rdata(rdata),
		.err(err),
		.need_wdata(need_wdata),
		.wdata_valid(wdata_valid),
		.wdata(wdata),

		.request(driver_request),
		.done(driver_done),
		.ack(driver_ack),
		.cin(driver_cin),
		.cout(driver_cout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control state machine

	enum logic[3:0]
	{
		STATE_BOOT_0	= 4'h0,
		STATE_BOOT_1	= 4'h1,
		STATE_BOOT_2	= 4'h2,
		STATE_BOOT_3	= 4'h3,
		STATE_BOOT_4	= 4'h4,
		STATE_BOOT_5	= 4'h5,
		STATE_IDLE		= 4'h6,
		STATE_POLL_0	= 4'h7,
		STATE_POLL_1	= 4'h8,
		STATE_POLL_2	= 4'h9,
		STATE_POLL_3	= 4'ha,
		STATE_POLL_4	= 4'hb
	} state = STATE_BOOT_0;

	logic[7:0]	saved_rdata;

	always_ff @(posedge clk) begin

		open		<= 0;
		close		<= 0;
		select		<= 0;
		wdata_valid	<= 0;

		poll_done	<= 0;

		if(rdata_valid)
			saved_rdata	<= rdata;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BOOT - load configuration and calibation register with the requested settings

			STATE_BOOT_0: begin
				open	<= 1;
				state	<= STATE_BOOT_1;
			end	//end STATE_BOOT_0

			STATE_BOOT_1: begin
				if(ready) begin
					select		<= 1;
					addr		<= REG_CONFIG;
					we			<= 1;
					state		<= STATE_BOOT_2;
				end
			end	//end STATE_BOOT_1

			STATE_BOOT_2: begin
				if(need_wdata) begin
					wdata_valid	<= 1;
					wdata		<= {4'h4, CFG_AVERAGE[2:0], CFG_VBUS_TIME[2]};
					state		<= STATE_BOOT_3;
				end
			end	//end STATE_BOOT_2

			STATE_BOOT_3: begin
				if(need_wdata) begin
					wdata_valid	<= 1;
					wdata		<= {CFG_VBUS_TIME[1:0], CFG_VSHUNT_TIME, 3'b111};
				end
				if(ready) begin
					select		<= 1;
					addr		<= REG_CONFIG;
					we			<= 1;
					state		<= STATE_BOOT_4;
				end
			end	//end STATE_BOOT_3

			STATE_BOOT_4: begin
				if(need_wdata) begin
					wdata_valid	<= 1;
					wdata		<= CALIBRATION[15:8];
					state		<= STATE_BOOT_5;
				end
			end	//end STATE_BOOT_4

			STATE_BOOT_5: begin
				if(need_wdata) begin
					wdata_valid	<= 1;
					wdata		<= CALIBRATION[7:0];
				end
				if(ready) begin
					close		<= 1;
					state		<= STATE_IDLE;
					booting		<= 0;
				end
			end	//end STATE_BOOT_5

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for an update request

			STATE_IDLE: begin

				if(poll_en) begin
					open		<= 1;
					state		<= STATE_POLL_0;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// POLL - read sensor registers

			//Request read of Vbus
			STATE_POLL_0: begin
				if(ready) begin
					select		<= 1;
					addr		<= REG_VBUS;
					we			<= 0;
					state		<= STATE_POLL_1;
				end
			end	//end STATE_POLL_0

			STATE_POLL_1: begin
				if(rdata_valid)
					state		<= STATE_POLL_2;
			end	//end STATE_POLL_1


			STATE_POLL_2: begin
				if(ready) begin

					select		<= 1;
					addr		<= REG_CURRENT;
					we			<= 0;

					state		<= STATE_POLL_3;

					bus_voltage	<= {saved_rdata, rdata};
				end
			end	//end STATE_POLL_2

			STATE_POLL_3: begin
				if(rdata_valid)
					state		<= STATE_POLL_4;
			end	//end STATE_POLL_3

			STATE_POLL_4: begin
				if(ready) begin
					current_scaled	<= { saved_rdata, rdata };
					close			<= 1;
					poll_done		<= 1;
					state			<= STATE_IDLE;
				end
			end

		endcase

	end

endmodule
