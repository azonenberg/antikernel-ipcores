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
	@brief Logic to read an I2C EEPROM with MAC address (AT24MAC402 and compatible)
 */
module I2CMACAddressReader #(
	parameter ADDR_PINS = 3'h0				//State of the 3 address pins on the EEPROM
)(
	input wire				clk,

	output wire				driver_req,
	input wire				driver_ack,
	output wire				driver_done,
	output i2c_in_t			driver_cin,
	input wire i2c_out_t	driver_cout,

	output logic			done		= 0,
	output logic[47:0]		mac			= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register helper block

	logic		open = 0;
	wire		ready;
	logic		select = 0;
	logic		close = 0;
	wire		rdata_valid;
	wire[7:0]	rdata;
	wire		burst_done;
	wire		err;

	I2CRegisterHelper #(
		.ADDR_BYTES(1)		//2 Kbit = 256 byte eeprom
	) helper (
		.clk(clk),
		.slave_addr({4'hb, ADDR_PINS[2:0], 1'h1}),

		.open(open),
		.ready(ready),
		.select(select),
		.addr(8'h9a),		//constant address for mac addr
		.we(1'b0),			//always reading, can't write to the mac
		.burst_len(8'd6),	//always read entire mac addr
		.close(close),
		.rdata_valid(rdata_valid),
		.rdata(rdata),
		.err(err),
		.wdata_valid(1'b0),	//can't write
		.wdata(8'h0),
		.need_wdata(),
		.burst_done(burst_done),

		.request(driver_req),
		.done(driver_done),
		.ack(driver_ack),
		.cin(driver_cin),
		.cout(driver_cout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	enum logic[1:0]
	{
		STATE_IDLE 		= 2'h0,
		STATE_WAIT_ACK	= 2'h1,
		STATE_READ		= 2'h2,
		STATE_DONE		= 2'h3
	} state = STATE_IDLE;

	always_ff @(posedge clk) begin
		open	<= 0;
		select	<= 0;
		close	<= 0;

		case(state)

			STATE_IDLE: begin
				open	<= 1;
				state	<= STATE_WAIT_ACK;
			end	//end STATE_IDLE

			STATE_WAIT_ACK: begin
				if(ready && !open) begin
					select	<= 1;
					state	<= STATE_READ;
				end
			end	//end STATE_WAIT_ACK

			STATE_READ: begin
				if(rdata_valid)
					mac		<= { mac[39:0], rdata };

				if(burst_done) begin
					state	<= STATE_DONE;
					done	<= 1;
				end
			end	//end STATE_READ

			STATE_DONE: begin
				//hang forever
			end //end STATE_DONE

		endcase

	end

endmodule
