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
	@brief Controller for a Microchip MCP4726 I2C DAC.

	Also works for the MCP4716.
	The 10-bit DAC code should be padded at right with two zero bits.
 */
module MCP4726 #(
	parameter SLAVE_ADDR 		= 8'hc0
)(
	input wire	clk,

	input wire				update_en,
	input wire[11:0]		dac_code,
	output logic			done			= 0,

	output logic			driver_request	= 0,
	output logic			driver_done		= 0,
	input wire				driver_ack,
	output i2c_in_t			driver_cin		= {$bits(i2c_in_t){1'b0}},
	input wire i2c_out_t	driver_cout
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	enum logic[3:0]
	{
		STATE_IDLE		= 4'h0,
		STATE_UPDATE_0	= 4'h1,
		STATE_UPDATE_1	= 4'h2,
		STATE_UPDATE_2	= 4'h3,
		STATE_UPDATE_3	= 4'h4,
		STATE_UPDATE_4	= 4'h5,
		STATE_UPDATE_5	= 4'h6
	} state = STATE_IDLE;

	always_ff @(posedge clk) begin

		done					<= 0;

		driver_done				<= 0;
		driver_request			<= 0;
		driver_cin.tx_en		<= 0;
		driver_cin.rx_en		<= 0;
		driver_cin.rx_ack		<= 1;
		driver_cin.start_en		<= 0;
		driver_cin.restart_en	<= 0;
		driver_cin.stop_en		<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for stuff to happen

			STATE_IDLE: begin
				if(update_en) begin
					driver_request	<= 1;
					state			<= STATE_UPDATE_0;
				end
			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// UPDATE - load new DAC code

			STATE_UPDATE_0: begin
				if(driver_ack) begin
					driver_cin.start_en	<= 1;
					state				<= STATE_UPDATE_1;
				end
			end	//end STATE_UPDATE_0

			STATE_UPDATE_1: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= SLAVE_ADDR;
					state				<= STATE_UPDATE_2;
				end
			end	//end STATE_UPDATE_1

			STATE_UPDATE_2: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= {2'h0, 2'h0, dac_code[11:8]};
					state				<= STATE_UPDATE_3;
				end
			end	//end STATE_UPDATE_2

			STATE_UPDATE_3: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= dac_code[7:0];
					state				<= STATE_UPDATE_4;
				end
			end	//end STATE_UPDATE_3

			STATE_UPDATE_4: begin
				if(!driver_cout.busy) begin
					driver_cin.stop_en	<= 1;
					state				<= STATE_UPDATE_5;
				end
			end	//end STATE_UPDATE_4

			STATE_UPDATE_5: begin
				if(!driver_cout.busy) begin
					driver_done			<= 1;
					done				<= 1;
					state				<= STATE_IDLE;
				end
			end	//end STATE_UPDATE_5

		endcase

	end

endmodule
