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
	@brief Controller for a TI TCA6424A I2C I/O expander

	TODO: add support for polarity inversion?
	Inverters are basically free in FPGA so not sure if we need them.

	TODO: implement read support (currently TRIS only)
	TODO: implement byte-wide read/write vs blasting entire port
 */
module TCA6424A #(
	parameter SLAVE_ADDR 	= 8'h44
)(
	input wire				clk,

	input wire[23:0]		tx_data,		//data to be sent out
	input wire[23:0]		tx_tris,		//1=input, 0=output
	output logic[23:0]		rx_data	= 0,	//data read from pins

	input wire				write_en,		//assert for one cycle to write output values
	input wire				tris_en,		//assert for one cycle to write tri-state values
	input wire				read_en,		//assert for one cycle to read input values

	output logic			done	= 0,	//asserted for one cycle when an operation has completed

	//I2C controller lines
	output logic			driver_request	= 0,
	output logic			driver_done		= 0,
	input wire				driver_ack,
	output i2c_in_t			driver_cin = {$bits(i2c_in_t){1'b0}},
	input wire i2c_out_t	driver_cout
);

	enum logic[4:0]
	{
		STATE_IDLE		= 5'h00,

		STATE_WRITE_0	= 5'h01,
		STATE_WRITE_1	= 5'h02,
		STATE_WRITE_2	= 5'h03,
		STATE_WRITE_3	= 5'h04,
		STATE_WRITE_4	= 5'h05,
		STATE_WRITE_5	= 5'h06,
		STATE_WRITE_6	= 5'h07,
		STATE_WRITE_7	= 5'h08,

		STATE_TRIS_0	= 5'h09,
		STATE_TRIS_1	= 5'h0a,
		STATE_TRIS_2	= 5'h0b,
		STATE_TRIS_3	= 5'h0c,
		STATE_TRIS_4	= 5'h0d,
		STATE_TRIS_5	= 5'h0e,
		STATE_TRIS_6	= 5'h0f,
		STATE_TRIS_7	= 5'h10

	} state = STATE_IDLE;

	//auto increment enabled
	localparam REG_OUT_0 	= 8'h84;
	localparam REG_TRIS_0	= 8'h8c;

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

				if(write_en) begin
					driver_request	<= 1;
					state			<= STATE_WRITE_0;
				end

				else if(tris_en) begin
					driver_request	<= 1;
					state			<= STATE_TRIS_0;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// WRITE - update WRITE data register

			STATE_WRITE_0: begin
				if(driver_ack) begin
					driver_cin.start_en	<= 1;
					state				<= STATE_WRITE_1;
				end
			end	//end STATE_WRITE_0

			STATE_WRITE_1: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= SLAVE_ADDR;
					state				<= STATE_WRITE_2;
				end
			end	//end STATE_WRITE_1

			STATE_WRITE_2: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= REG_OUT_0;
					state				<= STATE_WRITE_3;
				end
			end	//end STATE_WRITE_2

			STATE_WRITE_3: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= tx_data[7:0];
					state				<= STATE_WRITE_4;
				end
			end	//end STATE_WRITE_3

			STATE_WRITE_4: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= tx_data[15:8];
					state				<= STATE_WRITE_5;
				end
			end	//end STATE_WRITE_4

			STATE_WRITE_5: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= tx_data[23:16];
					state				<= STATE_WRITE_6;
				end
			end	//end STATE_WRITE_5

			STATE_WRITE_6: begin
				if(!driver_cout.busy) begin
					driver_cin.stop_en	<= 1;
					state				<= STATE_WRITE_7;
				end
			end	//end STATE_WRITE_6

			STATE_WRITE_7: begin
				if(!driver_cout.busy) begin
					driver_done			<= 1;
					done				<= 1;
					state				<= STATE_IDLE;
				end
			end	//end STATE_WRITE_7

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// TRIS - update tri-state data register

			STATE_TRIS_0: begin
				if(driver_ack) begin
					driver_cin.start_en	<= 1;
					state				<= STATE_TRIS_1;
				end
			end	//end STATE_TRIS_0

			STATE_TRIS_1: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= SLAVE_ADDR;
					state				<= STATE_TRIS_2;
				end
			end	//end STATE_TRIS_1

			STATE_TRIS_2: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= REG_TRIS_0;
					state				<= STATE_TRIS_3;
				end
			end	//end STATE_TRIS_2

			STATE_TRIS_3: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= tx_tris[7:0];
					state				<= STATE_TRIS_4;
				end
			end	//end STATE_TRIS_3

			STATE_TRIS_4: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= tx_tris[15:8];
					state				<= STATE_TRIS_5;
				end
			end	//end STATE_TRIS_4

			STATE_TRIS_5: begin
				if(!driver_cout.busy) begin
					driver_cin.tx_en	<= 1;
					driver_cin.tx_data	<= tx_tris[23:16];
					state				<= STATE_TRIS_6;
				end
			end	//end STATE_TRIS_5

			STATE_TRIS_6: begin
				if(!driver_cout.busy) begin
					driver_cin.stop_en	<= 1;
					state				<= STATE_TRIS_7;
				end
			end	//end STATE_TRIS_6

			STATE_TRIS_7: begin
				if(!driver_cout.busy) begin
					driver_done			<= 1;
					done				<= 1;
					state				<= STATE_IDLE;
				end
			end	//end STATE_TRIS_7

		endcase

	end

endmodule
