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

`include "I2CTransceiver.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief I2C master-mode transceiver

	Note that this core uses active-high ACKs, not active-low as seen on the wire!
 */
module I2CTransceiver(

	//Clocking
	input wire clk,
	input wire[15:0] clkdiv,

	//I2C pads
	output reg	i2c_scl = 1,
	inout wire	i2c_sda,

	//Control/data lines
	input wire	i2c_in_t	cin,
	output		i2c_out_t	cout

    );

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialization

	initial begin
		cout.tx_ack = 0;
		cout.rx_rdy = 0;
		cout.rx_out = 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////
	// I2C tristate processing

	reg sda_out = 0;
	reg sda_tx = 0;

	assign i2c_sda = sda_tx ? sda_out : 1'bz;

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Transceiver logic

	logic[14:0] clk_count = 0;

	enum logic[3:0]
	{
		STATE_IDLE		= 4'h0,
		STATE_START		= 4'h1,
		STATE_TX		= 4'h2,
		STATE_TX_2		= 4'h3,
		STATE_READ_ACK	= 4'h4,
		STATE_RX		= 4'h5,
		STATE_SEND_ACK	= 4'h6,
		STATE_STOP		= 4'h7,
		STATE_STOP_2	= 4'h8,
		STATE_RESTART	= 4'h9,
		STATE_RESTART_2	= 4'ha
	} state = STATE_IDLE;

	assign cout.busy =
		(state != STATE_IDLE) |
		cin.start_en |
		cin.restart_en |
		cin.stop_en |
		cin.tx_en |
		cin.rx_en;

	logic[7:0]	tx_buf 		= 0;
	logic		rx_ackbuf 	= 0;
	logic[3:0]	bitcount	= 0;
	logic[7:0]	rx_buf		= 0;

	always_ff @(posedge clk) begin

		cout.rx_rdy <= 0;

		case(state)

			//Ready to do stuff
			STATE_IDLE: begin

				//Send start bit
				//Data and clock should be high
				if(cin.start_en) begin
					sda_tx 		<= 1;
					sda_out		<= 0;
					clk_count	<= 0;
					state		<= STATE_START;
				end

				//Send a byte of data
				//Clock should be low at this point.
				else if(cin.tx_en) begin
					tx_buf		<= cin.tx_data;
					clk_count	<= 0;
					bitcount	<= 0;
					state		<= STATE_TX;
					i2c_scl		<= 0;
				end

				//Read a byte of data
				//Clock should be low at this point.
				else if(cin.rx_en) begin
					clk_count	<= 0;
					bitcount	<= 0;
					state		<= STATE_RX;
					i2c_scl		<= 0;
					rx_ackbuf	<= cin.rx_ack;

					rx_buf		<= 8'h00;
				end

				//Send stop bit
				//Clock should be low at this point
				else if(cin.stop_en) begin

					//Make data low
					sda_tx		<= 1;
					sda_out		<= 0;
					i2c_scl		<= 0;

					clk_count	<= 0;
					state		<= STATE_STOP;
				end

				//Send restart bit
				//Clock should be low at this point
				else if(cin.restart_en) begin

					//Make data high
					sda_tx		<= 1;
					sda_out		<= 1;
					i2c_scl		<= 0;
					clk_count	<= 0;
					state		<= STATE_RESTART;

				end

			end

			//Sending start bit
			STATE_START: begin
				//Keep track of time, clock goes low at clkdiv/2
				clk_count		<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count)
					i2c_scl		<= 0;

				if(clk_count == clkdiv)
					state		<= STATE_IDLE;
			end

			//Sending data
			STATE_TX: begin

				sda_tx <= 1;

				//Send the next data bit
				if(clk_count == 0) begin
					sda_out <= tx_buf[7];
					tx_buf	<= {tx_buf[6:0], 1'b0};
				end

				//Keep track of time, clock goes high at clkdiv/2
				clk_count	<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count)
					i2c_scl <= 1;

				//End of this bit? Go on to the next
				if(clkdiv[14:0] == clk_count) begin
					i2c_scl		<= 0;
					clk_count	<= 0;
					bitcount	<= bitcount + 4'd1;

					//stop at end of byte
					if(bitcount == 7)
						state 	<= STATE_READ_ACK;
				end

			end

			//Read the acknowledgement bit
			STATE_READ_ACK: begin

				//set SDA to read mode
				sda_tx			<= 0;

				//Keep track of time, clock goes high at clkdiv/2
				//Read ACK on rising edge
				clk_count		<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count) begin
					i2c_scl		<= 1;
					cout.tx_ack <= !i2c_sda;
				end

				//End of this bit? Go on to the next
				if(clkdiv[14:0] == clk_count) begin
					i2c_scl		<= 0;
					clk_count	<= 0;

					state		<= STATE_IDLE;
				end

			end

			//Read a byte of data
			STATE_RX: begin

				//read mode
				sda_tx 			<= 0;

				//Keep track of time, clock goes high at clkdiv/2
				//Read data on rising edge (high order bit is sent first)
				clk_count		<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count) begin
					i2c_scl		<= 1;

					rx_buf		<= {rx_buf[6:0], i2c_sda};
				end

				//End of this bit? Go on to the next
				if(clkdiv[14:0] == clk_count) begin

					i2c_scl		<= 0;
					clk_count	<= 0;
					bitcount	<= bitcount + 4'd1;

					//stop at end of byte
					if(bitcount == 7)
						state	<= STATE_SEND_ACK;
				end

			end

			//Send the ACK
			STATE_SEND_ACK: begin

				//set SDA to write mode and send the ack (invert ack to nack)
				sda_tx			<= 1;
				sda_out			<= !rx_ackbuf;

				//Keep track of time, clock goes high at clkdiv/2
				clk_count		<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count)
					i2c_scl		<= 1;

				//End of this bit? Finished
				if(clkdiv[14:0] == clk_count) begin
					i2c_scl		<= 0;
					clk_count	<= 0;

					cout.rx_rdy	<= 1;
					cout.rx_out	<= rx_buf;

					state		<= STATE_IDLE;
				end

			end

			//Sending stop bit
			STATE_STOP: begin

				//Keep track of time, clock goes high at clkdiv/2
				clk_count 		<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count)
					i2c_scl		<= 1;

				//End of the stop bit? Let SDA float high
				if(clkdiv[14:0] == clk_count) begin
					sda_tx		<= 0;
					clk_count	<= 0;

					state		<= STATE_STOP_2;
				end

			end

			//Wait one additional bit period after the stop bit
			STATE_STOP_2: begin
				clk_count		<= clk_count + 15'd1;

				if(clk_count == clkdiv)
					state 		<= STATE_IDLE;
			end

			//Sending repeated start bit
			STATE_RESTART: begin

				//Keep track of time, clock goes high at clkdiv/2
				clk_count		<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count)
					i2c_scl		<= 1;

				//End of the start bit? SDA goes low
				if(clkdiv[14:0] == clk_count) begin
					sda_out		<= 0;
					clk_count	<= 0;
					state		<= STATE_RESTART_2;
				end

			end

			STATE_RESTART_2: begin

				//Keep track of time, clock goes low at clkdiv/2
				clk_count		<= clk_count + 15'd1;
				if(clkdiv[15:1] == clk_count)
					i2c_scl		<= 0;

				//End of restart bit? Go idle
				if(clkdiv[14:0] == clk_count)
					state		<= STATE_IDLE;

			end

		endcase

	end

endmodule
