`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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
	@brief A single 8-bit, no parity, 1 stop bit serial port.

	This is the raw UART and does not have a FIFO etc. The parent module must read
	data the cycle after it becomes available and process or store it.

	@param clk		Input clock signal
	@param clkdiv	Clock divisor for baud rate generator (115200 baud = 174 @ 20 MHz)

	@param tx		Outbound serial data line
	@param tx_data	8-bit wide bus containing data to be transmitted
	@param tx_en	Bring high for 1 clk when data is ready to be transmitted
	@param txactive	Indicates if transmitter is busy

	@param rx		Inbound serial data line
	@param rx_data	8-bit wide bus containing data recieved
	@param rx_en	Goes high for one clk when valid data is present on rx_data
	@param rxactive	Indicates if reciever is busy
 */
module UART(
	input wire			clk,
	input wire[15:0]	clkdiv,

	input wire			rx,
	output logic		rxactive	= 0,
	output logic[7:0]	rx_data		= 0,
	output logic		rx_en		= 0,

	output logic		tx			= 1,
	input wire[7:0]		tx_data,
	input wire			tx_en,
	output logic		txactive	= 0,
	output logic		tx_done		= 0
	);

	//1/4 of the clock divisor (for 90 degree phase offset)
	wire[13:0] clkdiv_offset;
	assign clkdiv_offset = clkdiv[15:2];

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Metastability protection on input

	wire	rx_sync;

	ThreeStageSynchronizer #(
		.INIT(1),
		.IN_REG(0)
	) sync_rx(
		.clk_in(),
		.din(rx),
		.clk_out(clk),
		.dout(rx_sync)
	);

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Receiver

	logic[15:0]	rxbrg		= 0;
	logic[4:0]	rxbitcount	= 0;
	logic[7:0]	rxbuf		= 0;

	always @(posedge clk) begin

		//Clear data from output after one clock
		if(rx_en) begin
			rx_en		<= 0;
			rx_data		<= 0;
		end

		//If not currently recieving, look for falling edge on RX (start bit).
		if(!rxactive) begin
			if(rx_sync == 0) begin

				//Falling edge, start receiving after 1.25 bit period
				//We want to sample 90 degrees out of phase with the original signal to get nice stable values
				rxactive	<= 1;
				rxbrg 		<= clkdiv_offset + clkdiv;
				rxbitcount	<= 0;
			end
		end

		//Currently recieving
		else begin
			rxbrg	<= rxbrg - 16'd1;

			//Time to sample a new bit
			if(rxbrg == 0) begin

				//If we are on bits 0 through 7 (not the stop bit)
				//read the bit into the rxbuf and bump the bit count, then reset the baud generator
				if(rxbitcount < 8) begin
					rxbuf		<= { rx_sync, rxbuf[7:1] };
					rxbitcount	<= rxbitcount + 5'd1;
					rxbrg		<= clkdiv;
				end

				//Stop bit
				else begin

					//We're done reading
					rxbitcount	<= 0;
					rxactive	<= 0;

					//Data is ready
					rx_data		<= rxbuf;
					rx_en		<= 1;
				end

			end

		end

	end

	///////////////////////////////////////////////////////////////////////////////////////////////
	//Transmitter

	logic[15:0]	txbrg		= 0;
	logic[7:0] 	txbuf		= 0;
	logic[3:0]	txbitcount	= 0;

	always_ff @(posedge clk) begin

		tx_done	<= 0;

		//Time to start sending a new byte?
		if(tx_en) begin

			//Already transmitting? Drop the byte, nothing we can do here.
			//External FIFO required to handle stuff
			if(txactive) begin
			end

			//Nope, set up a transmission
			else begin
				txbuf		<= tx_data;
				txactive	<= 1;
				txbitcount	<= 0;

				//Send the start bit immediately
				tx			<= 0;
				txbrg		<= clkdiv;
			end

		end

		//Currently transmitting?
		if(txactive) begin

			//Done with this bit?
			if(txbrg == 0) begin

				//Are we still sending normal data bits?
				//Send the next data bit (LSB first)
				if(txbitcount < 8) begin
					txbitcount	<= txbitcount + 4'd1;
					tx			<= txbuf[0];
					txbuf		<= {1'b0, txbuf[7:1]};
					txbrg		<= clkdiv;
				end

				//Time to send the stop bit?
				//Send it
				else if(txbitcount == 8) begin
					txbitcount	<= 9;
					tx			<= 1;
					txbrg		<= clkdiv;
				end

				//Done sending? Reset stuff
				else if(txbitcount == 9) begin
					txbitcount	<= 0;
					txactive	<= 0;
					tx_done		<= 1;
				end

			end

			//Nope, just keep count
			else
				txbrg			<= txbrg - 16'd1;

		end

	end

endmodule
