`default_nettype none
`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
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
	@brief	AD5065 16-bit SPI DAC, as used in the Xilinx AMS101 demo board
 */
module AD5065_SPI(
	input wire			clk,			//nominal 125 MHz for now
	input wire[15:0]	clkdiv,			//need to divide clk to <50 MHz

	output logic		dac_load_n		= 0,
	output wire			dac_clk,
	output logic		dac_sync_n		= 1,
	output wire			dac_din,

	input wire			update,
	input wire			channel,		//0 = channel A, 1 = channel B
	input wire[15:0]	value,
	output logic		done			= 0

);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SPI interface

	logic		dac_shift_en	= 0;
	wire		dac_shift_done;

	logic[7:0]	dac_tx_data	= 0;

	SPITransceiver #(
		.SAMPLE_EDGE("FALLING"),
		.CHANGE_ON_DONE(1)
	) spi_txvr (
		.clk(clk),
		.clkdiv(clkdiv),

		.spi_sck(dac_clk),
		.spi_mosi(dac_din),
		.spi_miso(1'b0),

		.shift_en(dac_shift_en),
		.shift_done(dac_shift_done),
		.tx_data(dac_tx_data),
		.rx_data()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control state machine

	enum logic[3:0]
	{
		DAC_STATE_IDLE		= 4'h0,
		DAC_STATE_SELECT	= 4'h1,
		DAC_STATE_UPDATE_0	= 4'h2,
		DAC_STATE_UPDATE_1	= 4'h3,
		DAC_STATE_UPDATE_2	= 4'h4,
		DAC_STATE_UPDATE_3	= 4'h5,
		DAC_STATE_UPDATE_4	= 4'h6,
		DAC_STATE_DESELECT	= 4'h7
	} dac_state = DAC_STATE_IDLE;

	/*
		Clock rate can be up to 50 MHz

		Wire format:
			4 dontcare bits, 4 command bits
			4 register bits, high 4 of DAC code
			Middle 8 of DAC code
			Low 4 of DAC code, 4 dontcare bits

		SYNC must be high for >1.9 us between updates
	 */

	logic[3:0] dcount = 0;

	always_ff @(posedge clk) begin

		dac_shift_en	<= 0;

		done			<= 0;

		case(dac_state)

			//Update the DAC
			DAC_STATE_IDLE: begin

				if(update) begin
					dac_sync_n	<= 0;
					dac_state	<= DAC_STATE_SELECT;
				end

			end	//end DAC_STATE_IDLE

			//SYNC must be low for at least 16.5 ns (a bit over 2 clocks at 125 MHz)
			//before we do anything
			DAC_STATE_SELECT: begin
				dcount	<= dcount + 1'h1;
				if(dcount == 2)
					dac_state	<= DAC_STATE_UPDATE_0;
			end	//end DAC_STATE_SELECT

			//4 dontcare bits, 4 command bits
			DAC_STATE_UPDATE_0: begin
				dac_shift_en	<= 1;
				dac_tx_data		<= 8'h03;
				dac_state		<= DAC_STATE_UPDATE_1;
			end	//end DAC_STATE_UPDATE_0

			//4 register bits, high bits of DAC code
			DAC_STATE_UPDATE_1: begin
				if(dac_shift_done) begin
					dac_shift_en	<= 1;
					if(channel == 0)
						dac_tx_data	<= { 4'h0, value[15:12] };
					else
						dac_tx_data	<= { 4'h3, value[15:12] };

					dac_state		<= DAC_STATE_UPDATE_2;
				end
			end	//end DAC_STATE_UPDATE_1

			//Mid 8 bits of DAC code
			DAC_STATE_UPDATE_2: begin
				if(dac_shift_done) begin
					dac_shift_en	<= 1;
					dac_tx_data		<= value[11:4];
					dac_state		<= DAC_STATE_UPDATE_3;
				end
			end	//end DAC_STATE_UPDATE_2

			//Low 4 bits of DAC code, 4 dontcare bits
			DAC_STATE_UPDATE_3: begin
				if(dac_shift_done) begin
					dac_shift_en	<= 1;
					dac_tx_data		<= { value[3:0], 4'h0 };
					dac_state		<= DAC_STATE_UPDATE_4;
				end
			end	//end DAC_STATE_UPDATE_3

			DAC_STATE_UPDATE_4: begin
				if(dac_shift_done) begin
					dcount			<= 0;
					dac_state		<= DAC_STATE_DESELECT;
				end
			end	//end DAC_STATE_UPDATE_4

			DAC_STATE_DESELECT: begin
				dcount	<= dcount + 1'h1;

				if(dcount == 2) begin
					dac_sync_n	<= 1;
					dcount		<= dcount + 1'h1;
					done		<= 1;
					dac_state	<= DAC_STATE_IDLE;
				end
			end	//end DAC_STATE_DESELECT

		endcase

	end

endmodule
