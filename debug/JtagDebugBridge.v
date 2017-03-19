`default_nettype none
`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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

module JtagDebugBridge(
	input wire clk,
	output reg[3:0] led = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Buffer the main system clock

	wire clk_bufg;
	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("NO")
	) sysclk_clkbuf (
		.clkin(clk),
		.clkout(clk_bufg),
		.ce(1'b1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The TAP interface for discovery (DEBUG_IDCODE register)

	//See https://github.com/azonenberg/jtaghal/wiki/FPGA-debug for ID table
	JtagUserIdentifier #(
		.IDCODE_VID(24'h42445a),	//"ADZ"
		.IDCODE_PID(8'h00)			//Antikernel NoC interface
	) id ();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The TAP interface for Antikernel debug

	reg[31:0]	tx_shreg = 0;
	reg[31:0]	rx_shreg = 0;

	wire		tap_active;
	wire		tap_shift;
	wire		tap_clear;
	wire		tap_tck_raw;
	wire		tap_tck_bufh;
	wire		tap_tdi;

	//The TAP itself
	JtagTAP #(
		.USER_INSTRUCTION(2)
	) tap_tap (
		.instruction_active(tap_active),
		.state_capture_dr(tap_clear),
		.state_reset(),
		.state_runtest(),
		.state_shift_dr(tap_shift),
		.state_update_dr(),
		.tck(tap_tck_raw),
		.tck_gated(),
		.tms(),
		.tdi(tap_tdi),
		.tdo(tx_shreg[0])
	);

	//Buffer the clock b/c ISE is derpy and often won't instantiate a buffer (woo skew!)
	//TODO: according to comments in older code BUFHs here sometimes won't work in spartan6?
	ClockBuffer #(
		.TYPE("LOCAL"),
		.CE("NO")
	) tap_tck_clkbuf (
		.clkin(tap_tck_raw),
		.clkout(tap_tck_bufh),
		.ce(1'b1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Convert from a stream of bits to a stream of 32-bit words

	reg[4:0] phase = 0;
	always @(posedge tap_tck_bufh) begin

		//Use the capture-dr -> shift-dr transition to word align our data
		if(tap_clear)
			phase	<= 0;

		//Nothign fancy happening, just go to the next bit
		else if(tap_shift)
			phase	<= phase + 1'h1;

	end

	//TX data shift register
	reg[31:0]	tx_data = 0;
	always @(posedge tap_tck_bufh) begin

		if(!tap_active) begin
		end

		//Load new outbound data
		else if(tap_clear || (tap_shift && phase == 31) )
			tx_shreg	<= tx_data;

		//Send stuff
		else if(tap_shift)
			tx_shreg	<= { 1'b0, tx_shreg[31:1] };

	end

	//RX data shift register
	reg			rx_valid = 0;

	always @(posedge tap_tck_bufh) begin
		rx_valid	<= 0;

		if(!tap_active) begin
		end

		//Receive stuff
		else if(tap_shift) begin
			rx_shreg		<= { tap_tdi, rx_shreg[31:1] };

			if(phase == 31)
				rx_valid	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX state machine

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX CRC calculation

	reg			rx_crc_reset		= 0;
	reg			rx_crc_en			= 0;
	wire[7:0]	rx_crc_dout;

	CRC8_ATM rx_crc(
		.clk(tap_tck_bufh),
		.reset(rx_crc_reset),
		.update(rx_valid),
		.din(rx_shreg),
		.crc(),
		.crc_first24(rx_crc_dout)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX state machine

	localparam RX_STATE_IDLE		= 4'h0;
	localparam RX_STATE_HEADER_0	= 4'h1;
	localparam RX_STATE_HEADER_1	= 4'h2;

	reg[3:0]	rx_state	= RX_STATE_IDLE;
	reg[7:0]	rx_expected_crc = 0;

	always @(posedge tap_tck_bufh) begin

		rx_crc_reset		<= 0;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Waiting for a packet to come in

			RX_STATE_IDLE: begin

				//FIRST header word is here!
				//See https://github.com/azonenberg/antikernel/wiki/JTAG-Tunnel
				//31	= ack
				//30	= nak
				//29:20 = seq
				//19:10 = credits
				//9:0	= ack_seq
				if(rx_valid) begin
					//TODO: process ack, seq, etc
					rx_state		<= RX_STATE_HEADER_0;
				end

				//We always have at least one clock in idle before a new word comes in
				//Use that time to clear the CRC for the new packet
				else begin
					rx_crc_reset	<= 1;
				end

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// New message came in.

			//Save header fields
			RX_STATE_HEADER_0: begin

				//SECOND header word is here!
				//31	= payload present
				//30	= RPC
				//29	= DMA
				//28:18	= reserved
				//17:8	= payload length
				//7:0	= header checksum
				if(rx_valid) begin
					rx_expected_crc	<= rx_shreg[7:0];
					rx_state		<= RX_STATE_HEADER_1;
				end

			end	//end RX_STATE_HEADER_0

			//One clock after the header word was fully processed. At this point the CRC is done.
			RX_STATE_HEADER_1: begin
				//TODO: process messages that have payloads

				tx_data		<= { 24'h00, rx_crc_dout };
				led[2]		<= (rx_crc_dout == rx_expected_crc);

				rx_state	<= RX_STATE_IDLE;

			end	//end RX_STATE_HEADER_1

		endcase

		//Reset everything when the TAP reinitializes
		if(!tap_active || tap_clear)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TODO: NoC transceivers

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug glue

	always @(posedge clk) begin
		led[3] <= 1;
	end


endmodule
