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
	wire		tap_reset;

	//The TAP itself
	JtagTAP #(
		.USER_INSTRUCTION(2)
	) tap_tap (
		.instruction_active(tap_active),
		.state_capture_dr(tap_clear),
		.state_reset(tap_reset),
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
	reg[31:0]	tx_data			= 0;
	reg			tx_data_needed	= 0;
	always @(posedge tap_tck_bufh) begin

		tx_data_needed		<= 0;

		if(!tap_active) begin
		end

		//Load the next word of data
		else if(tap_clear || (tap_shift && phase == 31) )
			tx_shreg		<= tx_data;

		//Send stuff
		else if(tap_shift)
			tx_shreg		<= { 1'b0, tx_shreg[31:1] };

		//If we are almost done with the current word, ask for another one
		//27	prepare to assert
		//28	TX state machine writes tx_crc_din
		//29	TX state machine writes tx_crc_din_ff
		//30	TX state machine writes tx_data
		//31	we have data ready
		if(tap_shift && phase == 27)
			tx_data_needed	<= 1;

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
	// RX CRC calculation

	reg			rx_crc_reset		= 0;
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
	localparam RX_STATE_RESET_WAIT	= 4'h3;

	reg[3:0]	rx_state			= RX_STATE_IDLE;
	reg[7:0]	rx_expected_crc 	= 0;

	reg			rx_failed			= 1;

	reg[9:0]	tx_seq_num			= 0;
	reg[9:0]	tx_ack_num			= 0;

	always @(posedge tap_tck_bufh) begin

		rx_crc_reset		<= 0;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Waiting for a packet to come in

			RX_STATE_IDLE: begin

				rx_failed			<= 0;

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
			// New message came in. Process it

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

				//If the header checksum is bad, we have a problem.
				//Since we don't know how long this frame was (because the length might be corrupted),
				//we can't skip it and go to the next one.
				//Eventually we can try to recover by looking for the next frame and seeing when we get a good CRC.
				//For now, take the easy way out and just die.
				if(rx_crc_dout != rx_expected_crc) begin
					rx_state	<= RX_STATE_RESET_WAIT;
					led[2]		<= 1;
					rx_failed	<= 1;
				end

				//All good, ready for the next frame
				else
					rx_state	<= RX_STATE_IDLE;

			end	//end RX_STATE_HEADER_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Sit around and wait until we get reset

			RX_STATE_RESET_WAIT: begin
			end	//end RX_STATE_RESET_WAIT

		endcase

		//Reset everything when the TAP reinitializes
		if(!tap_active || tap_clear)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX CRC calculation

	reg[31:0]	tx_crc_din	 	= 0;
	reg[31:0]	tx_crc_din_ff	= 0;
	reg			tx_crc_reset	= 0;
	reg			tx_crc_update	= 0;
	wire[7:0]	tx_crc_dout;

	CRC8_ATM tx_crc(
		.clk(tap_tck_bufh),
		.reset(tx_crc_reset),
		.update(tx_crc_update),
		.din(tx_crc_din),
		.crc(),
		.crc_first24(tx_crc_dout)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX state machine

	localparam TX_STATE_RESET		= 4'h0;
	localparam TX_STATE_IDLE_0		= 4'h1;
	localparam TX_STATE_IDLE_1		= 4'h2;
	localparam TX_STATE_DOWN_0		= 4'h3;
	localparam TX_STATE_DOWN_1		= 4'h4;

	reg[3:0]	tx_state			= TX_STATE_IDLE_0;
	reg			tx_header_done		= 0;
	reg			tx_header_done_adv	= 0;

	always @(posedge tap_tck_bufh) begin

		//Clear flags
		tx_crc_reset		<= 0;
		tx_crc_update		<= 0;
		tx_header_done_adv	<= 0;

		//Save whatever we fed to the CRC (this is going to get sent out in a bit).
		//If we just finished generating the packet header, it's CRCing this cycle so remember that.
		tx_crc_din_ff		<= tx_crc_din;
		tx_header_done		<= tx_header_done_adv;

		//If we have the header checksum available, munge it into the outbound word.
		//Otherwise, send the word as-is
		if(tx_header_done)
			tx_data			<= { tx_crc_din_ff[31:8], tx_crc_dout };
		else
			tx_data			<= tx_crc_din_ff;

		//Main state machine
		case(tx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// RESET - send a single idle frame and reset some stuff

			//Send initial idle frame when we reset the link
			TX_STATE_RESET: begin

				//Write this immediately (without waiting for data-needed flag)
				//because as soon as we enter the capture state, this data has to be available!
				tx_crc_din		<=
				{
					1'b0,		//ACK flag, always 0 since we haven't seen any packets yet
					1'b0,		//NAK flag, always 0 since we haven't seen any packets yet
					10'h0,		//Outbound sequence number (we start from zero when the link goes up)
					10'h3FF,	//Inbound credit counter (max, since we have an empty buffer so far)
								//TODO
					10'h0		//ACK sequence number (ignored since ACK/NAK aren't set)
				};

				//We sent this packet so bump the sequence number for the next one
				tx_seq_num		<= 1;

				//Start a new checksum
				tx_crc_update	<= 1;
				tx_crc_reset	<= 1;

				//Move on immediately to the second half of the packet
				tx_state		<= TX_STATE_IDLE_1;

			end	//end TX_STATE_RESET

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - send idle frames until we have something else to do

			//Send first half of idle frame
			TX_STATE_IDLE_0: begin

				led[0]				<= 1;
				led[1]				<= 0;

				if(tx_data_needed) begin

					//TODO: check for tx data in fifo etc
					tx_crc_din		<=
					{
						1'b0,		//ACK flag, always 0 for now
						1'b0,		//NAK flag, always 0 for now
						tx_seq_num,	//Outbound sequence number
						10'h3FF,	//Inbound credit counter (always max for now)
						tx_ack_num	//ACK sequence number
					};

					//Start a new checksum
					tx_crc_update	<= 1;
					tx_crc_reset	<= 1;

					//We sent this packet so bump the sequence number for the next one
					tx_seq_num		<= tx_seq_num + 1'h1;

					//Prepare to send second word
					tx_state		<= TX_STATE_IDLE_1;

				end

			end	//end TX_STATE_IDLE_0

			//Send second half of idle frame
			TX_STATE_IDLE_1: begin

				if(tx_data_needed) begin

					//Format header
					tx_crc_din		<=
					{
						1'b0,		//No payload
						1'b0,		//No RPC payload
						1'b0,		//No DMA payload
						11'h0,		//Reserved
						10'h0,		//Length
						8'h0		//Placeholder for header checksum
					};

					//Checksum the data
					tx_crc_update		<= 1;
					tx_header_done_adv	<= 1;

					//If something bad happened, wait for link reset
					if(rx_failed)
						tx_state			<= TX_STATE_DOWN_0;

					else
						tx_state			<= TX_STATE_IDLE_0;
				end

			end	//end TX_STATE_IDLE_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// DOWN - link went down due to a loss of sync, send NAK frames forever

			//Send first half of frame
			TX_STATE_DOWN_0: begin

				led[0]				<= 0;
				led[1]				<= 1;

				if(tx_data_needed) begin

					tx_crc_din		<=
					{
						1'b0,		//ACK flag, always 0 since the link is down
						1'b1,		//NAK flag, always 1 since the link is down
						tx_seq_num,	//Outbound sequence number
						10'h3FF,	//Inbound credit counter, doesn't matter what it is since the link is down
						tx_ack_num	//ACK sequence number
					};

					//Start a new checksum
					tx_crc_update	<= 1;
					tx_crc_reset	<= 1;

					//We sent this packet so bump the sequence number for the next one
					tx_seq_num		<= tx_seq_num + 1'h1;

					//Prepare to send second word
					tx_state		<= TX_STATE_DOWN_1;

				end

			end	//end TX_STATE_DOWN_0

			//Send second half of frame
			TX_STATE_DOWN_1: begin

				if(tx_data_needed) begin

					//Format header
					tx_crc_din		<=
					{
						1'b0,		//No payload
						1'b0,		//No RPC payload
						1'b0,		//No DMA payload
						11'h0,		//Reserved
						10'h0,		//Length
						8'h0		//Placeholder for header checksum
					};

					//Checksum the data
					tx_crc_update		<= 1;
					tx_header_done_adv	<= 1;

					//Stay in link-down state until reset
					tx_state			<= TX_STATE_DOWN_0;
				end

			end	//end TX_STATE_IDLE_1

		endcase

		//Reset everything when the TAP reinitializes
		if(!tap_active || tap_reset) begin
			tx_data		<= 0;
			tx_state	<= TX_STATE_RESET;
		end

		//When we reset the link, wipe sequence numbers back to initial values.
		//TODO: is this a good idea, or should we keep going from where we left off?
		if(tap_reset)
			tx_seq_num	<= 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TODO: NoC transceivers

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug glue

	always @(posedge clk) begin
		led[3] <= 1;
	end


endmodule
