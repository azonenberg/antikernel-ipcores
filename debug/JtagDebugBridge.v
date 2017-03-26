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

module JtagDebugBridge #(
	parameter NOC_DATA_WIDTH = 32		//Data width of the NoC link (not the JTAG side)
) (
	//Shared by everything NoC side
	input wire					clk,

	//RPC link
	output wire						rpc_tx_en,
	output wire[NOC_DATA_WIDTH-1:0]	rpc_tx_data,
	input wire						rpc_tx_ready,
	input wire						rpc_rx_en,
	input wire[NOC_DATA_WIDTH-1:0]	rpc_rx_data,
	output wire						rpc_rx_ready,

	//Debug stuff
	output reg[3:0]					led = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// NoC transceivers

	reg			rpc_fab_tx_en		= 0;
	wire		rpc_fab_tx_busy;
	reg[15:0]	rpc_fab_tx_dst_addr	= 0;
	reg[7:0]	rpc_fab_tx_callnum	= 0;
	reg[2:0]	rpc_fab_tx_type		= 0;
	reg[20:0]	rpc_fab_tx_d0		= 0;
	reg[31:0]	rpc_fab_tx_d1		= 0;
	reg[31:0]	rpc_fab_tx_d2		= 0;
	wire		rpc_fab_tx_done;

	reg			rpc_fab_rx_ready	= 0;
	wire		rpc_fab_rx_busy;
	wire		rpc_fab_rx_en;

	RPCv3Transceiver #(
		.DATA_WIDTH(NOC_DATA_WIDTH),
		.QUIET_WHEN_IDLE(1),			//TODO: make this configurable?
		.NODE_ADDR(16'h4141)			//TODO: allow sending from >1 host
	) rpc_txvr (
		.clk(clk),

		//Network side
		.rpc_tx_en(rpc_tx_en),
		.rpc_tx_data(rpc_tx_data),
		.rpc_tx_ready(rpc_tx_ready),
		.rpc_rx_en(rpc_rx_en),
		.rpc_rx_data(rpc_rx_data),
		.rpc_rx_ready(rpc_rx_ready),

		//Fabric side
		.rpc_fab_tx_en(rpc_fab_tx_en),
		.rpc_fab_tx_busy(rpc_fab_tx_busy),
		.rpc_fab_tx_dst_addr(rpc_fab_tx_dst_addr),
		.rpc_fab_tx_callnum(rpc_fab_tx_callnum),
		.rpc_fab_tx_type(rpc_fab_tx_type),
		.rpc_fab_tx_d0(rpc_fab_tx_d0),
		.rpc_fab_tx_d1(rpc_fab_tx_d1),
		.rpc_fab_tx_d2(rpc_fab_tx_d2),
		.rpc_fab_tx_done(rpc_fab_tx_done),

		.rpc_fab_rx_ready(rpc_fab_rx_ready),
		.rpc_fab_rx_busy(rpc_fab_rx_busy),
		.rpc_fab_rx_en(rpc_fab_rx_en),
		.rpc_fab_rx_src_addr(),
		.rpc_fab_rx_dst_addr(),
		.rpc_fab_rx_callnum(),
		.rpc_fab_rx_type(),
		.rpc_fab_rx_d0(),
		.rpc_fab_rx_d1(),
		.rpc_fab_rx_d2()
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The TAP interface for discovery (DEBUG_IDCODE register)

	//See https://github.com/azonenberg/jtaghal/wiki/FPGA-debug for ID table
	JtagUserIdentifier #(
		.IDCODE_VID(24'h42445a),	//"ADZ"
		.IDCODE_PID(8'h00)			//Antikernel NoC interface
	) id (

		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The TAP interface for Antikernel debug (DEBUG_DATA register)

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
	// Synchronize TAP reset request over to NoC clock domain.
	// For now, a reset of the TAP will wipe all buffers.

	reg tap_reset_ff = 0;
	reg jtag_side_reset = 0;
	always @(posedge tap_tck_bufh) begin
		tap_reset_ff	<= tap_reset;
		jtag_side_reset	<= tap_reset && !tap_reset_ff;
	end

	wire	noc_side_reset;
	HandshakeSynchronizer sync_tail(
		.clk_a(tap_tck_bufh),
		.en_a(jtag_side_reset),
		.ack_a(),							//We don't need a reset acknowledgement.
											//As long as the NoC clock isn't more than ~30x slower than the JTAG clock,
											//the reset will complete long before anything can happen
		.busy_a(),							//No need to check for busy state, resetting during a reset is a no-op

		.clk_b(clk),
		.en_b(noc_side_reset),
		.ack_b(noc_side_reset)				//Acknowledge the reset as soon as we get it
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFOs between the clock domains

	localparam TX_FIFO_DEPTH = 11'd1024;		//Depth *in words* of transmit fifo

	//Packet structure:
	//First word: 31=RPC, 30=DMA, 10:0=length
	//Data words
	//Last word: 0 = crc OK
	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(TX_FIFO_DEPTH)
		) tx_fifo (
			.wr_clk(tap_tck_bufh),
			.wr_en(),
			.wr_data(),
			.wr_reset(jtag_side_reset),
			.wr_size(),

			.rd_clk(clk),
			.rd_en(),
			.rd_offset(),
			.rd_pop_single(),
			.rd_pop_packet(),
			.rd_packet_size(),
			.rd_data(),
			.rd_size(),
			.rd_reset(noc_side_reset)
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

	CRC8_ATM rx_header_crc(
		.clk(tap_tck_bufh),
		.reset(rx_crc_reset),
		.update(rx_valid),
		.din(rx_shreg),
		.crc(),
		.crc_first24(rx_crc_dout)
		);

	wire[31:0]	rx_payload_crc_dout;
	CRC32_Ethernet_x32 rx_payload_crc(
		.clk(tap_tck_bufh),
		.reset(rx_crc_reset),
		.update(rx_valid && rx_payload_rpc),
		.din(rx_shreg),
		.crc_flipped(rx_payload_crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX state machine

	localparam RX_STATE_IDLE		= 4'h0;
	localparam RX_STATE_HEADER_0	= 4'h1;
	localparam RX_STATE_HEADER_1	= 4'h2;
	localparam RX_STATE_RESET_WAIT	= 4'h3;
	localparam RX_STATE_RPC_0		= 4'h4;
	localparam RX_STATE_RPC_1		= 4'h5;
	localparam RX_STATE_RPC_2		= 4'h6;
	localparam RX_STATE_RPC_3		= 4'h7;
	localparam RX_STATE_CRC			= 4'h8;

	reg[3:0]	rx_state			= RX_STATE_IDLE;
	reg[7:0]	rx_expected_crc 	= 0;

	reg			rx_failed			= 1;

	//Headers for the next outbound frame
	reg			tx_ack_valid		= 0;
	reg			tx_nak_valid		= 0;
	reg[9:0]	tx_seq_num			= 0;
	reg[9:0]	tx_ack_num			= 0;

	//Headers from the current inbound frame (still being parsed)
	reg			rx_flag_ack			= 0;
	reg			rx_flag_nak			= 0;
	reg[9:0]	rx_seq_num			= 0;
	reg[9:0]	rx_credits			= 0;
	reg[9:0]	rx_ack_num			= 0;
	reg			rx_payload_present	= 0;
	reg			rx_payload_rpc		= 0;
	reg			rx_payload_dma		= 0;
	reg[9:0]	rx_payload_length	= 0;
	reg[10:0]	rx_reserved			= 0;

	//DEBUG flags
	reg			tx_payload			= 0;
	reg[31:0]	tx_payload_data		= 0;

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
					rx_flag_ack		<= rx_shreg[31];
					rx_flag_nak		<= rx_shreg[30];
					rx_seq_num		<= rx_shreg[29:20];
					rx_credits		<= rx_shreg[19:10];
					rx_ack_num		<= rx_shreg[9:0];
					rx_state		<= RX_STATE_HEADER_0;
				end

				//We always have at least one clock in idle before a new word comes in
				//Use that time to clear the CRC for the new packet.
				//Also clear RPC/DMA flags so that we don't start CRC-ing too soon
				else begin
					rx_payload_rpc	<= 0;
					rx_payload_dma	<= 0;
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
					rx_payload_present	<= rx_shreg[31];
					rx_payload_rpc		<= rx_shreg[30];
					rx_payload_dma		<= rx_shreg[29];
					rx_reserved			<= rx_shreg[28:18];
					rx_expected_crc		<= rx_shreg[7:0];
					rx_payload_length	<= rx_shreg[17:8];
					rx_state			<= RX_STATE_HEADER_1;
				end

			end	//end RX_STATE_HEADER_0

			//One clock after the header word was fully processed. At this point the CRC is done.
			RX_STATE_HEADER_1: begin

				//If the header checksum is bad, we have a problem.
				//Since we don't know how long this frame was (because the length might be corrupted),
				//we can't skip it and go to the next one.
				//Eventually we can try to recover by looking for the next frame and seeing when we get a good CRC.
				//For now, take the easy way out and just die.
				//Send NAKs with the expected next-sequence number so we can resume later on.
				//Note that we can't send the actual sequence number as it may have been affected by the bit error.
				if(rx_crc_dout != rx_expected_crc) begin
					tx_ack_num		<= tx_ack_num + 1'h1;
					rx_state		<= RX_STATE_RESET_WAIT;
				end

				//Reserved field nonzero? Asking for unsupported option, drop the link
				else if(rx_reserved != 0) begin
					tx_ack_num		<= rx_seq_num;
					rx_state		<= RX_STATE_RESET_WAIT;
				end

				//Good packet with a payload?
				else if(rx_payload_present) begin

					//If RPC is set and not DMA, we have an RPC packet
					if(rx_payload_rpc && !rx_payload_dma) begin
						rx_state		<= RX_STATE_RPC_0;
					end

					//Anything else is currently unsupported, drop the link
					else begin
						tx_ack_num		<= rx_seq_num;
						rx_state		<= RX_STATE_RESET_WAIT;
					end

				end

				//Header-only packet. All good, ready for the next frame
				//TODO: process ACK numbers etc by flushing buffers
				else begin
					tx_ack_valid	<= 1;
					tx_ack_num		<= rx_seq_num;
					rx_state		<= RX_STATE_IDLE;
				end

			end	//end RX_STATE_HEADER_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Sit around and wait until we get reset

			RX_STATE_RESET_WAIT: begin
				rx_failed		<= 1;
				tx_ack_valid	<= 0;
			end	//end RX_STATE_RESET_WAIT

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Receiving an RPC message from the host

			RX_STATE_RPC_0: begin
				if(rx_valid)
					rx_state	<= RX_STATE_RPC_1;
			end	//end RX_STATE_RPC_0

			RX_STATE_RPC_1: begin
				if(rx_valid)
					rx_state	<= RX_STATE_RPC_2;
			end	//end RX_STATE_RPC_1

			RX_STATE_RPC_2: begin
				if(rx_valid)
					rx_state	<= RX_STATE_RPC_3;
			end	//end RX_STATE_RPC_2

			RX_STATE_RPC_3: begin
				if(rx_valid)
					rx_state	<= RX_STATE_CRC;
			end	//end RX_STATE_RPC_3

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Verify the checksum on an incoming data frame

			//For now, assume all is well
			RX_STATE_CRC: begin
				if(rx_valid) begin

					tx_payload		<= 1;
					tx_payload_data	<= 32'h41414141;

					//Check if the CRC is good
					/*
					if(rx_payload_crc_dout == rx_shreg)
						led			<= 4'hf;
					else
						led			<= 4'he;
					*/

					tx_ack_num		<= rx_seq_num;
					rx_state		<= RX_STATE_IDLE;
				end
			end	//end RX_STATE_CRC

		endcase

		//Reset everything when the TAP reinitializes
		if(!tap_active || tap_clear) begin
			tx_ack_valid	<= 0;
			tx_ack_num		<= 0;
			tx_nak_valid	<= 0;
			rx_failed		<= 0;
			rx_state		<= RX_STATE_IDLE;
		end

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

	reg			tx_payload_crc_update	= 0;
	wire[31:0]	tx_payload_crc_dout;
	CRC32_Ethernet_x32 tx_payload_crc(
		.clk(tap_tck_bufh),
		.reset(tx_crc_reset),
		.update(tx_payload_crc_update),
		.din(tx_crc_din),
		.crc_flipped(tx_payload_crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX state machine

	localparam TX_STATE_RESET		= 4'h0;
	localparam TX_STATE_HEADER		= 4'h1;
	localparam TX_STATE_IDLE		= 4'h2;
	localparam TX_STATE_DOWN_0		= 4'h3;
	localparam TX_STATE_DOWN_1		= 4'h4;
	localparam TX_STATE_PAYLOAD_0	= 4'h5;
	localparam TX_STATE_PAYLOAD_1	= 4'h6;
	localparam TX_STATE_PAYLOAD_2	= 4'h7;

	reg[3:0]	tx_state			= TX_STATE_RESET;
	reg			tx_header_done		= 0;
	reg			tx_header_done_adv	= 0;

	always @(posedge tap_tck_bufh) begin

		//Clear flags
		tx_crc_reset			<= 0;
		tx_crc_update			<= 0;
		tx_payload_crc_update	<= 0;
		tx_header_done_adv		<= 0;

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
				tx_state		<= TX_STATE_IDLE;

			end	//end TX_STATE_RESET

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - send idle frames until we have something else to do

			//Send header (doesn't matter if idle or payload
			TX_STATE_HEADER: begin

				if(tx_data_needed) begin

					tx_crc_din		<=
					{
						tx_ack_valid,	//ACK flag
						tx_nak_valid,	//NAK flag
						tx_seq_num,		//Outbound sequence number
						10'h3FF,		//Inbound credit counter (always max for now)
						tx_ack_num		//ACK sequence number
					};

					//Start a new checksum
					tx_crc_update	<= 1;
					tx_crc_reset	<= 1;

					//We sent this packet so bump the sequence number for the next one
					tx_seq_num		<= tx_seq_num + 1'h1;

					//Prepare to send second word
					//TODO: check for tx data in fifo etc
					if(tx_payload)
						tx_state		<= TX_STATE_PAYLOAD_0;
					else
						tx_state		<= TX_STATE_IDLE;

				end

			end	//end TX_STATE_HEADER

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - send idle frames until we have something else to do

			//Send second half of idle frame
			TX_STATE_IDLE: begin

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

					//Prepare to send the next frame unless something bad happened
					if(rx_failed)
						tx_state			<= TX_STATE_DOWN_0;
					else
						tx_state			<= TX_STATE_HEADER;
				end

			end	//end TX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// PAYLOAD -

			TX_STATE_PAYLOAD_0: begin
				if(tx_data_needed) begin

					//Format header
					tx_crc_din		<=
					{
						1'b1,		//Payload is RPC for now
						1'b1,		//RPC payload
						1'b0,		//No DMA payload
						11'h0,		//Reserved
						10'h1,		//Length
						8'h0		//Placeholder for header checksum
					};

					//Checksum the data
					tx_crc_update			<= 1;
					tx_header_done_adv		<= 1;

					tx_state				<= TX_STATE_PAYLOAD_1;

				end
			end	//end TX_STATE_PAYLOAD_0

			TX_STATE_PAYLOAD_1: begin

				if(tx_data_needed) begin

					tx_crc_din				<= tx_payload_data;

					//Checksum the data
					tx_payload_crc_update	<= 1;

					//Go on to the CRC
					//TODO: if we have multiple words of payload, stay in this state for a while
					tx_state				<= TX_STATE_PAYLOAD_2;

				end

			end	//end TX_STATE_PAYLOAD_1

			TX_STATE_PAYLOAD_2: begin

				if(tx_data_needed) begin

					tx_crc_din				<= tx_payload_crc_dout;

					//Prepare to send the next frame unless something bad happened
					if(rx_failed)
						tx_state			<= TX_STATE_DOWN_0;
					else
						tx_state			<= TX_STATE_HEADER;

				end

			end	//end TX_STATE_PAYLOAD_2

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// DOWN - link went down due to a loss of sync, send NAK frames forever

			//Send first half of frame
			TX_STATE_DOWN_0: begin

				if(tx_data_needed) begin

					tx_crc_din		<=
					{
						1'b0,		//ACK flag, always 0 since the link is down
						1'b1,		//NAK flag, always 1 since the link is down
						tx_seq_num,	//Outbound sequence number
						10'h1FF,	//Inbound credit counter, doesn't matter what it is since the link is down
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

endmodule
