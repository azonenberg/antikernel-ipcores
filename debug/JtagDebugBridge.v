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
	parameter NOC_WIDTH = 32		//Data width of the NoC link (not the JTAG side)
) (
	//Shared by everything NoC side
	input wire					clk,

	//RPC link
	output wire						rpc_tx_en,
	output wire[NOC_WIDTH-1:0]	rpc_tx_data,
	input wire						rpc_tx_ready,
	input wire						rpc_rx_en,
	input wire[NOC_WIDTH-1:0]	rpc_rx_data,
	output wire						rpc_rx_ready,

	//Debug stuff
	output reg[3:0]					led = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// NoC transceivers

	reg			rpc_fab_tx_en		= 0;
	wire		rpc_fab_tx_busy;
	reg[15:0]	rpc_fab_tx_src_addr	= 0;	//TODO: force us to be in a specific subnet
	reg[15:0]	rpc_fab_tx_dst_addr	= 0;
	reg[7:0]	rpc_fab_tx_callnum	= 0;
	reg[2:0]	rpc_fab_tx_type		= 0;
	reg[20:0]	rpc_fab_tx_d0		= 0;
	reg[31:0]	rpc_fab_tx_d1		= 0;
	reg[31:0]	rpc_fab_tx_d2		= 0;
	wire		rpc_fab_tx_done;

	reg			rpc_fab_rx_ready	= 1;	//start out ready
	wire		rpc_fab_rx_busy;
	wire		rpc_fab_rx_en;
	wire[15:0]	rpc_fab_rx_src_addr;
	wire[15:0]	rpc_fab_rx_dst_addr;
	wire[7:0]	rpc_fab_rx_callnum;
	wire[2:0]	rpc_fab_rx_type;
	wire[20:0]	rpc_fab_rx_d0;
	wire[31:0]	rpc_fab_rx_d1;
	wire[31:0]	rpc_fab_rx_d2;

	RPCv3Transceiver #(
		.DATA_WIDTH(NOC_WIDTH),
		.QUIET_WHEN_IDLE(1),			//TODO: make this configurable?
		.LEAF_NODE(0)					//We can send from any address
										//TODO: filtering to only send from one subnet
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
		.rpc_fab_tx_src_addr(rpc_fab_tx_src_addr),
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
		.rpc_fab_rx_src_addr(rpc_fab_rx_src_addr),
		.rpc_fab_rx_dst_addr(rpc_fab_rx_dst_addr),
		.rpc_fab_rx_callnum(rpc_fab_rx_callnum),
		.rpc_fab_rx_type(rpc_fab_rx_type),
		.rpc_fab_rx_d0(rpc_fab_rx_d0),
		.rpc_fab_rx_d1(rpc_fab_rx_d1),
		.rpc_fab_rx_d2(rpc_fab_rx_d2)
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

	`include "../synth_helpers/clog2.vh"

	localparam TX_FIFO_DEPTH		= 11'd1024;						//Depth *in words* of transmit fifo
	localparam TX_FIFO_ADDR_BITS	= clog2(TX_FIFO_DEPTH);
	localparam TX_FIFO_SIZE_BITS	= TX_FIFO_ADDR_BITS + 1'h1;		//Number of bits for size (needs one more for empty)

	reg							tx_fifo_wr_en	= 0;
	reg[31:0]					tx_fifo_wr_data	= 0;
	wire[TX_FIFO_SIZE_BITS-1:0]	tx_fifo_wr_size;

	reg							tx_fifo_rd_en			= 0;
	reg[8:0]					tx_fifo_rd_offset		= 0;
	reg							tx_fifo_rd_pop_single	= 0;
	reg							tx_fifo_rd_pop_packet	= 0;
	reg[9:0]					tx_fifo_rd_pop_size		= 0;
	wire[TX_FIFO_SIZE_BITS-1:0]	tx_fifo_rd_size;
	wire[31:0]					tx_fifo_rd_data;

	//Data from JTAG to NoC
	//Packet structure:
	//First word: 31=RPC, 30=DMA, 10:0=length
	//Data words
	//Last word: 0 = crc OK
	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(TX_FIFO_DEPTH)
		) tx_fifo (
			.wr_clk(tap_tck_bufh),
			.wr_en(tx_fifo_wr_en),
			.wr_data(tx_fifo_wr_data),
			.wr_reset(jtag_side_reset),
			.wr_size(tx_fifo_wr_size),

			.rd_clk(clk),
			.rd_en(tx_fifo_rd_en),
			.rd_offset(tx_fifo_rd_offset),
			.rd_pop_single(tx_fifo_rd_pop_single),
			.rd_pop_packet(tx_fifo_rd_pop_packet),
			.rd_packet_size(tx_fifo_rd_pop_size),
			.rd_data(tx_fifo_rd_data),
			.rd_size(tx_fifo_rd_size),
			.rd_reset(noc_side_reset)
		);

	reg							rx_fifo_wr_en	= 0;
	reg[31:0]					rx_fifo_wr_data	= 0;
	wire[TX_FIFO_SIZE_BITS-1:0]	rx_fifo_wr_size;

	//Data from NoC to JTAG
	reg							rx_fifo_rd_en			= 0;
	reg[8:0]					rx_fifo_rd_offset		= 0;
	reg							rx_fifo_rd_pop_single	= 0;
	reg							rx_fifo_rd_pop_packet	= 0;
	reg[9:0]					rx_fifo_rd_pop_size		= 0;
	wire[TX_FIFO_SIZE_BITS-1:0]	rx_fifo_rd_size;
	wire[31:0]					rx_fifo_rd_data;
	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(TX_FIFO_DEPTH)
		) rx_fifo (
			.wr_clk(clk),
			.wr_en(rx_fifo_wr_en),
			.wr_data(rx_fifo_wr_data),
			.wr_reset(noc_side_reset),
			.wr_size(rx_fifo_wr_size),

			.rd_clk(tap_tck_bufh),
			.rd_en(rx_fifo_rd_en),
			.rd_offset(rx_fifo_rd_offset),
			.rd_pop_single(rx_fifo_rd_pop_single),
			.rd_pop_packet(rx_fifo_rd_pop_packet),
			.rd_packet_size(rx_fifo_rd_pop_size),
			.rd_data(rx_fifo_rd_data),
			.rd_size(rx_fifo_rd_size),
			.rd_reset(jtag_side_reset)
		);

	//Compute credits available
	//Credits are measured in 128-bit units, tx_fifo_wr_size is measured in 32-bit units
	reg[9:0] credit_count = 0;
	always @(*) begin

		//If <12 bit size, we can fit full size in credit_count (10 bits plus two LSBs cut off)
		if(TX_FIFO_SIZE_BITS <= 12)
			credit_count		<= tx_fifo_wr_size[TX_FIFO_SIZE_BITS-1 : 2];

		//Nope, have to truncate etc
		else begin

			//More than max credits available? Report max since we don't have enough bits
			if(tx_fifo_wr_size[TX_FIFO_SIZE_BITS-1 : 12] != 0)
				credit_count	<= 10'h3ff;

			//Nope, truncate off the leading zeroes and report exact size
			else
				credit_count	<= tx_fifo_wr_size[11:2];

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// NoC TX state machine

	localparam NTX_STATE_IDLE				= 0;
	localparam NTX_STATE_READ_HEADER		= 1;
	localparam NTX_STATE_WAIT_FOR_PACKET	= 2;
	localparam NTX_STATE_READ_CRCSTATUS		= 3;
	localparam NTX_STATE_POP				= 4;
	localparam NTX_STATE_RPC_0				= 5;
	localparam NTX_STATE_RPC_1				= 6;
	localparam NTX_STATE_RPC_2				= 7;
	localparam NTX_STATE_RPC_3				= 8;
	localparam NTX_STATE_RPC_WAIT			= 9;

	reg[3:0]	ntx_state					= NTX_STATE_IDLE;
	reg[10:0]	ntx_packet_len				= 0;
	reg			ntx_packet_rpc				= 0;
	reg			ntx_packet_dma				= 0;

	always @(posedge clk) begin

		tx_fifo_rd_en			<= 0;
		tx_fifo_rd_pop_single	<= 0;
		tx_fifo_rd_pop_packet	<= 0;

		rpc_fab_tx_en			<= 0;

		case(ntx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Sit around and wait for data to appear in our FIFO

			NTX_STATE_IDLE: begin

				//If we have any data at all to read, get the header word (type and length).
				//We don't yet know if the entire packet is in the buffer since we don't know how big it is!
				//Pop the header word since we don't need it in the FIFO anymore.
				if(tx_fifo_rd_size > 0) begin
					tx_fifo_rd_en			<= 1;
					tx_fifo_rd_offset		<= 0;
					tx_fifo_rd_pop_single	<= 1;
					ntx_state				<= NTX_STATE_READ_HEADER;
				end

			end	//end NTX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for pop to occur, then move back to idle

			NTX_STATE_POP: begin
				if(!tx_fifo_rd_pop_packet && !tx_fifo_rd_pop_single)
					ntx_state				<= NTX_STATE_POP;
			end	//end NTX_STATE_POP

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait until the entire packet has been received

			//Save headers, then wait for the whole packet to be ready
			NTX_STATE_READ_HEADER: begin
				if(!tx_fifo_rd_en) begin
					ntx_packet_len		<= tx_fifo_rd_data[10:0];
					ntx_packet_rpc		<= tx_fifo_rd_data[31];
					ntx_packet_dma		<= tx_fifo_rd_data[30];
					ntx_state			<= NTX_STATE_WAIT_FOR_PACKET;
				end
			end	//end NTX_STATE_READ_HEADER

			//Wait for the whole packet to be in the buffer (this may take a while!)
			NTX_STATE_WAIT_FOR_PACKET: begin

				//need at least packet_len + one more for CRC status
				if(tx_fifo_rd_size > ntx_packet_len) begin
					tx_fifo_rd_en		<= 1;
					tx_fifo_rd_offset	<= ntx_packet_len[8:0];
					ntx_state			<= NTX_STATE_READ_CRCSTATUS;
				end

			end	//NTX_STATE_WAIT_FOR_PACKET

			//We have the entire packet!
			//Read the CRC pass/fail status
			NTX_STATE_READ_CRCSTATUS: begin

				if(!tx_fifo_rd_en) begin

					//If CRC failure, pop the entire packet. (Was already NAK'd by JTAG clock domain)
					if(!tx_fifo_rd_data[0]) begin
						tx_fifo_rd_pop_packet	<= 1;
						tx_fifo_rd_pop_size		<= ntx_packet_len[9:0];
						ntx_state				<= NTX_STATE_POP;
					end

					//If not RPC, drop it (TODO handle DMA)
					else if(!ntx_packet_rpc || ntx_packet_dma || (ntx_packet_len != 4)) begin
						tx_fifo_rd_pop_packet	<= 1;
						tx_fifo_rd_pop_size		<= ntx_packet_len[9:0];
						ntx_state				<= NTX_STATE_POP;
					end

					//It's an RPC packet, read the first data word
					else begin
						tx_fifo_rd_en			<= 1;
						tx_fifo_rd_offset		<= 0;
						ntx_state				<= NTX_STATE_RPC_0;
					end

				end

			end //end NTX_STATE_READ_CRCSTATUS

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Process an RPC message

			NTX_STATE_RPC_0: begin

				//If we're waiting for the read of the first word, kick off the second one
				if(tx_fifo_rd_offset == 0) begin
					tx_fifo_rd_en		<= 1;
					tx_fifo_rd_offset	<= 1;
				end

				//Second word's read is executing now, first word is ready.
				//TODO: make "router" transceiver that does sram-style reads?
				else begin
					rpc_fab_tx_src_addr	<= tx_fifo_rd_data[31:16];
					rpc_fab_tx_dst_addr	<= tx_fifo_rd_data[15:0];

					tx_fifo_rd_en		<= 1;
					tx_fifo_rd_offset	<= 2;
					ntx_state			<= NTX_STATE_RPC_1;
				end

			end	//end NTX_STATE_RPC_0

			NTX_STATE_RPC_1: begin
				rpc_fab_tx_callnum	<= tx_fifo_rd_data[31:24];
				rpc_fab_tx_type		<= tx_fifo_rd_data[23:21];
				rpc_fab_tx_d0		<= tx_fifo_rd_data[20:0];

				tx_fifo_rd_en		<= 1;
				tx_fifo_rd_offset	<= 3;
				ntx_state			<= NTX_STATE_RPC_2;
			end	//end NTX_STATE_RPC_1

			NTX_STATE_RPC_2: begin
				rpc_fab_tx_d1		<= tx_fifo_rd_data;
				ntx_state			<= NTX_STATE_RPC_3;
			end	//end NTX_STATE_RPC_2

			NTX_STATE_RPC_3: begin
				rpc_fab_tx_d2			<= tx_fifo_rd_data;
				ntx_state				<= NTX_STATE_RPC_WAIT;

				//Pop the RPC packet from the FIFO since we're done with it now
				//Pop 5 words (4 data + 1 CRC status)
				tx_fifo_rd_pop_packet	<= 1;
				tx_fifo_rd_pop_size		<= 5;

				//Send the message
				rpc_fab_tx_en			<= 1;

			end	//end NTX_STATE_RPC_3

			//Wait for the transmission to complete
			NTX_STATE_RPC_WAIT: begin
				if(rpc_fab_tx_done)
					ntx_state			<= NTX_STATE_IDLE;
			end	//end NTX_STATE_RPC_WAIT

		endcase

		//Reset everything when jtag link comes up
		if(noc_side_reset)
			ntx_state		<= NTX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// NoC RX state machine

	localparam NRX_STATE_IDLE				= 0;
	localparam NRX_STATE_RPC_0				= 1;
	localparam NRX_STATE_RPC_1				= 2;
	localparam NRX_STATE_RPC_2				= 3;
	localparam NRX_STATE_RPC_3				= 4;
	localparam NRX_STATE_RPC_4				= 5;

	reg[3:0]	nrx_state					= NRX_STATE_IDLE;

	always @(posedge clk) begin

		rpc_fab_rx_ready		<= 0;

		rx_fifo_wr_en			<= 0;

		case(nrx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE: wait for a message to come in

			NRX_STATE_IDLE: begin

				//TODO: handle DMA

				//If a new RPC message comes in, push it into the RX FIFO
				if(rpc_fab_rx_en)
					nrx_state	<= NRX_STATE_RPC_0;

			end	//end NRX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// RPC: Handle RPC messages

			NRX_STATE_RPC_0: begin

				//Wait for there to be enough buffer space for the entire message
				if(rx_fifo_wr_size >= 5) begin
					rx_fifo_wr_en		<= 1;
					rx_fifo_wr_data		<= {1'b1, 1'b0, 19'h0, 11'd4};	//rpc, dma, padding, length
					nrx_state			<= NRX_STATE_RPC_1;
				end

			end	//end NRX_STATE_RPC_0

			NRX_STATE_RPC_1: begin
				rx_fifo_wr_en			<= 1;
				rx_fifo_wr_data			<= {rpc_fab_rx_src_addr, rpc_fab_rx_dst_addr};
				nrx_state				<= NRX_STATE_RPC_2;
			end	//end NRX_STATE_RPC_1

			NRX_STATE_RPC_2: begin
				rx_fifo_wr_en			<= 1;
				rx_fifo_wr_data			<= {rpc_fab_rx_callnum, rpc_fab_rx_type, rpc_fab_rx_d0 };
				nrx_state				<= NRX_STATE_RPC_3;
			end	//end NRX_STATE_RPC_2

			NRX_STATE_RPC_3: begin
				rx_fifo_wr_en			<= 1;
				rx_fifo_wr_data			<= rpc_fab_rx_d1;
				nrx_state				<= NRX_STATE_RPC_4;
			end	//end NRX_STATE_RPC_3

			NRX_STATE_RPC_4: begin
				rx_fifo_wr_en			<= 1;
				rx_fifo_wr_data			<= rpc_fab_rx_d2;

				//Let the transceiver accept new messages again
				rpc_fab_rx_ready		<= 1;
				nrx_state				<= NRX_STATE_IDLE;

			end	//end NRX_STATE_RPC_4

		endcase

		//Reset everything when jtag link comes up
		//Note that this may result in dropping a packet if we had one half-received
		if(noc_side_reset) begin
			rpc_fab_rx_ready	<= 1;
			nrx_state			<= NRX_STATE_IDLE;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Convert JTAG data from a stream of bits to a stream of 32-bit words

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
	// JTAG RX CRC calculation

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
	// JTAG RX state machine

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

	always @(posedge tap_tck_bufh) begin

		rx_crc_reset		<= 0;
		tx_fifo_wr_en		<= 0;

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

				//TODO: If incoming sequence number is not one greater than our last packet, we have a problem.
				//NAK this one and don't do anything with it

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

						//If there's enough space for the packet (length/type, data, OK)
						//then start pushing data
						if(tx_fifo_wr_size >= 6) begin
							rx_state		<= RX_STATE_RPC_0;
							tx_fifo_wr_en	<= 1;
							tx_fifo_wr_data	<= {1'b1, 1'b0, 19'h0, 11'd4};	//rpc, dma, padding, length
						end

						//Not enough space, drop the link
						else begin
							tx_ack_num		<= rx_seq_num;
							rx_state		<= RX_STATE_RESET_WAIT;
						end

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
				if(rx_valid) begin
					tx_fifo_wr_en	<= 1;
					tx_fifo_wr_data	<= rx_shreg;
					rx_state		<= RX_STATE_RPC_1;
				end
			end	//end RX_STATE_RPC_0

			RX_STATE_RPC_1: begin
				if(rx_valid) begin
					tx_fifo_wr_en	<= 1;
					tx_fifo_wr_data	<= rx_shreg;
					rx_state		<= RX_STATE_RPC_2;
				end
			end	//end RX_STATE_RPC_1

			RX_STATE_RPC_2: begin
				if(rx_valid) begin
					tx_fifo_wr_en	<= 1;
					tx_fifo_wr_data	<= rx_shreg;
					rx_state		<= RX_STATE_RPC_3;
				end
			end	//end RX_STATE_RPC_2

			RX_STATE_RPC_3: begin
				if(rx_valid) begin
					tx_fifo_wr_en	<= 1;
					tx_fifo_wr_data	<= rx_shreg;
					rx_state		<= RX_STATE_CRC;
				end
			end	//end RX_STATE_RPC_3

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Verify the checksum on an incoming data frame

			//For now, assume all is well
			RX_STATE_CRC: begin
				if(rx_valid) begin

					//Write CRC pass/fail status to the FIFO
					tx_fifo_wr_en			<= 1;
					if(rx_payload_crc_dout == rx_shreg)
						tx_fifo_wr_data		<= 32'h1;
					else
						tx_fifo_wr_data		<= 32'h0;

					//TODO: Send a single NAK if payload CRC is bad

					//Grab ACK number and get ready to receive the next frame
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
	// JTAG TX CRC calculation

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
	// JTAG TX state machine

	localparam TX_STATE_RESET		= 4'h0;
	localparam TX_STATE_HEADER		= 4'h1;
	localparam TX_STATE_IDLE		= 4'h2;
	localparam TX_STATE_DOWN_0		= 4'h3;
	localparam TX_STATE_DOWN_1		= 4'h4;
	localparam TX_STATE_RPC_0	= 4'h5;
	localparam TX_STATE_RPC_1	= 4'h6;
	localparam TX_STATE_RPC_2	= 4'h7;

	reg[3:0]	tx_state			= TX_STATE_RESET;
	reg			tx_header_done		= 0;
	reg			tx_header_done_adv	= 0;

	reg			tx_read_header		= 0;
	reg			tx_is_rpc			= 0;
	reg			tx_is_dma			= 0;
	reg[10:0]	tx_length			= 0;

	reg			rx_fifo_rd_en_ff	= 0;

	always @(posedge tap_tck_bufh) begin

		//Clear flags
		tx_crc_reset			<= 0;
		tx_crc_update			<= 0;
		tx_payload_crc_update	<= 0;
		tx_header_done_adv		<= 0;
		rx_fifo_rd_en			<= 0;
		rx_fifo_rd_pop_packet	<= 0;
		rx_fifo_rd_pop_single	<= 0;

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

		//If there's at least two words of data in the RX FIFO, and we aren't already looking at a packet,
		//read the header to see what it is.
		if(!tx_read_header && (rx_fifo_rd_size > 1) && !rx_fifo_rd_en && !rx_fifo_rd_en_ff) begin
			rx_fifo_rd_offset	<= 0;
			rx_fifo_rd_en		<= 1;
		end

		//Once the read completes, parse the header and pull it out of the FIFO.
		rx_fifo_rd_en_ff			<= rx_fifo_rd_en;
		if(!tx_read_header && rx_fifo_rd_en_ff) begin
			tx_read_header			<= 1;
			rx_fifo_rd_pop_single	<= 1;
			tx_is_rpc				<= rx_fifo_rd_data[31];
			tx_is_dma				<= rx_fifo_rd_data[30];
			tx_length				<= rx_fifo_rd_data[10:0];
		end

		//After we pop a packet out of the FIFO, the saved headers are no longer valid
		if(rx_fifo_rd_pop_packet)
			tx_read_header	<= 0;

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
					1'b0,			//ACK flag, always 0 since we haven't seen any packets yet
					1'b0,			//NAK flag, always 0 since we haven't seen any packets yet
					10'h0,			//Outbound sequence number (we start from zero when the link goes up)
					credit_count,	//Inbound credit counter (# free words in jtag-to-NoC FIFO)
									//TODO
					10'h0			//ACK sequence number (ignored since ACK/NAK aren't set)
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

			//Send header (doesn't matter if idle or payload)
			TX_STATE_HEADER: begin

				if(tx_data_needed) begin

					tx_crc_din		<=
					{
						tx_ack_valid,	//ACK flag
						tx_nak_valid,	//NAK flag
						tx_seq_num,		//Outbound sequence number
						credit_count,	//Inbound credit counter (# free words in jtag-to-NoC FIFO)
						tx_ack_num		//ACK sequence number
					};

					//Start a new checksum
					tx_crc_update	<= 1;
					tx_crc_reset	<= 1;

					//We sent this packet so bump the sequence number for the next one
					tx_seq_num		<= tx_seq_num + 1'h1;

					//Prepare to send second word.
					//What we do here depends on if we have a packet to forward and, if so, what it is.
					//Default to sending an idle frame.
					tx_state			<= TX_STATE_IDLE;

					//We have a packet! May or may not be interesting at this point in time
					if(tx_read_header) begin

						//If it's a properly sized RPC frame, handle that
						if(tx_is_rpc && !tx_is_dma && (tx_length == 4) )
							tx_state	<= TX_STATE_RPC_0;

						//TODO: If it's a properly sized DMA frame, handle that
						//else if(tx_is_dma && !tx_is_rpc && (tx_length >= 3) ) begin
						//end

						//Packet is malformed in some way. Discard it silently and continue on our merry way.
						//TODO: send some kind of alert to host?
						else begin
							rx_fifo_rd_pop_packet	<= 1;
							rx_fifo_rd_pop_size		<= tx_length[9:0];
						end

					end

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
			// RPC - sending an RPC message

			TX_STATE_RPC_0: begin
				if(tx_data_needed) begin

					//Format header
					tx_crc_din		<=
					{
						1'b1,		//We have a payload
						1'b1,		//It's RPC
						1'b0,		//(and not DMA)
						11'h0,		//Reserved, leave zero
						10'h4,		//4 bytes long
						8'h0		//Placeholder for header checksum
					};

					//Checksum the data
					tx_crc_update			<= 1;
					tx_header_done_adv		<= 1;

					//Read the first data word
					rx_fifo_rd_en			<= 1;
					rx_fifo_rd_offset		<= 0;

					tx_state				<= TX_STATE_RPC_1;

				end
			end	//end TX_STATE_RPC_0

			TX_STATE_RPC_1: begin

				if(tx_data_needed) begin

					tx_crc_din					<= rx_fifo_rd_data;

					//Checksum the data
					tx_payload_crc_update		<= 1;

					//If we just sent the last payload word, go on to the CRC.
					//As we do that, pop the completed packet from the RX FIFO.
					if(rx_fifo_rd_offset == 3) begin
						rx_fifo_rd_pop_packet	<= 1;
						rx_fifo_rd_pop_size		<= 4;

						tx_state				<= TX_STATE_RPC_2;
					end

					//Nope, read next data word
					else begin
						rx_fifo_rd_en			<= 1;
						rx_fifo_rd_offset		<= rx_fifo_rd_offset + 1'h1;
					end

				end

			end	//end TX_STATE_RPC_1

			TX_STATE_RPC_2: begin

				if(tx_data_needed) begin

					tx_crc_din				<= tx_payload_crc_dout;

					//Prepare to send the next frame unless something bad happened
					if(rx_failed)
						tx_state			<= TX_STATE_DOWN_0;
					else
						tx_state			<= TX_STATE_HEADER;

				end

			end	//end TX_STATE_RPC_2

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// DOWN - link went down due to a loss of sync, send NAK frames forever

			//Send first half of frame
			TX_STATE_DOWN_0: begin

				if(tx_data_needed) begin

					tx_crc_din		<=
					{
						1'b0,			//ACK flag, always 0 since the link is down
						1'b1,			//NAK flag, always 1 since the link is down
						tx_seq_num,		//Outbound sequence number
						credit_count,	//Inbound credit counter (# free words in jtag-to-NoC FIFO)
						tx_ack_num		//ACK sequence number
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
