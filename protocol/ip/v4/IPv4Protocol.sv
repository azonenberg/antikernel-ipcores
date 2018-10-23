`timescale 1ns / 1ps
`default_nettype none
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

`include "EthernetBus.svh"

module IPv4Protocol(

	//Clocks
	input wire				clk,

	//Constant-ish state data
	input wire[31:0]		our_ip_address,
	input wire[31:0]		our_subnet_mask,
	input wire[31:0]		our_broadcast_address,

	//Incoming Ethernet data
	input wire EthernetRxL2Bus	rx_l2_bus,

	//Outbound data (same clock domain as incoming)
	output EthernetTxArpBus	tx_l2_bus					= {1'h0, 1'h0, 1'h0, 32'h0, 32'h0, 1'h0, 1'h0},

	//Interface to upper level protocol
	output EthernetBus		rx_l3_bus			 		= {1'h0, 1'h0, 1'h0, 32'h0, 1'h0, 1'h0},

	output reg[15:0]		rx_l3_payload_len			= 0,	//size of upper layer payload only
															//(not the IP datagram length)
	output reg[7:0]			rx_l3_protocol				= 0,
	output reg				rx_l3_protocol_is_icmp		= 0,
	output reg				rx_l3_protocol_is_udp		= 0,
	output reg				rx_l3_protocol_is_tcp		= 0,
	output reg[31:0]		rx_l3_src_ip				= 0,
	output reg[31:0]		rx_l3_dst_ip				= 0,
	output reg				rx_l3_headers_valid			= 0,
	output wire[15:0]		rx_l3_pseudo_header_csum,

	//Transmit data from upper level protocol
	input wire EthernetBus	tx_l3_bus,
	input wire[15:0]		tx_l3_payload_len,
	input wire[31:0]		tx_l3_dst_ip,
	input wire[7:0]			tx_l3_protocol

	//TODO: performance counters
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// State values for receiver (need this before checksum so we can ignore the incoming checksum properly)

	`include "../IPProtocols.vh"

	localparam	RX_STATE_IDLE		= 4'h0;
	localparam	RX_STATE_HEADER_0	= 4'h1;
	localparam	RX_STATE_HEADER_1	= 4'h2;
	localparam	RX_STATE_HEADER_2	= 4'h3;
	localparam	RX_STATE_HEADER_3	= 4'h4;
	localparam	RX_STATE_HEADER_4	= 4'h5;
	localparam	RX_STATE_HEADER_5	= 4'h6;
	localparam	RX_STATE_BODY		= 4'h7;
	localparam	RX_STATE_PADDING	= 4'h8;

	reg[3:0]	rx_state			= RX_STATE_IDLE;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX checksum calculation

	wire[15:0]	rx_header_checksum_expected;

	//Don't worry about unaligned message sizes
	//since Ethernet2FrameDecoder pads with zeroes (identity for this checksum algorithm)
	InternetChecksum32bit rx_checksum(
		.clk(clk),
		.load(1'b0),
		.reset(rx_l2_bus.start),
		.process(rx_l2_bus.data_valid && (rx_state != RX_STATE_BODY) ),
		.din(rx_l2_bus.data),
		.sumout(),
		.csumout(rx_header_checksum_expected)
	);

	//RX pseudo header checksum calculation
	reg[31:0]	rx_phdr_din;
	reg			rx_phdr_process;
	always_comb begin

		//Default to processing everything

		rx_phdr_din		<= rx_l2_bus.data[31:0];
		rx_phdr_process	<= 0;

		case(rx_state)

			RX_STATE_HEADER_2: begin
				rx_phdr_din		<= rx_l3_payload_len;
				rx_phdr_process	<= 1;
			end

			RX_STATE_HEADER_3: begin
				rx_phdr_din		<= rx_l2_bus.data[23:16];
				rx_phdr_process	<= 1;
			end

			RX_STATE_HEADER_4: rx_phdr_process	<= 1;

			RX_STATE_HEADER_5: rx_phdr_process	<= 1;

		endcase

	end
	InternetChecksum32bit rx_phdr_checksum(
		.clk(clk),
		.load(1'b0),
		.reset(rx_l2_bus.start),
		.process(rx_phdr_process),
		.din(rx_phdr_din),
		.sumout(rx_l3_pseudo_header_csum),
		.csumout()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX datapath

	//Packet state that we don't currently use
	//May eventually be brought up to top-level ports
	/*
	reg[5:0]	rx_diffserv_point		= 0;
	reg[1:0]	rx_ecn					= 0;
	reg[15:0]	rx_identifier			= 0;
	reg			rx_flag_evil			= 0;
	reg			rx_flag_df				= 0;
	reg			rx_flag_mf				= 0;
	reg[12:0]	rx_frag_offset			= 0;
	reg[7:0]	rx_ttl					= 0;
	*/

	reg[15:0]	payload_bytes_so_far	= 0;

	wire[15:0]	payload_bytes_next		= payload_bytes_so_far + rx_l2_bus.bytes_valid;

	always_ff @(posedge clk) begin

		//Forward flags from layer 2 by default
		rx_l3_bus.start			<= rx_l2_bus.start;
		rx_l3_bus.drop			<= rx_l2_bus.drop;

		rx_l3_bus.data_valid	<= 0;
		rx_l3_bus.bytes_valid	<= 0;
		rx_l3_bus.commit		<= 0;
		rx_l3_headers_valid		<= 0;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for a new packet to come in

			RX_STATE_IDLE: begin

				if(rx_l2_bus.start) begin
					payload_bytes_so_far	<= 0;
					rx_state				<= RX_STATE_HEADER_0;
				end

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// HEADER - read the packet headers and calculate the checksum

			RX_STATE_HEADER_0: begin

				//Wait for layer 2 headers to be read
				//Drop anything that's not an IPv4 packet
				if(rx_l2_bus.headers_valid) begin
					if(rx_l2_bus.ethertype_is_ipv4)
						rx_state		<= RX_STATE_HEADER_1;
					else begin
						rx_state		<= RX_STATE_IDLE;
						rx_l3_bus.drop	<= 1;
					end
				end

			end	//end RX_STATE_HEADER_0

			RX_STATE_HEADER_1: begin

				if(rx_l2_bus.data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_bus.drop			<= 1;
					end

					//Check version and header length.
					//We don't support options so drop anything with them
					else if(rx_l2_bus.data[31:24] != {4'h4, 4'h5}) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_bus.drop			<= 1;
					end

					else begin
						/*
						rx_diffserv_point		<= rx_l2_bus.data[23:18];
						rx_ecn					<= rx_l2_bus.data[17:16];
						*/

						//Total length must be at least 20 bytes
						//(minimum IP header size)
						if(rx_l2_bus.data[15:0] < 16'd20) begin
							rx_state			<= RX_STATE_IDLE;
							rx_l3_bus.drop		<= 1;
						end

						//Valid, figure out how big the payload is
						else begin
							rx_l3_payload_len	<= rx_l2_bus.data[15:0] - 16'd20;
							rx_state			<= RX_STATE_HEADER_2;
						end
					end

				end

			end	//end RX_STATE_HEADER_1

			RX_STATE_HEADER_2: begin

				if(rx_l2_bus.data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_bus.drop			<= 1;
					end

					//We don't support fragmentation.
					//Drop anything with MF or frag offset nonzero.
					else if(rx_l2_bus.data[13] || (rx_l2_bus.data[12:0] != 0) ) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_bus.drop			<= 1;
					end

					//Save fragmentation/flag data
					else begin
						/*
						rx_identifier			<= rx_l2_bus.data[31:16];
						rx_flag_evil			<= rx_l2_bus.data[15];
						rx_flag_df				<= rx_l2_bus.data[14];
						rx_flag_mf				<= rx_l2_bus.data[13];
						rx_frag_offset			<= rx_l2_bus.data[12:0];
						*/

						rx_state				<= RX_STATE_HEADER_3;
					end

				end

			end	//end RX_STATE_HEADER_2

			RX_STATE_HEADER_3: begin

				if(rx_l2_bus.data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						rx_l3_bus.drop				<= 1;
					end

					else begin
						//rx_ttl					<= rx_l2_bus.data[31:24];
						rx_l3_protocol				<= rx_l2_bus.data[23:16];
						rx_l3_protocol_is_icmp		<= (rx_l2_bus.data[23:16] == IP_PROTO_ICMP);
						rx_l3_protocol_is_udp		<= (rx_l2_bus.data[23:16] == IP_PROTO_UDP);
						rx_l3_protocol_is_tcp		<= (rx_l2_bus.data[23:16] == IP_PROTO_TCP);
						//rx_header_checksum		<= rx_l2_bus.data[15:0];

						rx_state					<= RX_STATE_HEADER_4;
					end

				end

			end	//end RX_STATE_HEADER_3

			RX_STATE_HEADER_4: begin

				if(rx_l2_bus.data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						rx_l3_bus.drop				<= 1;
					end

					else begin
						rx_l3_src_ip				<= rx_l2_bus.data;
						rx_state					<= RX_STATE_HEADER_5;
					end

				end

			end	//end RX_STATE_HEADER_4

			RX_STATE_HEADER_5: begin

				if(rx_l2_bus.data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						rx_l3_bus.drop				<= 1;
					end

					else begin
						rx_l3_dst_ip				<= rx_l2_bus.data;

						//See if the packet is intended for us
						//This means either our unicast address, our subnet's broadcast address,
						//or the global broadcast address.
						if( (rx_l2_bus.data == our_ip_address) ||
							(rx_l2_bus.data == our_broadcast_address) ||
							(rx_l2_bus.data == 32'hffffffff) ) begin

							rx_state				<= RX_STATE_BODY;
							rx_l3_headers_valid		<= 1;

						end

						//Nope, it's for somebody else. Drop it.
						else begin
							rx_state				<= RX_STATE_IDLE;
							rx_l3_bus.drop			<= 1;
						end

					end

				end

			end	//end RX_STATE_HEADER_5

			RX_STATE_BODY: begin

				//Forward packet data
				if(rx_l2_bus.data_valid) begin
					rx_l3_bus.data_valid		<= 1;
					rx_l3_bus.bytes_valid		<= rx_l2_bus.bytes_valid;
					rx_l3_bus.data				<= rx_l2_bus.data;

					payload_bytes_so_far		<= payload_bytes_next;

					//If we've reached the end of the packet body, anything afterward is padding
					//Ignore it. Also, tell upper level protocol to do so
					if(payload_bytes_next >= rx_l3_payload_len) begin
						rx_state				<= RX_STATE_PADDING;
						rx_l3_bus.bytes_valid	<= rx_l3_payload_len - payload_bytes_so_far;
					end

				end

				//Verify header checksum
				if(payload_bytes_so_far == 0) begin

					if(rx_header_checksum_expected != 16'h0000) begin
						rx_state			<= RX_STATE_IDLE;
						rx_l3_bus.drop		<= 1;
					end

				end

				//At the end of the packet, commit upstream and go back to idle
				else if(rx_l2_bus.commit) begin
					rx_l3_bus.commit		<= 1;
					rx_state				<= RX_STATE_IDLE;
				end

			end	//end RX_STATE_BODY

			//Ignore padding, wait for end of packet
			RX_STATE_PADDING: begin

				//At the end of the packet, commit upstream and go back to idle
				if(rx_l2_bus.commit) begin
					rx_l3_bus.commit		<= 1;
					rx_state				<= RX_STATE_IDLE;
				end

			end	//end RX_STATE_PADDING

		endcase

		//If we get a drop request from the MAC, abort everything
		if(rx_l2_bus.drop)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX FIFO to store incoming data while we calculate the header checksum

	reg			tx_fifo_rd	= 0;
	reg			tx_fifo_rst	= 0;
	wire[31:0]	tx_fifo_rdata;

	wire[9:0]	tx_fifo_rsize;

	SingleClockFifo #(
		.WIDTH(32),
		.DEPTH(512)
	) tx_fifo (
		.clk(clk),
		.wr(tx_l3_bus.data_valid),
		.din(tx_l3_bus.data),

		.rd(tx_fifo_rd),
		.dout(tx_fifo_rdata),
		.overflow(),
		.underflow(),
		.empty(),
		.full(),
		.rsize(tx_fifo_rsize),
		.wsize(),
		.reset(tx_fifo_rst)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX state machine constants

	enum logic[3:0]
	{
		TX_STATE_IDLE		= 4'h0,
		TX_STATE_HEADER_0	= 4'h1,
		TX_STATE_HEADER_1	= 4'h2,
		TX_STATE_HEADER_2	= 4'h3,
		TX_STATE_HEADER_3	= 4'h4,
		TX_STATE_HEADER_4	= 4'h5,
		TX_STATE_HEADER_5	= 4'h6,
		TX_STATE_HEADER_6	= 4'h7,
		TX_STATE_BODY		= 4'h8,
		TX_STATE_COMMIT		= 4'h9
	} tx_state	= TX_STATE_IDLE;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX checksum engine

	wire[15:0]	tx_header_checksum;

	reg			tx_checksum_process;
	reg[31:0]	tx_checksum_din;

	reg[15:0]	tx_total_len		= 0;

	//Cheat a bit: Some fields in our outbound header are constant
	//Version = IPv4
	//Header len = 5 words
	//DSCP / ECN = 0
	//Flags = DF (3'b010)
	//Frag offset = 0
	//ID = 0
	//We can thus initialize our checksum to the constant value 16'h8500
	//then add in the {length, TTL/protocol}, source IP, and dest IP in only 3 words.
	//TODO: bake our source IP into this pregenerated checksum too

	InternetChecksum32bit tx_checksum(
		.clk(clk),
		.load(tx_l3_bus.start),
		.reset(1'b0),
		.process(tx_checksum_process),
		.din(tx_checksum_din),
		.sumout(),
		.csumout(tx_header_checksum)
	);

	always_ff @(posedge clk) begin

		tx_checksum_process	<= 0;

		//If idle, seed our checksum with a constant value
		//TODO: separate port on checksum core for initialization
		tx_checksum_din		<= 16'h8500;

		case(tx_state)

			//Insert length, TTL, protocol
			TX_STATE_HEADER_0: begin
				tx_checksum_process		<= 1;
				tx_checksum_din			<= {8'hff, tx_l3_protocol, tx_total_len};
			end	//end TX_STATE_HEADER_0

			//Insert source IP (TODO: bake this into init value so we can skip the idle in TX_STATE_HEADER_0)
			TX_STATE_HEADER_1: begin
				tx_checksum_process		<= 1;
				tx_checksum_din			<= our_ip_address;
			end	//end TX_STATE_HEADER_1

			//Insert dest IP
			TX_STATE_HEADER_2: begin
				tx_checksum_process		<= 1;
				tx_checksum_din			<= tx_l3_dst_ip;
			end	//end TX_STATE_HEADER_2

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX datapath

	reg[15:0]	tx_bytes_left	= 0;

	always_ff @(posedge clk) begin

		tx_fifo_rst				<= 0;
		tx_fifo_rd				<= 0;

		tx_l2_bus.start			<= 0;
		tx_l2_bus.drop			<= 0;
		tx_l2_bus.commit		<= 0;
		tx_l2_bus.data_valid	<= 0;
		tx_l2_bus.bytes_valid	<= 0;

		case(tx_state)

			TX_STATE_IDLE: begin

					//We require header information to be valid when we get the start request so we can start
					//sending stuff right away.
					//Ignore any request to send less than 8 bytes, all layer-3 protocols have >=8 bytes of headers
					//and we can optimize a bit by guaranteeing this.
					if(tx_l3_bus.start && (tx_l3_payload_len > 8) ) begin
						tx_l2_bus.start	<= 1;
						tx_state		<= TX_STATE_HEADER_0;

						//Precompute total packet length
						tx_total_len		<= 8'd20 + tx_l3_payload_len;

						tx_bytes_left		<= tx_l3_payload_len;

						tx_l2_bus.dst_ip	<= tx_l3_dst_ip;
					end

			end	//end TX_STATE_IDLE

			//Can't send anything yet, waiting for checksum latency
			TX_STATE_HEADER_0: begin
				tx_state	<= TX_STATE_HEADER_1;
			end	//end TX_STATE_HEADER_0

			TX_STATE_HEADER_1: begin
				tx_state	<= TX_STATE_HEADER_2;
			end	//end TX_STATE_HEADER_1

			//This header is constant except for length since we don't support diffserv/ecn
			TX_STATE_HEADER_2: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.bytes_valid	<= 4;
				tx_l2_bus.data			<= {4'h4, 4'h5, 6'h0, 2'h0, tx_total_len};

				tx_state				<= TX_STATE_HEADER_3;
			end	//end TX_STATE_HEADER_2

			//This header is entirely constant since we don't support fragging
			TX_STATE_HEADER_3: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.bytes_valid	<= 4;
				tx_l2_bus.data			<= {16'h0000, 3'b010, 13'h0};


				tx_state				<= TX_STATE_HEADER_4;
			end	//end TX_STATE_HEADER_3

			//Checksum is valid at this point, send it
			TX_STATE_HEADER_4: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.bytes_valid	<= 4;
				tx_l2_bus.data			<= {8'hff, tx_l3_protocol, tx_header_checksum};

				tx_state				<= TX_STATE_HEADER_5;
			end	//end TX_STATE_HEADER_4

			//Send our IP and start reading the first upper-level protocol word
			TX_STATE_HEADER_5: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.bytes_valid	<= 4;
				tx_l2_bus.data			<= our_ip_address;

				tx_fifo_rd				<= 1;

				tx_state				<= TX_STATE_HEADER_6;

			end	//end TX_STATE_HEADER_5

			//Send destination IP and start reading second upper-level protocol word
			TX_STATE_HEADER_6: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.bytes_valid	<= 4;
				tx_l2_bus.data			<= tx_l3_dst_ip;

				tx_fifo_rd				<= 1;

				tx_state				<= TX_STATE_BODY;

			end	//end TX_STATE_HEADER_6

			//Send the current payload block
			TX_STATE_BODY: begin

				//Send data
				tx_l2_bus.data_valid		<= 1;
				tx_l2_bus.data				<= tx_fifo_rdata;

				//We already have a block being read. If that's not the last one, read another.
				if(tx_bytes_left > 8)
					tx_fifo_rd				<= 1;

				//Send not-last block
				if(tx_bytes_left > 4) begin
					tx_bytes_left			<= tx_bytes_left - 16'd4;
					tx_l2_bus.bytes_valid	<= 4;
				end

				//Send last (potentially partial) block
				else begin
					tx_bytes_left			<= 0;					//not necessary as nothing checks it after this
																//but makes debug traces cleaner
					tx_l2_bus.bytes_valid	<= tx_bytes_left;
					tx_state				<= TX_STATE_COMMIT;
				end

			end	//end TX_STATE_BODY

			TX_STATE_COMMIT: begin
				tx_l2_bus.commit	<= 1;
				tx_state			<= TX_STATE_IDLE;
			end	//end TX_STATE_COMMIT

		endcase

		//At any time, if we abort the message in progress, reset stuff
		if(tx_l3_bus.drop) begin
			tx_fifo_rst				<= 1;
			tx_state				<= TX_STATE_IDLE;
			tx_l2_bus.drop			<= 1;
		end

	end

endmodule
