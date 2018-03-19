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

module IPv4Protocol(

	//Clocks
	input wire			clk,

	//Constant-ish state data
	input wire[31:0]	our_ip_address,
	input wire[31:0]	our_subnet_mask,
	input wire[31:0]	our_broadcast_addr,

	//Incoming Ethernet data
	input wire			rx_l2_start,
	input wire			rx_l2_data_valid,
	input wire[2:0]		rx_l2_bytes_valid,
	input wire[31:0]	rx_l2_data,
	input wire			rx_l2_commit,
	input wire			rx_l2_drop,
	input wire			rx_l2_headers_valid,
	input wire			rx_l2_ethertype_is_ipv4,

	//Outbound data (same clock domain as incoming)
	output reg			tx_l2_start			= 0,
	output reg			tx_l2_data_valid	= 0,
	output reg[2:0]		tx_l2_bytes_valid	= 0,
	output reg[31:0]	tx_l2_data			= 0,
	output reg			tx_l2_commit		= 0,
	output reg			tx_l2_drop			= 0,
	output reg[47:0]	tx_l2_dst_mac		= 0,
	//TX src MAC is implied, it's always us

	//Interface to upper level protocol
	output reg			rx_l3_start					= 0,
	output reg[15:0]	rx_l3_payload_len			= 0,	//size of upper layer payload only
															//(not the IP datagram length)
	output reg[7:0]		rx_l3_protocol				= 0,
	output reg			rx_l3_protocol_is_icmp		= 0,
	output reg			rx_l3_protocol_is_udp		= 0,
	output reg			rx_l3_protocol_is_tcp		= 0,
	output reg[31:0]	rx_l3_src_ip				= 0,
	output reg[31:0]	rx_l3_dst_ip				= 0,
	output reg			rx_l3_data_valid			= 0,
	output reg[2:0]		rx_l3_bytes_valid			= 0,
	output reg[31:0]	rx_l3_data					= 0,
	output reg			rx_l3_commit				= 0,
	output reg			rx_l3_drop					= 0,
	output reg			rx_l3_headers_valid			= 0

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

	reg[15:0]	rx_header_checksum_expected	= 0;

	reg[23:0]	rx_header_checksum_temp		= 0;

	always @(posedge clk) begin

		//Clear things
		if(rx_l2_start)
			rx_header_checksum_temp		<= 0;

		if(rx_l2_data_valid && (rx_state != RX_STATE_BODY) ) begin

			//Add in the new message data
			//Do NOT add in the checksum field
			rx_header_checksum_temp = rx_header_checksum_temp + rx_l2_data[31:16];
			if(rx_state != RX_STATE_HEADER_3)
				rx_header_checksum_temp = rx_header_checksum_temp + rx_l2_data[15:0];

			//Add in the carry
			rx_header_checksum_temp		= rx_header_checksum_temp[15:0] + rx_header_checksum_temp[23:16];

		end

	end

	always @(*) begin
		rx_header_checksum_expected		<= ~rx_header_checksum_temp[15:0];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX datapath

	//Packet state that we don't currently use
	//May eventually be brought up to top-level ports
	reg[5:0]	rx_diffserv_point		= 0;
	reg[1:0]	rx_ecn					= 0;
	reg[15:0]	rx_identifier			= 0;
	reg			rx_flag_evil			= 0;
	reg			rx_flag_df				= 0;
	reg			rx_flag_mf				= 0;
	reg[12:0]	rx_frag_offset			= 0;
	reg[7:0]	rx_ttl					= 0;

	reg[15:0]	rx_header_checksum			= 0;

	reg[15:0]	payload_bytes_so_far	= 0;

	wire[15:0]	payload_bytes_next		= payload_bytes_so_far + rx_l2_bytes_valid;

	always @(posedge clk) begin

		//Forward flags from layer 2 by default
		rx_l3_start			<= rx_l2_start;
		rx_l3_drop			<= rx_l2_drop;

		rx_l3_data_valid	<= 0;
		rx_l3_bytes_valid	<= 0;
		rx_l3_commit		<= 0;
		rx_l3_headers_valid	<= 0;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for a new packet to come in

			RX_STATE_IDLE: begin

				if(rx_l2_start) begin
					rx_header_checksum		<= 0;
					payload_bytes_so_far	<= 0;
					rx_state				<= RX_STATE_HEADER_0;
				end

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// HEADER - read the packet headers and calculate the checksum

			RX_STATE_HEADER_0: begin

				//Wait for layer 2 headers to be read
				//Drop anything that's not an IPv4 packet
				if(rx_l2_headers_valid) begin
					if(rx_l2_ethertype_is_ipv4)
						rx_state	<= RX_STATE_HEADER_1;
					else begin
						rx_state	<= RX_STATE_IDLE;
						rx_l3_drop	<= 1;
					end
				end

			end	//end RX_STATE_HEADER_0

			RX_STATE_HEADER_1: begin

				if(rx_l2_data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bytes_valid != 4) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_drop				<= 1;
					end

					//Check version and header length.
					//We don't support options so drop anything with them
					else if(rx_l2_data[31:24] != {4'h4, 4'h5}) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_drop				<= 1;
					end

					else begin
						rx_diffserv_point		<= rx_l2_data[23:18];
						rx_ecn					<= rx_l2_data[17:16];

						//Total length must be at least 20 bytes
						//(minimum IP header size)
						if(rx_l2_data[15:0] < 16'd20) begin
							rx_state			<= RX_STATE_IDLE;
							rx_l3_drop			<= 1;
						end

						//Valid, figure out how big the payload is
						else begin
							rx_l3_payload_len	<= rx_l2_data[15:0] - 16'd20;
							rx_state			<= RX_STATE_HEADER_2;
						end
					end

				end

			end	//end RX_STATE_HEADER_1

			RX_STATE_HEADER_2: begin

				if(rx_l2_data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bytes_valid != 4) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_drop				<= 1;
					end

					//We don't support fragmentation.
					//Drop anything with MF or frag offset nonzero.
					else if(rx_l2_data[13] || (rx_l2_data[12:0] != 0) ) begin
						rx_state				<= RX_STATE_IDLE;
						rx_l3_drop				<= 1;
					end

					//Save fragmentation/flag data
					else begin
						rx_identifier			<= rx_l2_data[31:16];
						rx_flag_evil			<= rx_l2_data[15];
						rx_flag_df				<= rx_l2_data[14];
						rx_flag_mf				<= rx_l2_data[13];
						rx_frag_offset			<= rx_l2_data[12:0];

						rx_state				<= RX_STATE_HEADER_3;
					end

				end

			end	//end RX_STATE_HEADER_2

			RX_STATE_HEADER_3: begin

				if(rx_l2_data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						rx_l3_drop					<= 1;
					end

					else begin
						rx_ttl						<= rx_l2_data[31:24];
						rx_l3_protocol				<= rx_l2_data[23:16];
						rx_l3_protocol_is_icmp		<= (rx_l2_data[23:16] == IP_PROTO_ICMP);
						rx_l3_protocol_is_udp		<= (rx_l2_data[23:16] == IP_PROTO_UDP);
						rx_l3_protocol_is_tcp		<= (rx_l2_data[23:16] == IP_PROTO_TCP);
						rx_header_checksum			<= rx_l2_data[15:0];

						rx_state					<= RX_STATE_HEADER_4;
					end

				end

			end	//end RX_STATE_HEADER_3

			RX_STATE_HEADER_4: begin

				if(rx_l2_data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						rx_l3_drop					<= 1;
					end

					else begin
						rx_l3_src_ip				<= rx_l2_data;
						rx_state					<= RX_STATE_HEADER_5;
					end

				end

			end	//end RX_STATE_HEADER_4

			RX_STATE_HEADER_5: begin

				if(rx_l2_data_valid) begin

					//Abort on truncated packet
					if(rx_l2_bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						rx_l3_drop					<= 1;
					end

					else begin
						rx_l3_dst_ip				<= rx_l2_data;

						//See if the packet is intended for us
						//This means either our unicast address, our subnet's broadcast address,
						//or the global broadcast address.
						if( (rx_l2_data == our_ip_addr) ||
							(rx_l2_data == our_broadcast_addr) ||
							(rx_l2_data == 32'hffffffff) ) begin

							rx_state					<= RX_STATE_BODY;

						end

						//Nope, it's for somebody else. Drop it.
						else begin
							rx_state					<= RX_STATE_IDLE;
							rx_l3_drop					<= 1;
						end

					end

				end

			end	//end RX_STATE_HEADER_5

			RX_STATE_BODY: begin

				//Forward packet data
				if(rx_l2_data_valid) begin
					rx_l3_data_valid		<= 1;
					rx_l3_bytes_valid		<= rx_l2_bytes_valid;
					rx_l3_data				<= rx_l2_data;

					payload_bytes_so_far	<= payload_bytes_next;

					//If we've reached the end of the packet body, anything afterward is padding
					//Ignore it. Also, tell upper level protocol to do so
					if(payload_bytes_next >= rx_l3_payload_len) begin
						rx_state			<= RX_STATE_PADDING;
						rx_l3_bytes_valid	<= rx_l3_payload_len - payload_bytes_so_far;
					end

				end

				//Verify header checksum
				if(payload_bytes_so_far == 0) begin

					if(rx_header_checksum != rx_header_checksum_expected) begin
						rx_state					<= RX_STATE_IDLE;
						rx_l3_drop					<= 1;
					end

				end

				//At the end of the packet, commit upstream and go back to idle
				else if(rx_l2_commit) begin
					rx_l3_commit		<= 1;
					rx_state			<= RX_STATE_IDLE;
				end

			end	//end RX_STATE_BODY

			//Ignore padding, wait for end of packet
			RX_STATE_PADDING: begin

				//At the end of the packet, commit upstream and go back to idle
				if(rx_l2_commit) begin
					rx_l3_commit		<= 1;
					rx_state			<= RX_STATE_IDLE;
				end

			end	//end RX_STATE_PADDING

		endcase

		//If we get a drop request from the MAC, abort everything
		if(rx_l2_drop)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX datapath

endmodule
