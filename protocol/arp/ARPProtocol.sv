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

module ARPProtocol(

	//Clocks
	input wire					clk,

	//Constant-ish state data
	input wire[47:0]			our_mac_address,
	input wire[31:0]			our_ip_address,

	//Incoming Ethernet data
	input wire EthernetRxL2Bus	rx_l2_bus,

	//Outbound data (same clock domain as incoming)
	output EthernetTxL2Bus		tx_l2_bus			= {1'h0, 1'h0, 1'h0, 32'h0, 48'h0, 16'h0, 1'h0, 1'h0},

	//New mappings we've learned (to cache)
	output logic				learn_valid	= 0,
	output logic[31:0]			learn_ip	= 0,
	output logic[47:0]			learn_mac	= 0,

	//Request to send an ARP query
	input wire					query_en,
	input wire[31:0]			query_ip

	//TODO: performance counters
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO of IPs that we need to send ARP queries for

	wire		query_fifo_empty;

	logic		query_fifo_rd	= 0;
	logic[31:0]	query_fifo_ip;

	SingleClockFifo #(
		.WIDTH(32),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) query_addr_fifo (
		.clk(clk),
		.wr(query_en),
		.din(query_ip),

		.rd(query_fifo_rd),
		.dout(query_fifo_ip),
		.overflow(),
		.underflow(),
		.empty(query_fifo_empty),
		.full(),
		.rsize(),
		.wsize(),
		.reset()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Process incoming ARP requests

	`include "../../interface/ethernet/Ethertypes.vh"

	enum logic[4:0]
	{
		RX_STATE_IDLE		= 5'h00,
		RX_STATE_L2_HEADER	= 5'h01,
		RX_STATE_BODY_0		= 5'h02,
		RX_STATE_BODY_1		= 5'h03,
		RX_STATE_BODY_2		= 5'h04,
		RX_STATE_BODY_3		= 5'h05,
		RX_STATE_BODY_4		= 5'h06,
		RX_STATE_BODY_5		= 5'h07,
		RX_STATE_BODY_6		= 5'h08,
		RX_STATE_COMMIT		= 5'h09,
		RX_STATE_QUERY_0	= 5'h0a,
		RX_STATE_QUERY_1	= 5'h0b,
		RX_STATE_QUERY_2	= 5'h0c,
		RX_STATE_QUERY_3	= 5'h0d,
		RX_STATE_QUERY_4	= 5'h0e,
		RX_STATE_QUERY_5	= 5'h0f,
		RX_STATE_QUERY_6	= 5'h10,
		RX_STATE_QUERY_7	= 5'h11
	} rx_state	= RX_STATE_IDLE;

	localparam ARP_OP_REQUEST			= 16'h1;
	localparam ARP_OP_REPLY				= 16'h2;

	reg			rx_packet_is_request	= 0;
	reg[47:0]	rx_sender_mac_addr		= 0;
	reg[31:0]	rx_sender_ip_addr		= 0;
	reg[47:0]	rx_target_mac_addr		= 0;
	reg[31:0]	rx_target_ip_addr		= 0;

	always_ff @(posedge clk) begin

		//Forward start/end flags to transmit side
		tx_l2_bus.start			<= rx_l2_bus.start;
		tx_l2_bus.drop			<= rx_l2_bus.drop;

		//Clear flags
		tx_l2_bus.data_valid	<= 0;
		tx_l2_bus.commit		<= 0;
		query_fifo_rd			<= 0;
		learn_valid				<= 0;

		//Any time we send anything it's a full 4 bytes
		tx_l2_bus.bytes_valid	<= 4;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - sit around and wait for a new incoming packet

			RX_STATE_IDLE: begin

				if(rx_l2_bus.start)
					rx_state	<= RX_STATE_L2_HEADER;

				//We need to send an ARP query.
				//For now, incoming ARP requests will be ignored for 8 clocks while we do this.
				//TODO: tiny FIFO on inbound packets to prevent loss, or buffer+arbiter on outbound traffic
				else if(!query_fifo_empty) begin
					tx_l2_bus.start		<= 1;
					query_fifo_rd		<= 1;
					rx_state			<= RX_STATE_QUERY_0;

					tx_l2_bus.dst_mac	<= 48'hff_ff_ff_ff_ff_ff;
				end

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// QUERY - send an ARP request

			//HTYPE, PTYPE
			RX_STATE_QUERY_0: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= { 16'h01, ETHERTYPE_IPV4};
				rx_state				<= RX_STATE_QUERY_1;
			end	//end RX_STATE_QUERY_0

			//HLEN, PLEN, OPER
			RX_STATE_QUERY_1: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= { 16'h0604, ARP_OP_REQUEST };
				rx_state				<= RX_STATE_QUERY_2;
			end	//end RX_STATE_QUERY_1

			//SHA high
			RX_STATE_QUERY_2: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= our_mac_address[47:16];
				rx_state				<= RX_STATE_QUERY_3;
			end	//end RX_STATE_QUERY_2

			//SHA low, SPA high
			RX_STATE_QUERY_3: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= { our_mac_address[15:0], our_ip_address[31:16] };
				rx_state				<= RX_STATE_QUERY_4;
			end	//end RX_STATE_QUERY_3

			//SPA low, THA high
			RX_STATE_QUERY_4: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= { our_ip_address[15:0], 16'h0000 };
				rx_state				<= RX_STATE_QUERY_5;
			end	//end RX_STATE_QUERY_4

			//THA low
			RX_STATE_QUERY_5: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= 32'h00000000;
				rx_state				<= RX_STATE_QUERY_6;
			end	//end RX_STATE_QUERY_5

			//TPA
			RX_STATE_QUERY_6: begin
				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= query_fifo_ip;
				rx_state				<= RX_STATE_QUERY_7;
			end	//end RX_STATE_QUERY_6

			//Done
			RX_STATE_QUERY_7: begin
				tx_l2_bus.commit		<= 1;
				rx_state				<= RX_STATE_IDLE;
			end	//end RX_STATE_QUERY_7

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// L2 HEADER - wait for the layer-2 headers to be valid. Discard any other ethertypes.

			RX_STATE_L2_HEADER: begin

				if(rx_l2_bus.headers_valid) begin
					if(rx_l2_bus.ethertype_is_arp)
						rx_state	<= RX_STATE_BODY_0;
					else begin
						tx_l2_bus.drop	<= 1;
						rx_state	<= RX_STATE_IDLE;
					end
				end

			end	//end RX_STATE_L2_HEADER

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BODY - crunch incoming data

			//HTYPE, PTYPE
			RX_STATE_BODY_0: begin

				if(rx_l2_bus.data_valid) begin

					//Expect HTYPE=Ethernet, PTYPE=IPv4. Ignore anything else
					if( (rx_l2_bus.bytes_valid != 4) || (rx_l2_bus.data != { 16'h01, ETHERTYPE_IPV4} ) ) begin
						rx_state				<= RX_STATE_IDLE;
						tx_l2_bus.drop			<= 1;
					end

					else begin
						rx_state				<= RX_STATE_BODY_1;

						//Start sending our reply
						tx_l2_bus.data_valid	<= 1;
						tx_l2_bus.data			<= { 16'h01, ETHERTYPE_IPV4};
					end

				end

			end	//end RX_STATE_BODY_0

			//HLEN, PLEN, OPER
			RX_STATE_BODY_1: begin

				if(rx_l2_bus.data_valid) begin

					rx_state					<= RX_STATE_BODY_2;

					//Expect HLEN = 48 bits, PLEN = 32 bits. Drop anything else
					if( (rx_l2_bus.bytes_valid != 4) || (rx_l2_bus.data[31:16] != {16'h0604}) ) begin
						rx_state				<= RX_STATE_IDLE;
						tx_l2_bus.drop			<= 1;
					end

					//Expect OPER = request/reply
					else if(rx_l2_bus.data[15:0] == ARP_OP_REQUEST) begin
						rx_packet_is_request	<= 1;
						tx_l2_bus.data_valid	<= 1;
						tx_l2_bus.data			<= { 16'h0604, ARP_OP_REPLY };
					end
					else if(rx_l2_bus.data[15:0] == ARP_OP_REPLY) begin
						rx_packet_is_request	<= 0;

						//we don't send anything in response to a reply
						tx_l2_bus.drop			<= 1;
					end

					//Nope, invalid operation - drop it
					else
						rx_state	<= RX_STATE_IDLE;

				end

			end	//end RX_STATE_BODY_1

			//SHA (first 4 bytes)
			RX_STATE_BODY_2: begin

				if(rx_l2_bus.data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						tx_l2_bus.drop				<= 1;
					end

					else begin
						rx_sender_mac_addr[47:16]	<= rx_l2_bus.data;
						rx_state					<= RX_STATE_BODY_3;

						//When replying to an incoming request, send our MAC
						if(rx_packet_is_request) begin
							tx_l2_bus.data_valid	<= 1;
							tx_l2_bus.data			<= our_mac_address[47:16];
						end

					end

				end

			end	//end RX_STATE_BODY_2

			//SHA (last 2 bytes) plus SPA, (first 2 bytes)
			RX_STATE_BODY_3: begin

				if(rx_l2_bus.data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						tx_l2_bus.drop				<= 1;
					end

					else begin
						rx_sender_mac_addr[15:0]	<= rx_l2_bus.data[31:16];
						rx_sender_ip_addr[31:16]	<= rx_l2_bus.data[15:0];
						rx_state					<= RX_STATE_BODY_4;

						//When replying to an incoming request, send our addresses
						if(rx_packet_is_request) begin
							tx_l2_bus.data_valid	<= 1;
							tx_l2_bus.data			<= { our_mac_address[15:0], our_ip_address[31:16] };
						end
					end

				end

			end	//end RX_STATE_BODY_3

			//SPA (last 2 bytes) plus THA, (first 2 bytes)
			RX_STATE_BODY_4: begin

				if(rx_l2_bus.data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						tx_l2_bus.drop				<= 1;
					end

					else begin
						rx_sender_ip_addr[15:0]		<= rx_l2_bus.data[31:16];
						rx_target_mac_addr[47:32]	<= rx_l2_bus.data[15:0];
						rx_state					<= RX_STATE_BODY_5;

						//When replying to an incoming request, send our address and reply to the sender
						if(rx_packet_is_request) begin
							tx_l2_bus.data_valid	<= 1;
							tx_l2_bus.data			<= { our_ip_address[15:0], rx_sender_mac_addr[47:32] };

							tx_l2_bus.dst_mac		<= rx_sender_mac_addr;
						end

					end

				end

			end	//end RX_STATE_BODY_4

			//THA (last 4 bytes)
			RX_STATE_BODY_5: begin

				if(rx_l2_bus.data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						tx_l2_bus.drop				<= 1;
					end

					else begin
						rx_target_mac_addr[31:0]	<= rx_l2_bus.data;
						rx_state					<= RX_STATE_BODY_6;

						//When replying to an incoming request, reply to the sender
						if(rx_packet_is_request) begin
							tx_l2_bus.data_valid	<= 1;
							tx_l2_bus.data			<= rx_sender_mac_addr[31:0];
						end

					end

				end

			end	//end RX_STATE_BODY_5

			//TPA
			RX_STATE_BODY_6: begin

				if(rx_l2_bus.data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bus.bytes_valid != 4) begin
						rx_state					<= RX_STATE_IDLE;
						tx_l2_bus.drop				<= 1;
					end

					else begin
						rx_target_ip_addr			<= rx_l2_bus.data;
						rx_state					<= RX_STATE_COMMIT;

						//When replying to an incoming request, reply to the sender
						if(rx_packet_is_request) begin
							tx_l2_bus.data_valid	<= 1;
							tx_l2_bus.data			<= rx_sender_ip_addr;
						end

					end

				end

			end	//end RX_STATE_BODY_6

			//Packet should end at this point.
			//Ignore padding at the end of the packet
			//(ARP is below the Ethernet MTU so we need some trailing padding)
			RX_STATE_COMMIT: begin

				//If it's a reply, add data to the cache when the packet ends
				if(!rx_packet_is_request) begin
				end

				//Nope, it's a request
				else begin

					//Is the packet asking for somebody else other than us? Drop it
					if(rx_target_ip_addr != our_ip_address) begin
						rx_state			<= RX_STATE_IDLE;
						tx_l2_bus.drop		<= 1;
					end

					//Valid, well-formed packet? Go ahead and send our reply
					else if(rx_l2_bus.commit)
						tx_l2_bus.commit	<= 1;

				end

				//Valid, well-formed packet? Add to the table.
				//Doesn't matter who sent it, the sender's address gets learned.
				if(rx_l2_bus.commit) begin
					rx_state			<= RX_STATE_IDLE;

					learn_valid			<= 1;
					learn_ip			<= rx_sender_ip_addr;
					learn_mac			<= rx_sender_mac_addr;
				end

			end

		endcase

		//At any time, if we get a "drop" request, cancel work in progress
		if(rx_l2_bus.drop) begin
			tx_l2_bus.drop	<= 1;
			rx_state		<= RX_STATE_IDLE;
		end

	end

endmodule
