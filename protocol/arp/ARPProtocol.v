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

module ARPProtocol(

	//Clocks
	input wire			clk,

	//Constant-ish state data
	input wire[47:0]	our_mac_address,
	input wire[31:0]	our_ip_address,

	//Incoming Ethernet data
	input wire			rx_l2_start,
	input wire			rx_l2_data_valid,
	input wire[2:0]		rx_l2_bytes_valid,
	input wire[31:0]	rx_l2_data,
	input wire			rx_l2_commit,
	input wire			rx_l2_drop,
	input wire			rx_l2_headers_valid,
	input wire			rx_l2_ethertype_is_arp,

	//Outbound data (same clock domain as incoming)
	output reg			tx_l2_start			= 0,
	output reg			tx_l2_data_valid	= 0,
	output reg[2:0]		tx_l2_bytes_valid	= 0,
	output reg[31:0]	tx_l2_data			= 0,
	output reg			tx_l2_commit		= 0,
	output reg			tx_l2_drop			= 0,
	output reg[47:0]	tx_l2_dst_mac		= 0
	//TX src MAC is implied, it's always us

	//TODO: performance counters
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Process incoming ARP requests

	`include "../../interface/ethernet/Ethertypes.vh"

	localparam RX_STATE_IDLE			= 4'h0;
	localparam RX_STATE_L2_HEADER		= 4'h1;
	localparam RX_STATE_BODY_0			= 4'h2;
	localparam RX_STATE_BODY_1			= 4'h3;
	localparam RX_STATE_BODY_2			= 4'h4;
	localparam RX_STATE_BODY_3			= 4'h5;
	localparam RX_STATE_BODY_4			= 4'h6;
	localparam RX_STATE_BODY_5			= 4'h7;
	localparam RX_STATE_BODY_6			= 4'h8;
	localparam RX_STATE_COMMIT			= 4'h9;

	localparam ARP_OP_REQUEST			= 16'h1;
	localparam ARP_OP_REPLY				= 16'h2;

	reg[3:0]	rx_state				= RX_STATE_IDLE;

	reg			rx_packet_is_request	= 0;
	reg[47:0]	rx_sender_mac_addr		= 0;
	reg[31:0]	rx_sender_ip_addr		= 0;
	reg[47:0]	rx_target_mac_addr		= 0;
	reg[31:0]	rx_target_ip_addr		= 0;

	always @(posedge clk) begin

		//Forward start/end flags to transmit side
		tx_l2_start			<= rx_l2_start;
		tx_l2_drop			<= rx_l2_drop;

		//Clear flags
		tx_l2_data_valid	<= 0;
		tx_l2_commit		<= 0;

		//Any time we send anything it's a full 4 bytes
		tx_l2_bytes_valid	<= 4;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - sit around and wait for a new incoming ARP request

			RX_STATE_IDLE: begin

				if(rx_l2_start)
					rx_state	<= RX_STATE_L2_HEADER;

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// L2 HEADER - wait for the layer-2 headers to be valid

			RX_STATE_L2_HEADER: begin

				if(rx_l2_headers_valid)
					rx_state	<= RX_STATE_BODY_0;

			end	//end RX_STATE_L2_HEADER

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BODY - crunch incoming data

			//HTYPE, PTYPE
			RX_STATE_BODY_0: begin

				if(rx_l2_data_valid) begin

					//Expect HTYPE=Ethernet, PTYPE=IPv4. Ignore anything else
					if( (rx_l2_bytes_valid != 4) || (rx_l2_data != { 16'h01, ETHERTYPE_IPV4} ) )
						rx_state			<= RX_STATE_IDLE;

					else
						rx_state			<= RX_STATE_BODY_1;

					//Start sending our reply
					tx_l2_data_valid	<= 1;
					tx_l2_data			<= { 16'h01, ETHERTYPE_IPV4};

				end

			end	//end RX_STATE_BODY_0

			//HLEN, PLEN, OPER
			RX_STATE_BODY_1: begin

				if(rx_l2_data_valid) begin

					rx_state			<= RX_STATE_BODY_2;
					tx_l2_data_valid	<= 1;

					//Expect HLEN = 48 bits, PLEN = 32 bits. Drop anything else
					if( (rx_l2_bytes_valid != 4) || (rx_l2_data[31:16] != {16'h0604}) )
						rx_state		<= RX_STATE_IDLE;

					//Expect OPER = request/reply
					else if(rx_l2_data[15:0] == ARP_OP_REQUEST) begin
						rx_packet_is_request	<= 1;
						tx_l2_data				<= { 16'h0604, ARP_OP_REPLY };
					end
					else if(rx_l2_data[15:0] == ARP_OP_REPLY) begin
						rx_packet_is_request	<= 0;

						//we don't send anything in response to a reply
						tx_l2_drop				<= 1;
					end

					//Nope, invalid operation - drop it
					else
						rx_state	<= RX_STATE_IDLE;

				end

			end	//end RX_STATE_BODY_1

			//SHA (first 4 bytes)
			RX_STATE_BODY_2: begin

				if(rx_l2_data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						rx_sender_mac_addr[47:16]	<= rx_l2_data;
						rx_state					<= RX_STATE_BODY_3;

						//When replying to an incoming request, send our MAC
						if(rx_packet_is_request) begin
							tx_l2_data_valid		<= 1;
							tx_l2_data				<= our_mac_address[47:16];
						end

					end

				end

			end	//end RX_STATE_BODY_2

			//SHA (last 2 bytes) plus SPA, (first 2 bytes)
			RX_STATE_BODY_3: begin

				if(rx_l2_data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						rx_sender_mac_addr[15:0]	<= rx_l2_data[31:16];
						rx_sender_ip_addr[31:16]	<= rx_l2_data[15:0];
						rx_state					<= RX_STATE_BODY_4;

						//When replying to an incoming request, send our addresses
						if(rx_packet_is_request) begin
							tx_l2_data_valid		<= 1;
							tx_l2_data				<= { our_mac_address[15:0], our_ip_address[31:16] };
						end
					end

				end

			end	//end RX_STATE_BODY_3

			//SPA (last 2 bytes) plus THA, (first 2 bytes)
			RX_STATE_BODY_4: begin

				if(rx_l2_data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						rx_sender_ip_addr[15:0]		<= rx_l2_data[31:16];
						rx_target_mac_addr[47:32]	<= rx_l2_data[15:0];
						rx_state					<= RX_STATE_BODY_5;

						//When replying to an incoming request, send our address and reply to the sender
						if(rx_packet_is_request) begin
							tx_l2_data_valid		<= 1;
							tx_l2_data				<= { our_ip_address[15:0], rx_sender_mac_addr[47:32] };

							tx_l2_dst_mac			<= rx_sender_mac_addr;
						end

					end

				end

			end	//end RX_STATE_BODY_4

			//THA (last 4 bytes)
			RX_STATE_BODY_5: begin

				if(rx_l2_data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						rx_target_mac_addr[31:0]	<= rx_l2_data;
						rx_state					<= RX_STATE_BODY_6;

						//When replying to an incoming request, reply to the sender
						if(rx_packet_is_request) begin
							tx_l2_data_valid		<= 1;
							tx_l2_data				<= rx_sender_mac_addr[31:0];
						end

					end

				end

			end	//end RX_STATE_BODY_5

			//TPA
			RX_STATE_BODY_6: begin

				if(rx_l2_data_valid) begin

					//As usual, bail out on truncated packets
					if(rx_l2_bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						rx_target_ip_addr			<= rx_l2_data;
						rx_state					<= RX_STATE_COMMIT;

						//When replying to an incoming request, reply to the sender
						if(rx_packet_is_request) begin
							tx_l2_data_valid		<= 1;
							tx_l2_data				<= rx_sender_ip_addr;
						end

					end

				end

			end	//end RX_STATE_BODY_6

			//Packet should end at this point.
			//Ignore padding at the end of the packet
			//(ARP is below the Ethernet MTU so we need some trailing padding)
			RX_STATE_COMMIT: begin

				//Is the packet asking for somebody else other than us? Drop it
				if(rx_target_ip_addr != our_ip_address) begin
					rx_state		<= RX_STATE_IDLE;
					tx_l2_drop	<= 1;
				end

				//Valid, well-formed packet? Go ahead and send our reply
				if(rx_l2_commit) begin
					rx_state		<= RX_STATE_IDLE;
					tx_l2_commit	<= 1;
				end

			end

		endcase

		//At any time, if we get a "drop" request, cancel work in progress
		if(rx_l2_drop)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LA for debugging things

	/*
	wire	trig_out;

	reg		trig_out_ack 	= 0;
	always @(posedge clk) begin
		trig_out_ack	<= trig_out;
	end

	ila_0 ila(
		.clk(clk),

		.probe0(rx_l2_start),
		.probe1(rx_l2_data_valid),
		.probe2(rx_l2_bytes_valid),
		.probe3(rx_l2_data),
		.probe4(rx_l2_commit),
		.probe5(rx_l2_drop),
		.probe6(rx_l2_headers_valid),
		.probe7(rx_l2_ethertype_is_arp),

		.probe8(tx_l2_start),
		.probe9(tx_l2_data_valid),
		.probe10(tx_l2_bytes_valid),
		.probe11(tx_l2_data),
		.probe12(tx_l2_commit),
		.probe13(tx_l2_drop),
		.probe14(tx_l2_dst_mac),

		.probe15(rx_state),
		.probe16(rx_packet_is_request),
		.probe17(rx_sender_mac_addr),
		.probe18(rx_sender_ip_addr),
		.probe19(rx_target_mac_addr),
		.probe20(rx_target_ip_addr),

		.trig_out(trig_out),
		.trig_out_ack(trig_out_ack)
	);
	*/

endmodule
