`timescale 1ns / 1ps
`default_nettype none
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

`include "EthernetBus.svh"
`include "IPv4Bus.svh"
`include "TCPv4Bus.svh"

module TCPProtocol(

	//Clocks
	input wire				clk,

	//Incoming data bus from IP stack
	//TODO: make this parameterizable for IPv4/IPv6, for now we only do v4
	input wire IPv4RxBus	rx_l3_bus,
	output IPv4TxBus		tx_l3_bus	=	{$bits(IPv4TxBus){1'b0}},

	//Outbound bus to applications
	output TCPv4RxBus		rx_l4_bus	=	{$bits(TCPv4RxBus){1'b0}},
	input wire TCPv4TxBus	tx_l4_bus
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Socket state

	//The SocketManager stores mappings from (sport, dport, client_ip) to socket handles.
	//We need to store other state ourselves.

	logic			lookup_en	= 0;
	socketstate_t	lookup_headers	= 0;
	wire			lookup_done;
	wire			lookup_hit;
	wire[10:0]		lookup_sockid;

	SocketManager #(
		.WAYS(4),
		.LATENCY(2),
		.BINS(256)
	) state_mgr (
		.clk(clk),

		.lookup_en(lookup_en),
		.lookup_headers(lookup_headers),
		.lookup_done(lookup_done),
		.lookup_hit(lookup_hit),
		.lookup_sockid(lookup_sockid),

		.insert_en(),
		.insert_headers(),
		.insert_done(),
		.insert_fail(),
		.insert_sockid(),

		.remove_en(),
		.remove_sockid(),
		.remove_done(),

		.aging_tick(),
		.max_age()
	);

	typedef struct
	{
		logic[31:0]	tx_seq;		//Starting sequence number for our next outbound segment
		logic[31:0]	rx_seq;		//Expected sequence number of the next inbound segment
		logic[15:0] tx_window;	//max window we're able to send
	} tcpstate_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX checksum verification

	wire[15:0]	rx_checksum_expected;

	InternetChecksum32bit rx_csum(
		.clk(clk),
		.load(rx_l3_bus.headers_valid),
		.reset(1'b0),
		.process(rx_l3_bus.data_valid),
		.din(rx_l3_bus.data_valid ? rx_l3_bus.data : rx_l3_bus.pseudo_header_csum),
		.sumout(),
		.csumout(rx_checksum_expected)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX datapath

	enum logic[3:0]
	{
		RX_STATE_IDLE				= 4'h0,
		RX_STATE_IP_HEADER			= 4'h1,
		RX_STATE_PORT_HEADER		= 4'h2,
		RX_STATE_SEQ_HEADER			= 4'h3,
		RX_STATE_ACK_HEADER			= 4'h4,
		RX_STATE_FLAG_HEADER		= 4'h5,
		RX_STATE_CHECKSUM_HEADER	= 4'h6,
		RX_STATE_OPTION_HEADER		= 4'h7,
		RX_STATE_DATA				= 4'h8,
		RX_STATE_CHECKSUM			= 4'h9
	} rx_state = RX_STATE_IDLE;

	logic[31:0]	rx_current_seq		= 0;
	logic[31:0]	rx_current_ack		= 0;
	logic[15:0]	rx_current_window	= 0;
	logic[3:0]	rx_data_offset		= 0;
	logic		rx_flag_ack			= 0;
	logic		rx_flag_syn			= 0;
	logic		rx_flag_fin			= 0;
	logic		rx_flag_rst			= 0;

	logic[3:0]	rx_current_option	= 0;

	always_ff @(posedge clk) begin

		lookup_en	<= 0;

		rx_l4_bus.start			<= 0;
		rx_l4_bus.data_valid	<= 0;
		rx_l4_bus.commit		<= 0;
		rx_l4_bus.drop			<= 0;

		case(rx_state)

			//Start when we get a new packet
			RX_STATE_IDLE: begin

				if(rx_l3_bus.start)
					rx_state			<= RX_STATE_IP_HEADER;

			end	//end RX_STATE_IDLE

			RX_STATE_IP_HEADER: begin

				if(rx_l3_bus.headers_valid) begin
					if( (rx_l3_bus.payload_len < 20) || !rx_l3_bus.protocol_is_tcp )
						rx_state			<= RX_STATE_IDLE;
					else
						rx_state			<= RX_STATE_PORT_HEADER;

				end

			end	//end RX_STATE_IP_HEADER

			RX_STATE_PORT_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						rx_l4_bus.start				<= 1;
						rx_l4_bus.src_port			<= rx_l3_bus.data[31:16];
						rx_l4_bus.dst_port			<= rx_l3_bus.data[15:0];
						rx_state					<= RX_STATE_SEQ_HEADER;

						//At this point we have all of the info we need to look up the socket and see if
						//this is an active session.
						lookup_en					<= 1;
						lookup_headers.address		<= rx_l3_bus.src_ip;
						lookup_headers.client_port	<= rx_l3_bus.data[31:16];
						lookup_headers.server_port	<= rx_l3_bus.data[15:0];

					end

				end
			end	//end RX_STATE_PORT_HEADER

			RX_STATE_SEQ_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					else begin
						rx_current_seq	<= rx_l3_bus.data;
						rx_state		<= RX_STATE_ACK_HEADER;
					end

				end
			end	//end RX_STATE_SEQ_HEADER

			RX_STATE_ACK_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					else begin
						rx_current_ack	<= rx_l3_bus.data;
						rx_state		<= RX_STATE_FLAG_HEADER;
					end

				end
			end	//end RX_STATE_ACK_HEADER

			RX_STATE_FLAG_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					else begin

						rx_current_window	<= rx_l3_bus.data[15:0];
						rx_data_offset		<= rx_l3_bus.data[31:28];

						//27:25 reserved
						//24 = NS (not supported)
						//23 = CWR (not supported)
						//22 = ECE (not supported)
						//21 = URG (TODO)
						rx_flag_ack			<= rx_l3_bus.data[20];
						//19 = PSH (ignored, we always push)
						rx_flag_rst			<= rx_l3_bus.data[18];
						rx_flag_syn			<= rx_l3_bus.data[17];
						rx_flag_fin			<= rx_l3_bus.data[16];

						rx_state			<= RX_STATE_CHECKSUM_HEADER;
					end

				end
			end	//end RX_STATE_FLAG_HEADER

			RX_STATE_CHECKSUM_HEADER: begin

				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					//valid data
					else begin

						//current data is checksum and urgent pointer, we don't care about either
						//so just ignore them

						//No options, jump right into data
						if(rx_data_offset <= 5)
							rx_state	<= RX_STATE_DATA;

						//We have options
						else begin
							rx_state			<= RX_STATE_OPTION_HEADER;
							rx_current_option	<= 6;
						end

					end
				end
			end	//end RX_STATE_CHECKSUM_HEADER

			RX_STATE_OPTION_HEADER: begin

				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					//Keep count of options
					else begin
						rx_current_option	<= rx_current_option + 1;

						if(rx_current_option == rx_data_offset)
							rx_state	<= RX_STATE_DATA;
					end

				end

			end	//end RX_STATE_OPTION_HEADER

			RX_STATE_DATA: begin
				if(rx_l3_bus.commit) begin
					rx_state		<= RX_STATE_CHECKSUM;
				end
			end	//end RX_STATE_DATA

			//Verify checksum
			RX_STATE_CHECKSUM: begin

				if(rx_checksum_expected == 16'h0000) begin
					rx_l4_bus.commit	<= 1;
					rx_state			<= RX_STATE_IDLE;
				end
				else begin
					rx_l4_bus.drop		<= 1;
					rx_state			<= RX_STATE_IDLE;
				end

			end	//end RX_STATE_CHECKSUM

		endcase

		//Handle drops
		if(rx_l3_bus.drop) begin
			rx_state		<= RX_STATE_IDLE;
			rx_l4_bus.drop	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX datapath

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug stuff

	ila_0 ila(
		.clk(clk),
		.probe0(rx_l3_bus),
		.probe1(rx_l4_bus),
		.probe2(rx_state),
		.probe3(lookup_en),
		.probe4(lookup_headers),
		.probe5(lookup_done),
		.probe6(lookup_hit),
		.probe7(lookup_sockid),
		.probe8(rx_current_seq),
		.probe9(rx_current_ack),
		.probe10(rx_current_window),
		.probe11(rx_data_offset),
		.probe12(rx_flag_ack),
		.probe13(rx_flag_rst),
		.probe14(rx_flag_syn),
		.probe15(rx_flag_fin),
		.probe16(rx_current_option),
		.probe17(rx_checksum_expected)
		);

endmodule
