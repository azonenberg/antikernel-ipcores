`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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
	SocketManager #(
		.WAYS(4),
		.LATENCY(2),
		.BINS(256)
	) state_mgr (
		.clk(clk),

		.lookup_en(),
		.lookup_headers(),
		.lookup_done(),
		.lookup_hit(),
		.lookup_sockid(),

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
	// RX datapath

	enum logic[3:0]
	{
		RX_STATE_IDLE		= 4'h0,
		RX_STATE_HEADER_0	= 4'h1,
		RX_STATE_HEADER_1	= 4'h2,
		RX_STATE_HEADER_2	= 4'h3
	} rx_state = RX_STATE_IDLE;

	always_ff @(posedge clk) begin

		case(rx_state)

			//Start when we get a new packet
			RX_STATE_IDLE: begin

				if(rx_l3_bus.start)
					rx_state			<= RX_STATE_HEADER_0;

			end	//end RX_STATE_IDLE

			RX_STATE_HEADER_0: begin
				if(rx_l3_bus.headers_valid) begin
					if( (rx_l3_bus.payload_len < 20) || !rx_l3_bus.protocol_is_tcp )
						rx_state			<= RX_STATE_IDLE;
					else
						rx_state			<= RX_STATE_HEADER_1;

				end
			end	//end RX_STATE_HEADER_0

			RX_STATE_HEADER_1: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						rx_l4_bus.src_port	<= rx_l3_bus.data[31:16];
						rx_l4_bus.dst_port	<= rx_l3_bus.data[15:0];
						rx_state			<= RX_STATE_HEADER_2;
					end

				end
			end	//end RX_STATE_HEADER_1

			RX_STATE_HEADER_2: begin
				rx_state	<= RX_STATE_IDLE;
			end	//end RX_STATE_HEADER_2

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX datapath

endmodule
