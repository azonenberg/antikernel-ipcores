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

`include "IPv4Bus.svh"
`include "IPProtocols.vh"

module ICMPv4Protocol(

	//Clocks
	input wire				clk,

	//Incoming data bus from IP stack
	input wire IPv4RxBus	rx_l3_bus,

	//Outbound data bus to IP stack
	//(src ip and protocol are added by IP stack)
	output IPv4TxBus		tx_l3_bus			=  {$bits(IPv4TxBus){1'b0}},

	//no layer-4 bus, we handle all ICMP traffic internally
	//TODO: allow originating pings etc?

	//Performance counters
	output reg[63:0]		perf_icmp_rx		= 0,
	output reg[63:0]		perf_icmp_tx		= 0,
	output reg[63:0]		perf_icmp_csumfail	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off constant fields

	always_comb
		tx_l3_bus.protocol	<= IP_PROTO_ICMP;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Performance counters

	always @(posedge clk) begin
		if(rx_l3_bus.commit && rx_l3_bus.protocol_is_icmp)
			perf_icmp_rx	<= perf_icmp_rx + 1'h1;
		if(tx_l3_bus.commit)
			perf_icmp_tx	<= perf_icmp_tx + 1'h1;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// State machine constants

	enum logic[3:0]
	{
		RX_STATE_IDLE			= 4'h0,
		RX_STATE_HEADER_0		= 4'h1,
		RX_STATE_HEADER_1		= 4'h2,
		RX_STATE_PING_HEADER	= 4'h3,
		RX_STATE_PING_BODY		= 4'h4
	} rx_state = RX_STATE_IDLE;

	enum logic[3:0]
	{
		TX_STATE_IDLE			= 4'h0,
		TX_STATE_HEADER			= 4'h1,
		TX_STATE_BODY			= 4'h2,
		TX_STATE_COMMIT			= 4'h3
	} tx_state = TX_STATE_IDLE;

	localparam ICMP_TYPE_ECHO_REPLY		= 8'h00;
	localparam ICMP_TYPE_ECHO_REQUEST	= 8'h08;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX packet buffer

	//Header is up at the top! We need to buffer the message body before we can send the checksum

	reg			tx_fifo_rd	= 0;
	reg			tx_fifo_rst	= 0;
	wire[31:0]	tx_fifo_rdata;

	wire[9:0]	tx_fifo_rsize;

	SingleClockFifo #(
		.WIDTH(32),
		.DEPTH(512)
	) tx_fifo (
		.clk(clk),
		.wr( (rx_state == RX_STATE_PING_BODY) && rx_l3_bus.data_valid ),
		.din(rx_l3_bus.data),

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
	// RX checksum engine

	wire[15:0]	rx_checksum;

	InternetChecksum32bit rx_csum(
		.clk(clk),
		.load(1'b0),
		.reset(rx_l3_bus.start),
		.process(rx_l3_bus.data_valid),
		.din(rx_l3_bus.data),
		.sumout(),
		.csumout(rx_checksum)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX checksum engine

	wire[15:0]	tx_checksum;

	//Start checksumming when we pass the type/code/checksum word.
	//This is a dirty hack that takes advantage of the fact that ICMP_TYPE_ECHO_REPLY is zero,
	//which is the identity element for the Internet checksum.
	//TODO: when we support sending other types of ICMP message, fix this!
	InternetChecksum32bit tx_csum(
		.clk(clk),
		.load(1'b0),
		.reset(rx_l3_bus.start),
		.process(rx_l3_bus.data_valid && (rx_state != RX_STATE_HEADER_1) ),
		.din(rx_l3_bus.data),
		.sumout(),
		.csumout(tx_checksum)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main datapath/control logic

	reg[7:0]	rx_type				= 0;
	reg[7:0]	rx_code				= 0;

	reg[15:0]	rx_ping_id			= 0;
	reg[15:0]	rx_ping_seq			= 0;

	reg[7:0]	tx_type				= 0;
	reg[7:0]	tx_code				= 0;

	reg[15:0]	tx_ping_id			= 0;
	reg[15:0]	tx_ping_seq			= 0;

	reg[15:0]	tx_bytes_left		= 0;

	reg			tx_fifo_rd_ff		= 0;

	always @(posedge clk) begin

		//Clear flags
		tx_l3_bus.drop			<= 0;
		tx_l3_bus.start			<= 0;
		tx_l3_bus.commit		<= 0;
		tx_l3_bus.data_valid	<= 0;
		tx_l3_bus.bytes_valid	<= 0;
		tx_fifo_rst				<= 0;
		tx_fifo_rd				<= 0;

		tx_fifo_rd_ff			<= tx_fifo_rd;

		//RX state machine
		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for new packets to come in

			RX_STATE_IDLE: begin

				if(rx_l3_bus.start)
					rx_state			<= RX_STATE_HEADER_0;

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// HEADER - crunch packet headers

			//Wait for IP headers
			RX_STATE_HEADER_0: begin

				if(rx_l3_bus.headers_valid) begin

					//Drop anything that isn't ICMP, or too small to be a valid ICMP packet
					if( (rx_l3_bus.payload_len < 8) || !rx_l3_bus.protocol_is_icmp ) begin
						tx_l3_bus.drop	<= 1;
						rx_state		<= RX_STATE_IDLE;
					end

					//Send the start command out to layer 3 after we get the payload length
					else begin
						rx_state		<= RX_STATE_HEADER_1;
					end

				end

			end	//end RX_STATE_HEADER_0

			//ICMP type/code/checksum
			RX_STATE_HEADER_1: begin

				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4) begin
						tx_l3_bus.drop	<= 1;
						rx_state		<= RX_STATE_IDLE;
					end

					else begin
						rx_type			<= rx_l3_bus.data[31:24];
						rx_code			<= rx_l3_bus.data[23:16];

						//Type determines where we go next
						case(rx_l3_bus.data[31:24])

							ICMP_TYPE_ECHO_REQUEST: begin

								if(rx_l3_bus.data[23:16] == 0) begin
									rx_state		<= RX_STATE_PING_HEADER;

									//Reply with an echo-reply message


								end

								//Bad code
								else begin
									tx_l3_bus.drop	<= 1;
									rx_state		<= RX_STATE_IDLE;
								end

							end	//end ICMP_TYPE_ECHO_REQUEST

							//Ignore anything else
							default: begin
								tx_l3_bus.drop		<= 1;
								rx_state			<= RX_STATE_IDLE;
							end

						endcase

					end

				end

			end	//end RX_STATE_HEADER_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Echo Request message

			//ID/sequence header
			RX_STATE_PING_HEADER: begin

				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4) begin
						tx_l3_bus.drop	<= 1;
						rx_state		<= RX_STATE_IDLE;
					end

					//Parse stuff
					else begin
						rx_ping_id		<= rx_l3_bus.data[31:16];
						rx_ping_seq		<= rx_l3_bus.data[15:0];
						rx_state		<= RX_STATE_PING_BODY;
					end

				end

			end	//end RX_STATE_PING_HEADER

			//Packet body
			RX_STATE_PING_BODY: begin

				//Push the incoming data into our TX buffer
				if(rx_l3_bus.data_valid) begin
					//no action needed, the TX buffer is self-contained
				end

				if(rx_l3_bus.commit) begin

					//Verify checksum
					if(rx_checksum != 0) begin
						rx_state			<= RX_STATE_IDLE;
						tx_l3_bus.drop		<= 1;
						tx_fifo_rst			<= 1;
						perf_icmp_csumfail	<= perf_icmp_csumfail + 1'h1;
					end

					//Reply to the incoming ping! Save our headers
					//TODO: Message length should be in a separate FIFO to prevent problems
					//if we have a huge packet followed by a tiny one. This is only safe if all pings are the same size!
					//We don't need a fifo for the type/code/id/seq fields as those will be used 3 clocks from now,
					//and we can't get another packet to here that quickly.
					else begin
						tx_l3_bus.payload_len	<= rx_l3_bus.payload_len;
						tx_l3_bus.dst_ip		<= rx_l3_bus.src_ip;
						tx_type					<= ICMP_TYPE_ECHO_REPLY;
						tx_code					<= 8'h0;
						tx_l3_bus.start			<= 1;
						rx_state				<= RX_STATE_IDLE;
						tx_ping_id				<= rx_ping_id;
						tx_ping_seq				<= rx_ping_seq;
						tx_bytes_left			<= { rx_l3_bus.payload_len - 'd8 };
					end
				end

			end	//end RX_STATE_PING_BODY

		endcase

		//TX state machine
		case(tx_state)

			TX_STATE_IDLE: begin
				if(tx_l3_bus.start) begin

					//Send ICMP header #1
					tx_l3_bus.data_valid	<= 1;
					tx_l3_bus.bytes_valid	<= 4;
					tx_l3_bus.data			<= { tx_type, tx_code, tx_checksum };
					tx_state				<= TX_STATE_HEADER;

					//Pop the first word of the FIFO
					tx_fifo_rd				<= 1;

				end
			end	//end TX_STATE_IDLE

			TX_STATE_HEADER: begin

				//Send ID/sequence header
				tx_l3_bus.data_valid	<= 1;
				tx_l3_bus.bytes_valid	<= 4;
				tx_l3_bus.data			<= { tx_ping_id, tx_ping_seq };
				tx_state				<= TX_STATE_BODY;

				//Pop the second word of the FIFO (if any)
				if(tx_bytes_left > 4) begin
					tx_fifo_rd		<= 1;
					tx_bytes_left	<= tx_bytes_left - 'd4;
				end
				else
					tx_bytes_left	<= 0;

			end	//end TX_STATE_HEADER

			TX_STATE_BODY: begin

				tx_l3_bus.data_valid	<= 1;
				tx_l3_bus.data			<= tx_fifo_rdata;

				tx_l3_bus.bytes_valid	<= 4;

				//If we have more than 4 bytes left, read the next word
				if(tx_bytes_left > 4) begin
					tx_fifo_rd			<= 1;
					tx_bytes_left		<= tx_bytes_left - 4;
				end

				//Don't send 0-byte words
				if(tx_bytes_left == 0)
					tx_l3_bus.data_valid	<= 0;

				//Not reading? Last round
				if(!tx_fifo_rd) begin
					tx_bytes_left			<= 0;
					tx_l3_bus.bytes_valid	<= tx_bytes_left;
					tx_state				<= TX_STATE_COMMIT;
				end

			end	//end TX_STATE_BODY

			TX_STATE_COMMIT: begin
				tx_l3_bus.commit	<= 1;
				tx_state			<= TX_STATE_IDLE;
			end	//end TX_STATE_COMMIT

		endcase

		//If we get a drop request from the IP stack, abort everything
		//Clear the TX FIFO when this happens.
		//This may cause an in-progress packet to be lost but dropping pings isn't going to hurt much.
		//Corrupted packets are rare enough that it's not worth trying to optimize behavior in that case.
		if(rx_l3_bus.drop) begin
			rx_state		<= RX_STATE_IDLE;
			tx_l3_bus.drop	<= 1;
			tx_fifo_rst		<= 1;
			tx_state		<= TX_STATE_IDLE;
		end

	end

endmodule
