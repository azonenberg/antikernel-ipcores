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

/**
	@file
	@author Andrew D. Zonenberg
	@brief	Arbiter between IPv4 layer-4 protocols
 */
module IPv4TransmitArbiter #(
	parameter FIFO_DEPTH	=	1024
)(
	input wire	clk,

	input wire IPv4TxBus		icmp_bus,
	input wire IPv4TxBus		tcp_bus,
	input wire IPv4TxBus		udp_bus,

	output IPv4TxBus			ipv4_bus,
	input wire					tx_busy
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input buffers

	//TODO: we don't have to store all of the struct fields. We can save a lot of BRAM by not buffering the IP etc
	//for every word of the message

	localparam BUS_WIDTH = $bits(IPv4TxBus);
	localparam SIZE_BITS = $clog2(FIFO_DEPTH);

	logic				icmp_fifo_rd	= 0;
	logic				icmp_fifo_wr	= 0;
	IPv4TxBus			icmp_fifo_rdata;

	wire[SIZE_BITS:0]	icmp_fifo_wsize;
	wire[SIZE_BITS:0]	icmp_fifo_rsize;

	SingleClockFifo #(
		.WIDTH(BUS_WIDTH),
		.DEPTH(FIFO_DEPTH)
	) icmp_fifo (
		.clk(clk),
		.wr(icmp_fifo_wr),
		.din(icmp_bus),

		.rd(icmp_fifo_rd),
		.dout(icmp_fifo_rdata),
		.overflow(),
		.underflow(),
		.empty(),
		.full(),
		.rsize(icmp_fifo_rsize),
		.wsize(icmp_fifo_wsize),
		.reset(1'b0)
	);

	logic				tcp_fifo_rd	= 0;
	logic				tcp_fifo_wr	= 0;
	IPv4TxBus			tcp_fifo_rdata;

	wire[SIZE_BITS:0]	tcp_fifo_wsize;
	wire[SIZE_BITS:0]	tcp_fifo_rsize;

	SingleClockFifo #(
		.WIDTH(BUS_WIDTH),
		.DEPTH(FIFO_DEPTH)
	) tcp_fifo (
		.clk(clk),
		.wr(tcp_fifo_wr),
		.din(tcp_bus),

		.rd(tcp_fifo_rd),
		.dout(tcp_fifo_rdata),
		.overflow(),
		.underflow(),
		.empty(),
		.full(),
		.rsize(tcp_fifo_rsize),
		.wsize(tcp_fifo_wsize),
		.reset(1'b0)
	);

	logic				udp_fifo_rd	= 0;
	logic				udp_fifo_wr = 0;
	IPv4TxBus			udp_fifo_rdata;

	wire[SIZE_BITS:0]	udp_fifo_wsize;
	wire[SIZE_BITS:0]	udp_fifo_rsize;

	SingleClockFifo #(
		.WIDTH(BUS_WIDTH),
		.DEPTH(FIFO_DEPTH)
	) udp_fifo (
		.clk(clk),
		.wr(udp_fifo_wr),
		.din(udp_bus),

		.rd(udp_fifo_rd),
		.dout(udp_fifo_rdata),
		.overflow(),
		.underflow(),
		.empty(),
		.full(),
		.rsize(udp_fifo_rsize),
		.wsize(udp_fifo_wsize),
		.reset(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input FIFO control

	logic	udp_wr_active	= 0;
	logic	tcp_wr_active	= 0;
	logic	icmp_wr_active	= 0;

	//Only write if we have space (192 words should be enough)
	wire	udp_has_space = (udp_fifo_wsize >= 192);
	wire	tcp_has_space = (tcp_fifo_wsize >= 192);
	wire	icmp_has_space = (icmp_fifo_wsize >= 192);

	//Forward if active and a flag is set, or starting
	always_comb begin

		//Default to not writing
		udp_fifo_wr		<= 0;
		tcp_fifo_wr		<= 0;
		icmp_fifo_wr	<= 0;

		//Start writing if we have a new packet inbound (if there's space)
		//TODO: performance counter for packets dropped due to lack of space
		if(udp_bus.start && udp_has_space)
			udp_fifo_wr	<= 1;
		if(tcp_bus.start && tcp_has_space)
			tcp_fifo_wr	<= 1;
		if(icmp_bus.start && icmp_has_space)
			icmp_fifo_wr	<= 1;

		//Keep writing if a flag is set
		if(udp_wr_active)
			udp_fifo_wr		<= (udp_bus.drop || udp_bus.commit || udp_bus.data_valid);
		if(tcp_wr_active)
			tcp_fifo_wr		<= (tcp_bus.drop || tcp_bus.commit || tcp_bus.data_valid);
		if(icmp_wr_active)
			icmp_fifo_wr	<= (icmp_bus.drop || icmp_bus.commit || icmp_bus.data_valid);

	end

	always_ff @(posedge clk) begin

		//Become active on a start packet
		if(udp_bus.start)
			udp_wr_active	<= udp_has_space;
		if(tcp_bus.start)
			tcp_wr_active	<= tcp_has_space;
		if(icmp_bus.start)
			icmp_wr_active	<= icmp_has_space;

		//Stop being active on a commit/drop packet
		if(udp_bus.commit || udp_bus.drop)
			udp_wr_active	<= 0;
		if(tcp_bus.commit || tcp_bus.drop)
			tcp_wr_active	<= 0;
		if(icmp_bus.commit || icmp_bus.drop)
			icmp_wr_active	<= 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output muxing

	enum logic[3:0]
	{
		STATE_IDLE		= 4'h0,
		STATE_ICMP_0	= 4'h1,
		STATE_ICMP_1	= 4'h2,
		STATE_UDP_0		= 4'h3,
		STATE_UDP_1		= 4'h4,
		STATE_TCP_0		= 4'h5,
		STATE_TCP_1		= 4'h6
	} state = STATE_IDLE;


	always_ff @(posedge clk) begin

		//Clear flags, but leave data after we finish sending a packet
		ipv4_bus.start	<= 0;
		ipv4_bus.commit	<= 0;

		tcp_fifo_rd		<= 0;
		udp_fifo_rd		<= 0;
		icmp_fifo_rd	<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for data to be ready

			STATE_IDLE: begin

				//IP stack has a bit of latency for header checksum calculation.
				//Don't start the next packet until it's done with the previous one.
				if(tx_busy) begin
				end

				//Pop the most-full fifo
				else if( (tcp_fifo_rsize >= udp_fifo_rsize) && (tcp_fifo_rsize >= icmp_fifo_rsize) && (tcp_fifo_rsize > 1) ) begin
					tcp_fifo_rd		<= 1;
					state			<= STATE_TCP_0;
				end
				else if( (udp_fifo_rsize >= tcp_fifo_rsize) && (udp_fifo_rsize >= icmp_fifo_rsize) && (udp_fifo_rsize > 1) ) begin
					udp_fifo_rd		<= 1;
					state			<= STATE_UDP_0;
				end
				else if(icmp_fifo_rsize > 1) begin
					icmp_fifo_rd	<= 1;
					state			<= STATE_ICMP_0;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// ICMP forwarding path

			//Wait for read latency
			STATE_ICMP_0: begin
				icmp_fifo_rd		<= 1;
				state				<= STATE_ICMP_1;
			end	//end STATE_ICMP_0

			//Data words are ready. Keep sending.
			STATE_ICMP_1: begin

				ipv4_bus			<= icmp_fifo_rdata;

				//If we hit a stop command, stop reading.
				if(icmp_fifo_rdata.drop || icmp_fifo_rdata.commit) begin

					//Already reading another word? We have to keep going to avoid losing it
					if(icmp_fifo_rd)
						state		<= STATE_ICMP_0;

					//Nope, fifo must be almost empty.
					//Go back to idle.
					else
						state		<= STATE_IDLE;

				end

				//If there's more data in the FIFO, keep going
				else if(icmp_fifo_rsize > 1)
					icmp_fifo_rd	<= 1;

			end	//end STATE_ICMP_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// UDP forwarding path

			//Wait for read latency
			STATE_UDP_0: begin
				udp_fifo_rd			<= 1;
				state				<= STATE_UDP_1;
			end	//end STATE_UDP_0

			//Data words are ready. Keep sending.
			STATE_UDP_1: begin

				ipv4_bus			<= udp_fifo_rdata;

				//If we hit a stop command, stop reading.
				if(udp_fifo_rdata.drop || udp_fifo_rdata.commit) begin

					//Already reading another word? We have to keep going to avoid losing it
					if(udp_fifo_rd)
						state		<= STATE_UDP_0;

					//Nope, fifo must be almost empty.
					//Go back to idle.
					else
						state		<= STATE_IDLE;

				end

				//If there's more data in the FIFO, keep going
				else if(udp_fifo_rsize > 1)
					udp_fifo_rd	<= 1;

			end	//end STATE_UDP_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// TCP forwarding path

			//Wait for read latency
			STATE_TCP_0: begin
				tcp_fifo_rd			<= 1;
				state				<= STATE_TCP_1;
			end	//end STATE_TCP_0

			//Data words are ready. Keep sending.
			STATE_TCP_1: begin

				ipv4_bus			<= tcp_fifo_rdata;

				//If we hit a stop command, stop reading.
				if(tcp_fifo_rdata.drop || tcp_fifo_rdata.commit) begin

					//Already reading another word? We have to keep going to avoid losing it
					if(tcp_fifo_rd)
						state		<= STATE_TCP_0;

					//Nope, fifo must be almost empty.
					//Go back to idle.
					else
						state		<= STATE_IDLE;

				end

				//If there's more data in the FIFO, keep going
				else if(tcp_fifo_rsize > 1)
					tcp_fifo_rd	<= 1;

			end	//end STATE_TCP_1

		endcase

	end

endmodule
