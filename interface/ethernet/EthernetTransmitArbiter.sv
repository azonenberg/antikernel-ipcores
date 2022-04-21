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

/**
	@brief Mux data from multiple layer-3 protocols to one MAC

	TODO: refactor this when we have more ethertypes to use generate loops or something
 */
module EthernetTransmitArbiter #(
	parameter PACKET_DEPTH		= 8192,		//Packet-data FIFO is 32 bits wide x this many words
											//Default 8192 = 32768 bytes
											//(21 standard frames, 3 jumbo frames, 512 min-sized frames)

	parameter ARP_PACKET_DEPTH		= 512,	//Packet-data FIFO is 32 bits wide x this many words
											//Default 512 = 2048 bytes
											//(1 max sized frame, 44 ARP replies)

	parameter HEADER_DEPTH			= 512,	//Depth of header FIFO, in packets
	parameter ARP_HEADER_DEPTH		= 64,
	parameter JUMBO_FRAME_SUPPORT	= 1
)(

	//Clocks
	input wire					clk,

	//Inbound data from the upper level protocols
	input wire EthernetTxL2Bus	ipv4_tx_l2_bus,
	input wire EthernetTxL2Bus	arp_tx_l2_bus,

	//Outbound data to the MAC
	output EthernetTxL2Bus		tx_l2_bus				= { $bits(EthernetTxL2Bus){1'b0} },

	output EthernetArbiterPerformanceCounters	perf	= {64'h0, 64'h0, 64'h0, 64'h0}
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration

	localparam PACKET_BITS		= $clog2(PACKET_DEPTH);
	localparam ARP_PACKET_BITS	= $clog2(ARP_PACKET_DEPTH);
	localparam HEADER_BITS		= $clog2(HEADER_DEPTH);
	localparam ARP_HEADER_BITS	= $clog2(ARP_HEADER_DEPTH);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input data FIFOs

	//These don't strictly need to be cross-clock and we can lower latency by not making them cross clock
	//but we don't have a good single clock packet FIFO :(

	//TODO: Can we merge this with EthernetTransmitElasticBuffer?

	wire[PACKET_BITS:0]		ipv4_payload_fifo_free;
	reg						ipv4_packet_active	= 0;

	reg						ipv4_rd_en			= 0;
	reg[PACKET_BITS-1:0]	ipv4_rd_offset		= 0;
	reg						ipv4_pop_packet		= 0;
	reg[PACKET_BITS:0]		ipv4_pop_size		= 0;
	wire[31:0]				ipv4_rd_data;

	wire[PACKET_BITS:0]		ipv4_rd_avail;

	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(PACKET_DEPTH)
	) ipv4_payload_fifo (
		.wr_clk(clk),
		.wr_en(ipv4_tx_l2_bus.data_valid && ipv4_packet_active),
		.wr_data(ipv4_tx_l2_bus.data),
		.wr_reset(1'b0),
		.wr_size(ipv4_payload_fifo_free),
		.wr_commit(ipv4_tx_l2_bus.commit),
		.wr_rollback(ipv4_tx_l2_bus.drop),

		.rd_clk(clk),
		.rd_en(ipv4_rd_en),
		.rd_offset(ipv4_rd_offset),
		.rd_pop_single(1'b0),
		.rd_pop_packet(ipv4_pop_packet),
		.rd_packet_size(ipv4_pop_size),
		.rd_data(ipv4_rd_data),
		.rd_size(ipv4_rd_avail),
		.rd_reset(1'b0)
	);

	wire[ARP_PACKET_BITS:0]		arp_payload_fifo_free;
	reg							arp_packet_active	= 0;

	reg							arp_rd_en			= 0;
	reg[ARP_PACKET_BITS-1:0]	arp_rd_offset		= 0;
	reg							arp_pop_packet		= 0;
	reg[ARP_PACKET_BITS:0]		arp_pop_size		= 0;
	wire[31:0]					arp_rd_data;

	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(ARP_PACKET_DEPTH)
	) arp_payload_fifo (
		.wr_clk(clk),
		.wr_en(arp_tx_l2_bus.data_valid && arp_packet_active),
		.wr_data(arp_tx_l2_bus.data),
		.wr_reset(1'b0),
		.wr_size(arp_payload_fifo_free),
		.wr_commit(arp_tx_l2_bus.commit),
		.wr_rollback(arp_tx_l2_bus.drop),

		.rd_clk(clk),
		.rd_en(arp_rd_en),
		.rd_offset(arp_rd_offset),
		.rd_pop_single(1'b0),
		.rd_pop_packet(arp_pop_packet),
		.rd_packet_size(arp_pop_size),
		.rd_data(arp_rd_data),
		.rd_size(),
		.rd_reset(1'b0)
	);

	reg[15:0]	ipv4_tx_frame_size	= 0;
	reg[15:0]	arp_tx_frame_size	= 0;

	localparam MIN_FREE = JUMBO_FRAME_SUPPORT ? 2250 : 375;

	always_ff @(posedge clk) begin

		//Keep track of frame state and only allow new frames if we have space
		//For now, require 9000 bytes / 2250 words (jumbo frame typical MTU)
		//TODO: allow arbitrary frame size and roll back if we overflow
		if(ipv4_tx_l2_bus.start) begin
			if(ipv4_payload_fifo_free > MIN_FREE) begin
				ipv4_packet_active	<= 1;
				ipv4_tx_frame_size	<= 0;
			end
			else begin
				ipv4_packet_active	<= 0;
				perf.ipv4_dropped	<= perf.ipv4_dropped + 1'h1;
			end
		end
		if(arp_tx_l2_bus.start) begin
			if(arp_payload_fifo_free > 128) begin	//arp packets are much smaller
				arp_packet_active	<= 1;
				arp_tx_frame_size	<= 0;
			end
			else
				perf.arp_dropped	<= perf.arp_dropped + 1'h1;
		end

		if(arp_tx_l2_bus.commit)
			perf.arp_sent		<= perf.arp_sent + 1'h1;
		if(ipv4_tx_l2_bus.commit && ipv4_packet_active)
			perf.ipv4_sent		<= perf.ipv4_sent + 1'h1;

		//Add new frame data as we go
		if(ipv4_tx_l2_bus.data_valid)
			ipv4_tx_frame_size	<= ipv4_tx_frame_size + ipv4_tx_l2_bus.bytes_valid;
		if(arp_tx_l2_bus.data_valid)
			arp_tx_frame_size	<= arp_tx_frame_size + arp_tx_l2_bus.bytes_valid;

		//End frames
		if(ipv4_tx_l2_bus.commit || ipv4_tx_l2_bus.drop)
			ipv4_packet_active	<= 0;
		if(arp_tx_l2_bus.commit || arp_tx_l2_bus.drop)
			arp_packet_active	<= 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input header FIFOs

	//HEADER INFO (ethertype is implicit)
	//63:48	Frame length
	//47:0	Dest MAC

	wire[HEADER_BITS:0]	ipv4_header_fifo_free;
	reg					ipv4_tx_l2_bus_commit_ff	= 0;

	wire[HEADER_BITS:0]	ipv4_header_fifo_avail;

	reg					ipv4_header_rd_en	= 0;

	wire[63:0]			ipv4_header;
	wire[15:0]			ipv4_packet_len		= ipv4_header[63:48];
	wire[47:0]			ipv4_packet_mac		= ipv4_header[47:0];

	CrossClockPacketFifo #(
		.WIDTH(64),
		.DEPTH(HEADER_DEPTH)
	) ipv4_header_fifo (
		.wr_clk(clk),
		.wr_en(ipv4_tx_l2_bus.commit && ipv4_packet_active),
		.wr_data({ipv4_tx_frame_size, ipv4_tx_l2_bus.dst_mac }),
		.wr_reset(1'b0),
		.wr_size(ipv4_header_fifo_free),
		.wr_commit(ipv4_tx_l2_bus_commit_ff),
		.wr_rollback(1'b0),

		.rd_clk(clk),
		.rd_en(ipv4_header_rd_en),
		.rd_offset({HEADER_BITS{1'b0}}),
		.rd_pop_single(ipv4_pop_packet),	//one packet is one word
		.rd_pop_packet(1'b0),
		.rd_packet_size({HEADER_BITS{1'b0}}),
		.rd_data(ipv4_header),
		.rd_size(ipv4_header_fifo_avail),
		.rd_reset(1'b0)
	);

	wire[ARP_HEADER_BITS:0]	arp_header_fifo_free;
	reg						arp_tx_l2_bus_commit_ff	= 0;

	wire[ARP_HEADER_BITS:0]	arp_header_fifo_avail;

	reg						arp_header_rd_en	= 0;

	wire[63:0]				arp_header;
	wire[15:0]				arp_packet_len		= arp_header[63:48];
	wire[47:0]				arp_packet_mac		= arp_header[47:0];

	CrossClockPacketFifo #(
		.WIDTH(64),
		.DEPTH(ARP_HEADER_DEPTH)
	) arp_header_fifo (
		.wr_clk(clk),
		.wr_en(arp_tx_l2_bus.commit && arp_packet_active),
		.wr_data({arp_tx_frame_size, arp_tx_l2_bus.dst_mac }),
		.wr_reset(1'b0),
		.wr_size(arp_header_fifo_free),
		.wr_commit(arp_tx_l2_bus_commit_ff),
		.wr_rollback(1'b0),

		.rd_clk(clk),
		.rd_en(arp_header_rd_en),
		.rd_offset({ARP_HEADER_BITS{1'b0}}),
		.rd_pop_single(arp_pop_packet),
		.rd_pop_packet(1'b0),
		.rd_packet_size({ARP_HEADER_BITS{1'b0}}),
		.rd_data(arp_header),
		.rd_size(arp_header_fifo_avail),
		.rd_reset(1'b0)
	);

	always @(posedge clk) begin
		ipv4_tx_l2_bus_commit_ff	<= ipv4_tx_l2_bus.commit;
		arp_tx_l2_bus_commit_ff		<= arp_tx_l2_bus.commit;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Muxing and output logic

	`include "Ethertypes.svh"

	enum logic[3:0]
	{
		STATE_IDLE				= 4'h0,
		STATE_ARP_HEADER_READ	= 4'h1,
		STATE_ARP_BODY			= 4'h2,
		STATE_ARP_COMMIT		= 4'h3,
		STATE_IPV4_HEADER_READ	= 4'h4,
		STATE_IPV4_BODY			= 4'h5,
		STATE_IPV4_COMMIT		= 4'h6
	} state = STATE_IDLE;

	logic[15:0]	tx_bytes_left	= 0;

	always_ff @(posedge clk) begin

		ipv4_rd_en			<= 0;
		ipv4_rd_offset		<= 0;
		ipv4_pop_packet		<= 0;
		ipv4_pop_size		<= 0;
		ipv4_header_rd_en	<= 0;

		arp_rd_en			<= 0;
		arp_rd_offset		<= 0;
		arp_pop_packet		<= 0;
		arp_pop_size		<= 0;
		arp_header_rd_en	<= 0;

		tx_l2_bus.start			<= 0;
		tx_l2_bus.data_valid	<= 0;
		tx_l2_bus.bytes_valid	<= 0;
		tx_l2_bus.data			<= 0;
		tx_l2_bus.commit		<= 0;
		tx_l2_bus.drop			<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for stuff to happen

			STATE_IDLE: begin

				//ARP replies get highest priority.
				//We shouldn't have too much ARP traffic compared to other stuff
				if(arp_header_fifo_avail) begin
					arp_header_rd_en	<= 1;
					state				<= STATE_ARP_HEADER_READ;
				end

				//IPv4 is lowest priority
				else if(ipv4_header_fifo_avail) begin
					ipv4_header_rd_en	<= 1;
					state				<= STATE_IPV4_HEADER_READ;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// ARP transmit datapath

			STATE_ARP_HEADER_READ: begin

				//Waiting for header FIFO to read.
				//While we're doing that, pre-fetch the first message data word
				if(arp_header_rd_en) begin
					arp_rd_en		<= 1;
					arp_rd_offset	<= 0;
				end

				//Headers are ready!
				//Update our headers, then read the second message data word
				else begin
					tx_l2_bus.start		<= 1;
					tx_l2_bus.ethertype	<= ETHERTYPE_ARP;
					tx_l2_bus.dst_mac	<= arp_packet_mac;
					tx_bytes_left		<= arp_packet_len;

					arp_rd_en			<= 1;
					arp_rd_offset		<= 1;

					state				<= STATE_ARP_BODY;
				end

			end	//end STATE_ARP_HEADER_READ

			STATE_ARP_BODY: begin

				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= arp_rd_data;

				arp_rd_offset			<= arp_rd_offset + 1'h1;
				tx_bytes_left			<= tx_bytes_left - 16'd4;

				if(tx_bytes_left > 8)
					arp_rd_en			<= 1;

				if(tx_bytes_left > 4)
					tx_l2_bus.bytes_valid	<= 4;

				else begin
					tx_l2_bus.bytes_valid	<= tx_bytes_left;
					tx_bytes_left			<= 0;

					//Pop the packet now. rather than in STATE_ARP_COMMIT
					//so that it'll be done by the time we get to STATE_IDLE again
					//Note, pop size is WORDS, not bytes!!!
					arp_pop_packet		<= 1;
					if(arp_packet_len[1:0])
						arp_pop_size	<= arp_packet_len[15:2] + 16'h1;
					else
						arp_pop_size	<= arp_packet_len[15:2];

					state				<= STATE_ARP_COMMIT;
				end

			end	//end STATE_ARP_BODY

			STATE_ARP_COMMIT: begin
				tx_l2_bus.commit		<= 1;

				state					<= STATE_IDLE;
			end	//end STATE_ARP_COMMIT

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IPv4 transmit datapath

			STATE_IPV4_HEADER_READ: begin

				//Waiting for header FIFO to read.
				//While we're doing that, pre-fetch the first message data word
				if(ipv4_header_rd_en) begin
					ipv4_rd_en		<= 1;
					ipv4_rd_offset	<= 0;
				end

				//Headers are ready!
				//Update our headers, then read the second message data word
				else begin
					tx_l2_bus.start		<= 1;
					tx_l2_bus.ethertype	<= ETHERTYPE_IPV4;
					tx_l2_bus.dst_mac	<= ipv4_packet_mac;
					tx_bytes_left		<= ipv4_packet_len;

					ipv4_rd_en			<= 1;
					ipv4_rd_offset		<= 1;

					state				<= STATE_IPV4_BODY;
				end

			end	//end STATE_IPV4_HEADER_READ

			STATE_IPV4_BODY: begin

				tx_l2_bus.data_valid	<= 1;
				tx_l2_bus.data			<= ipv4_rd_data;

				ipv4_rd_offset			<= ipv4_rd_offset + 1'h1;
				tx_bytes_left			<= tx_bytes_left - 16'd4;

				if(tx_bytes_left > 8)
					ipv4_rd_en				<= 1;

				if(tx_bytes_left > 4)
					tx_l2_bus.bytes_valid	<= 4;

				else begin
					tx_l2_bus.bytes_valid	<= tx_bytes_left;
					tx_bytes_left			<= 0;

					//Pop the packet now. rather than in STATE_IPV4_COMMIT
					//so that it'll be done by the time we get to STATE_IDLE again
					//Note, pop size is WORDS, not bytes!!!
					ipv4_pop_packet		<= 1;
					if(ipv4_packet_len[1:0])
						ipv4_pop_size	<= ipv4_packet_len[15:2] + 16'h1;
					else
						ipv4_pop_size	<= ipv4_packet_len[15:2];

					state				<= STATE_IPV4_COMMIT;
				end

			end	//end STATE_IPV4_BODY

			STATE_IPV4_COMMIT: begin
				tx_l2_bus.commit		<= 1;

				state					<= STATE_IDLE;
			end	//end STATE_IPV4_COMMIT


		endcase

	end

endmodule
