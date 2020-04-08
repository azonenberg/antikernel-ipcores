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

/**
	@file
	@author Andrew D. Zonenberg
	@brief	Controller and snooping/insertion logic for the ARP cache
 */
module ARPManager #(
	parameter		AGE_INTERVAL	= 125000000,		//clocks per aging tick (default is 1 Hz @ 125 MHz)
	parameter		LINES_PER_WAY	= 128,
	parameter		NUM_WAYS		= 4,
	parameter		MAX_AGE			= 3600				//age out old table entries after an hour
)(
	input wire					clk,

	//Incoming data from the IPv4 stack
	input wire EthernetTxArpBus	ipv4_tx_l2_bus,

	//Network configuration
	input wire					link_up,
	input wire					config_update,
	input wire IPv4Config		ip_config,

	//Outgoing data to the arbiter
	output EthernetTxL2Bus		ipv4_tx_arp_bus,

	//Address mappings learned from the ARP stack
	input wire					learn_en,
	input wire[31:0]			learn_ip,
	input wire[47:0]			learn_mac,

	//Query requests to the ARP stack
	output logic				query_en	= 0,
	output logic[31:0]			query_ip	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off constant ports

	`include "Ethertypes.svh"

	always_comb
		ipv4_tx_arp_bus.ethertype	<= ETHERTYPE_ARP;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Aging counter

	reg[31:0]	aging_count = 0;
	reg			aging_tick	= 0;

	always_ff @(posedge clk) begin

		aging_count	<= aging_count + 1'h1;
		aging_tick	<= 0;

		if(aging_count == AGE_INTERVAL) begin
			aging_count	<= 0;
			aging_tick	<= 1;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The ARP cache

	//TODO: When a cache entry gets old, send an ARP query for the target IP pre-emptively
	//rather than waiting for it to expire.

	logic		lookup_en	= 0;
	logic[31:0]	lookup_ip	= 0;

	wire		lookup_done;
	wire		lookup_hit;
	wire[47:0]	lookup_mac;

	ARPCache #(
		.LINES_PER_WAY(LINES_PER_WAY),
		.NUM_WAYS(NUM_WAYS)
	) cache(
		.clk(clk),

		.lookup_en(lookup_en),
		.lookup_ip(lookup_ip),
		.lookup_done(lookup_done),
		.lookup_hit(lookup_hit),
		.lookup_mac(lookup_mac),

		.learn_en(learn_en),
		.learn_ip(learn_ip),
		.learn_mac(learn_mac),

		.aging_tick(aging_tick),
		.max_age(MAX_AGE[14:0])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Buffer incoming traffic until we know where it goes

	localparam L2_BUS_SIZE = $bits(EthernetTxArpBus);

	logic	tx_fifo_wr	= 0;
	logic	tx_fifo_rd	= 0;
	wire	tx_fifo_empty;

	wire[5:0]	tx_fifo_rsize;

	EthernetTxArpBus tx_fifo_rdata;

	SingleClockFifo #(
		.WIDTH(L2_BUS_SIZE),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) tx_fifo (
		.clk(clk),
		.wr(tx_fifo_wr),
		.din(ipv4_tx_l2_bus),

		.rd(tx_fifo_rd),
		.dout(tx_fifo_rdata),
		.overflow(),
		.underflow(),
		.empty(tx_fifo_empty),
		.full(),
		.rsize(tx_fifo_rsize),
		.wsize(),
		.reset(1'b0)
	);

	always_comb begin
		tx_fifo_wr	<= 0;

		//Only write if something is happening
		if(ipv4_tx_l2_bus.start)
			tx_fifo_wr	<= 1;
		if(ipv4_tx_l2_bus.data_valid)
			tx_fifo_wr	<= 1;
		if(ipv4_tx_l2_bus.commit || ipv4_tx_l2_bus.drop)
			tx_fifo_wr	<= 1;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address lookup and insertion

	enum logic[3:0]
	{
		TX_STATE_IDLE		= 4'h0,
		TX_STATE_LOOKUP_0	= 4'h1,
		TX_STATE_LOOKUP_1	= 4'h2,
		TX_STATE_BODY		= 4'h3
	} tx_state = TX_STATE_IDLE;

	logic	tx_fifo_rd_ff			= 0;

	always_ff @(posedge clk) begin

		tx_fifo_rd				<= 0;
		lookup_en				<= 0;
		tx_fifo_rd_ff			<= tx_fifo_rd;

		ipv4_tx_arp_bus.start		<= 0;
		ipv4_tx_arp_bus.drop		<= 0;
		ipv4_tx_arp_bus.commit		<= 0;
		ipv4_tx_arp_bus.data_valid	<= 0;

		case(tx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for somebody to send a packet, or the gateway to need a lookup

			TX_STATE_IDLE: begin

				if(!tx_fifo_empty) begin
					tx_fifo_rd	<= 1;
					tx_state	<= TX_STATE_LOOKUP_0;
				end

			end	//end TX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for FIFO read and do table lookup

			TX_STATE_LOOKUP_0: begin
				if(!tx_fifo_rd) begin

					//If this isn't a start command, drop the packet (should never happen)
					if(!tx_fifo_rdata.start)
						tx_state	<= TX_STATE_IDLE;

					//It's a start command. Look up the MAC address.
					else begin
						lookup_en	<= 1;
						lookup_ip	<= tx_fifo_rdata.dst_ip;
						tx_state	<= TX_STATE_LOOKUP_1;
					end

				end
			end	//end TX_STATE_LOOKUP_0

			TX_STATE_LOOKUP_1: begin

				//Lookup has completed.
				//For now, don't even check the result. Use whatever came back as the destination (miss = broadcast)
				if(lookup_done) begin
					ipv4_tx_arp_bus.start	<= 1;
					ipv4_tx_arp_bus.dst_mac	<= lookup_mac;

					tx_fifo_rd				<= 1;
					tx_state				<= TX_STATE_BODY;
				end

			end	//end TX_STATE_LOOKUP_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Packet data

			TX_STATE_BODY: begin

				//If there's at least one word of data in the FIFO
				//(after the one we're popping now) submit another read requst
				if(tx_fifo_rsize > 1)
					tx_fifo_rd	<= 1;

				//If there is read data, send it.
				if(tx_fifo_rd_ff) begin

					//Commit or drop request? We're done.
					if(tx_fifo_rdata.commit || tx_fifo_rdata.drop) begin
						ipv4_tx_arp_bus.commit	<= tx_fifo_rdata.commit;
						ipv4_tx_arp_bus.drop	<= tx_fifo_rdata.drop;

						//Cancel any outstanding read.
						tx_fifo_rd				<= 0;

						//If there's a read in progress this cycle, jump right into the next lookup
						if(tx_fifo_rd)
							tx_state			<= TX_STATE_LOOKUP_0;

						//Otherwise we're done
						else
							tx_state			<= TX_STATE_IDLE;
					end

					//Nope, normal packet data
					else begin
						ipv4_tx_arp_bus.data_valid	<= tx_fifo_rdata.data_valid;
						ipv4_tx_arp_bus.data		<= tx_fifo_rdata.data;
						ipv4_tx_arp_bus.bytes_valid	<= tx_fifo_rdata.bytes_valid;
					end

				end

			end	//end TX_STATE_BODY

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ARP query generation

	logic	gateway_lookup_pending		= 0;
	logic	link_up_ff					= 0;

	logic[23:0]	gateway_lookup_timeout	= 0;	//24 bit timer @ 100 MHz = 167 ms

	always_ff @(posedge clk) begin
		query_en	<= 0;

		link_up_ff	<= link_up;

		if(gateway_lookup_timeout != 0)
			gateway_lookup_timeout	<= gateway_lookup_timeout - 1;

		//If we change IP config, or the link flaps
		//send a query for our default gateway after a short timeout period
		//TODO: if none comes back send more?
		if(link_up && (!link_up_ff || config_update) ) begin
			gateway_lookup_pending	<= 1;
			gateway_lookup_timeout	<= 24'hffffff;
		end

		//Generate a query in response to an ARP cache miss
		if(lookup_done && !lookup_hit) begin
			query_en	<= 1;
			query_ip	<= lookup_ip;
		end

		//Look up our gateway
		else if(gateway_lookup_pending && (gateway_lookup_timeout == 0) ) begin
			gateway_lookup_pending	<= 0;
			query_en				<= 1;
			query_ip				<= ip_config.gateway;
		end

	end

endmodule
