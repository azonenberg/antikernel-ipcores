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

	//Outgoing data to the arbiter
	output EthernetTxL2Bus		ipv4_tx_arp_bus,

	//Address mappings learned from the ARP stack
	input wire					learn_en,
	input wire[31:0]			learn_ip,
	input wire[47:0]			learn_mac

	//TODO: request signal to the ARP stack "hey, look up this IP"
);

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

	wire		lookup_done;
	wire		lookup_hit;
	wire[47:0]	lookup_mac;

	ARPCache #(
		.LINES_PER_WAY(LINES_PER_WAY),
		.NUM_WAYS(NUM_WAYS)
	) cache(
		.clk(clk),

		.lookup_en(ipv4_tx_l2_bus.start),
		.lookup_ip(ipv4_tx_l2_bus.dst_ip),
		.lookup_done(lookup_done),
		.lookup_hit(lookup_hit),
		.lookup_mac(lookup_mac),

		.learn_en(learn_en),
		.learn_ip(learn_ip),
		.learn_mac(learn_mac),

		.aging_tick(aging_tick),
		.max_age(MAX_AGE[15:0])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Buffer incoming traffic until we know where it goes

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address lookup and insertion

	always_ff @(posedge clk) begin
	end

	always_comb begin
		ipv4_tx_arp_bus.start		<= ipv4_tx_l2_bus.start;
		ipv4_tx_arp_bus.data_valid	<= ipv4_tx_l2_bus.data_valid;
		ipv4_tx_arp_bus.bytes_valid	<= ipv4_tx_l2_bus.bytes_valid;
		ipv4_tx_arp_bus.data		<= ipv4_tx_l2_bus.data;
		ipv4_tx_arp_bus.commit		<= ipv4_tx_l2_bus.commit;
		ipv4_tx_arp_bus.drop		<= ipv4_tx_l2_bus.drop;
		ipv4_tx_arp_bus.dst_mac		<= 48'hffffffffffff;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ARP query generation

endmodule
