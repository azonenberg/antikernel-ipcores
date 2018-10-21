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
	@brief IPv4 ARP cache

	Parameterizable set associative cache.

	====================================================================================================================
	TABLE STRUCTURE

	Each cache entry needs 48 bits of MAC address and 32 of IP address plus a valid bit. This is 81 bits.
	Padding with a 15-bit aging counter gives us 96 bits or 3 block RAMs wide.

	Table depth is LINES_PER_WAY*NUM_LINES rows total.

	====================================================================================================================
	LOOKUPS

	The cache walks each cache set sequentially to reduce the number of block RAMs needed for smaller caches. This means
	that a lookup will take O(NUM_WAYS) cycles. The parent module must be prepared for this delay with a FIFO.

	To perform a lookup, assert lookup_en for one cycle with lookup_ip set to the destination IPv4 address. Wait for
	lookup_done to go high. If lookup_hit is set, lookup_mac contains the target MAC address. If not set, lookup_mac
	will be ff:ff:ff:ff:ff:ff.

	====================================================================================================================
	LEARNING

	To learn a new entry, assert learn_en for one cycle with learn_ip and learn_mac set appropriately. Learning is lower
	priority than lookup and may take an indefinite time period. There is no acknowledgement when a new address
	has been learned.

	Every inbound packet should result in the address in question being re-learned to keep its cache entry fresh.

	====================================================================================================================
	CACHE LINE AGING

	aging_tick needs to be asserted (for one clock) periodically so old cache entries can be flushed. Ticks can be
	spaced as far apart as desired for a given cache lifetime, however there must be a minimum of LINES_PER_WAY *
	NUM_WAYS idle cycles (with no lookups or learning in progress) between ticks to ensure that the table can be fully
	walked.

	This is unlikely to be an issue outside of accelerated-timescale simulation since a typical real-world aging timer
	would run at 1 Hz or less.
 */
module ARPCache #(
	parameter			LINES_PER_WAY	= 128,
	parameter			NUM_WAYS		= 4,
)(
	input wire			clk,

	input wire			lookup_en,
	input wire[31:0]	lookup_ip,

	output logic		lookup_done	= 0,
	output logic		lookup_hit	= 0,
	output logic[47:0]	lookup_mac	= 0,

	input wire			learn_en,
	input wire[31:0]	learn_ip,
	input wire[47:0]	learn_mac,

	input wire[14:0]	max_age					//entries older than this are deleted during the next aging pass
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual ARP table

	localparam			CACHE_LINES	= NUM_WAYS * LINES_PER_WAY;
	localparam			ADDR_BITS	= $clog2(CACHE_LINES);
	localparam			LINE_BITS	= $clog2(LINES_PER_WAY);
	localparam			WAY_BITS	= $clog2(NUM_WAYS;

	typedef struct packed
	{
		logic			valid;
		logic[14:0]		age;		//time since last time this address was learned
		logic[31:0]		ip;
		logic[47:0]		mac;
	} arp_entry;

	logic					rd_en	= 0;
	logic[ADDR_BITS-1:0]	rd_addr	= 0;
	arp_entry				rd_data_raw;
	arp_entry				rd_data;

	logic					wr_en	= 0;
	logic[ADDR_BITS-1:0]	wr_addr	= 0;
	arp_entry				wr_data;

	arp_entry	arp_tbl[CACHE_LINES-1:0];

	//clear table to empty at boot
	integer i;
	initial begin
		for(i=0; i<CACHE_LINES; i=i+1) begin
			arp_tbl[i].valid	<= 0;
			arp_tbl[i].age		<= 0;
			arp_tbl[i].ip		<= 0;
			arp_tbl[i].mac		<= 0;
		end
	end

	always_ff @(posedge clk) begin

		//Pipelined read
		if(rd_en)
			rd_data	<= arp_tbl[rd_addr];
		rd_data		<= rd_data_raw;

		//Write
		if(wr_en)
			arp_tbl[wr_addr]	<= wr_data;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address hashing

	logic[31:0]			lookup_hash_raw;
	wire[LINE_BITS-1:0]	lookup_hash	= lookup_hash_raw[LINE_BITS-1:0];

	always_comb begin
		lookup_hash_raw	<= lookup_ip[7:0] + lookup_ip[15:8] + lookup_ip[23:16] + lookup_ip[31:24] ^ lookup_ip[15:0];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cache state machine

	enum logic[3:0]
	{
		STATE_IDLE		= 4'h0,
		STATE_LOOKUP_0	= 4'h1,
	} state	= STATE_IDLE;

	logic				learn_pending		= 0;
	logic[31:0]			learn_pending_ip	= 0;
	logic[47:0]			learn_pending_mac	= 0;

	logic[WAY_BITS-1:0]	lookup_way			= 0;

	always_ff @(posedge clk) begin

		lookup_done	<= 0;
		lookup_hit	<= 0;
		rd_en		<= 0;
		wr_en		<= 0;

		//If a new address-learn request comes in, save it.
		//If another address is already being learned, drop the request (TODO performance counters)
		if(learn_en && !learn_pending) begin
			learn_pending		<= 1;
			learn_pending_ip	<= learn_ip;
			learn_pending_mac	<= learn_mac;
		end

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Idle

			//Sit around and wait
			STATE_IDLE: begin

				//Lookups are top priority. Do them first.
				if(lookup_en) begin
					lookup_way	<= 1;

					rd_en		<= 1;
					rd_addr		<= { lookup_hash, {WAY_BITS{1'b0}} };
					state		<= STATE_LOOKUP_0;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Cache searching

			//First phase of a lookup. Read just kicked off, no data for us yet.
			//Start reading the next set.
			STATE_LOOKUP_0: begin
				rd_en			<= 1;
				lookup_way		<= lookup_way + 1'h1;
				rd_addr			<= { lookup_hash, lookup_way };
				state			<= STATE_LOOKUP_1;
			end	//end STATE_LOOKUP_0

			//Second phase of a lookup. First read is in pipeline register, no data for us yet.
			//Start reading the next set.
			STATE_LOOKUP_1: begin
				rd_en			<= 1;
				lookup_way		<= lookup_way + 1'h1;
				rd_addr			<= { lookup_hash, lookup_way };
				state			<= STATE_LOOKUP_2;
			end	//end STATE_LOOKUP_1

			//Read data is ready!
			STATE_LOOKUP_2: begin
			end	//end STATE_LOOKUP_2

		endcase

	end

endmodule
