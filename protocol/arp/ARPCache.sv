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
	PERFORMANCE

	Insertions and lookups each take NUM_WAYS cycles plus a bit of setup time. Worst case latency for a lookup is when
	it's submitted one cycle after a lookup so approximately 2*(NUM_WAYS + 3) cycles.

	For the default configuration NUM_WAYS=4, worst case latency is 14 cycles.

	Clock rates required to sustain various data rates:
		At 10 Gbps:
			Max 20 Mpps, times 14 clocks per packet = 280 MHz
		At 1 Gbps:
			Max 2 Mpps, times 14 clocks per packet = 28 MHz

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

	If a new learn request comes in before the old one has completed, the new one is silently dropped.

	Every inbound packet should result in the address in question being re-learned to keep its cache entry fresh.

	====================================================================================================================
	CACHE LINE AGING

	aging_tick needs to be asserted (for one clock) periodically so old cache entries can be flushed. Ticks can be
	spaced as far apart as desired for a given cache lifetime, however there must be a minimum of LINES_PER_WAY *
	NUM_WAYS * 4 idle cycles (with no lookups or learning in progress) between ticks to ensure that the table can be fully
	walked.

	This is unlikely to be an issue outside of accelerated-timescale simulation since a typical real-world aging timer
	would run at 1 Hz or less.
 */
module ARPCache #(
	parameter			LINES_PER_WAY	= 128,
	parameter			NUM_WAYS		= 4
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

	input wire			aging_tick,
	input wire[14:0]	max_age					//entries older than this are deleted during the next aging pass
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual ARP table

	localparam			CACHE_LINES	= NUM_WAYS * LINES_PER_WAY;
	localparam			ADDR_BITS	= $clog2(CACHE_LINES);
	localparam			LINE_BITS	= $clog2(LINES_PER_WAY);
	localparam			WAY_BITS	= $clog2(NUM_WAYS);

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

	logic[ADDR_BITS-1:0]	rd_addr_ff		= 0;
	logic[ADDR_BITS-1:0]	rd_data_addr	= 0;		//the address associated with rd_data

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
		if(rd_en) begin
			rd_data_raw	<= arp_tbl[rd_addr];
			rd_addr_ff	<= rd_addr;
		end
		rd_data			<= rd_data_raw;
		rd_data_addr	<= rd_addr_ff;

		//Write
		if(wr_en)
			arp_tbl[wr_addr]	<= wr_data;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address hashing

	logic[31:0]			lookup_hash_raw;
	wire[LINE_BITS-1:0]	lookup_hash	= lookup_hash_raw[LINE_BITS-1:0];

	logic[31:0]			learn_hash_raw;
	wire[LINE_BITS-1:0]	learn_hash	= learn_hash_raw[LINE_BITS-1:0];

	always_comb begin
		lookup_hash_raw	<= lookup_ip[7:0] + lookup_ip[15:8] + lookup_ip[23:16] + lookup_ip[31:24] ^ lookup_ip[15:0];
		learn_hash_raw	<= learn_ip[7:0] + learn_ip[15:8] + learn_ip[23:16] + learn_ip[31:24] ^ learn_ip[15:0];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cache state machine

	enum logic[3:0]
	{
		STATE_IDLE		= 4'h0,
		STATE_LOOKUP_0	= 4'h1,
		STATE_LOOKUP_1	= 4'h2,
		STATE_LOOKUP_2	= 4'h3,
		STATE_LOOKUP_3	= 4'h4,
		STATE_LOOKUP_4	= 4'h5,
		STATE_LEARN_0	= 4'h6,
		STATE_LEARN_1	= 4'h7,
		STATE_LEARN_2	= 4'h8,
		STATE_LEARN_3	= 4'h9,
		STATE_LEARN_4	= 4'ha,
		STATE_LEARN_5	= 4'hb,
		STATE_AGE_0		= 4'hc,
		STATE_AGE_1		= 4'hd
	} state	= STATE_IDLE;

	logic					learn_pending		= 0;
	logic[31:0]				learn_pending_ip	= 0;
	logic[47:0]				learn_pending_mac	= 0;
	logic[LINE_BITS-1:0]	learn_pending_hash	= 0;

	logic					lookup_pending		= 0;

	wire					addr_at_end			= rd_addr[WAY_BITS-1:0] == {WAY_BITS{1'b1}};

	logic[LINE_BITS-1:0]	target_hash			= 0;
	logic[31:0]				target_ip			= 0;
	wire					ip_match			= rd_data.valid && (rd_data.ip == target_ip);

	//Oldest cache line, for replacement purposes
	logic[14:0]				best_age			= 0;
	logic[ADDR_BITS-1:0]	best_addr			= 0;

	logic					age_pending			= 0;
	logic[ADDR_BITS-1:0]	age_addr			= 0;

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
			learn_pending_hash	<= learn_hash;
		end

		if(lookup_en)
			lookup_pending		<= 1;
		if(aging_tick)
			age_pending			<= 1;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Idle

			//Sit around and wait
			STATE_IDLE: begin

				//Lookups are top priority. Do them first.
				if(lookup_en || lookup_pending) begin

					lookup_pending	<= 0;

					rd_en			<= 1;
					rd_addr			<= { lookup_hash, {WAY_BITS{1'b0}} };
					state			<= STATE_LOOKUP_0;

					target_hash		<= lookup_hash;
					target_ip		<= lookup_ip;

					lookup_mac		<= 48'hff_ff_ff_ff_ff_ff;	//default to broadcast if we can't find it

					$display("[%t] Looking up IP %d.%d.%d.%d",
						$time(), lookup_ip[31:24], lookup_ip[23:16], lookup_ip[15:8], lookup_ip[7:0]);
				end

				//Nope, check for stuff to learn
				else if(learn_pending) begin

					rd_en			<= 1;
					rd_addr			<= { learn_hash, {WAY_BITS{1'b0}} };
					state			<= STATE_LEARN_0;

					target_hash		<= learn_hash;
					target_ip		<= learn_pending_ip;

					$display("[%t] Learning IP %d.%d.%d.%d is at MAC %02x:%02x:%02x:%02x:%02x:%02x",
						$time(),
						learn_pending_ip[31:24], learn_pending_ip[23:16], learn_pending_ip[15:8], learn_pending_ip[7:0],
						learn_pending_mac[47:40], learn_pending_mac[39:32], learn_pending_mac[31:24],
						learn_pending_mac[23:16], learn_pending_mac[15:8], learn_pending_mac[7:0]
						);

					//To start, any cache line is suitable to replace
					best_age		<= 0;

				end

				//Lowest priority: aging check
				else if(age_pending) begin
					$display("[%t] Starting aging pass", $time());
					if(age_addr != 0)
						$display("WARNING: previous pass wasn't done!");
					rd_en			<= 1;
					rd_addr			<= 0;
					age_pending		<= 0;
					state			<= STATE_AGE_0;
				end
				else if(age_addr != 0) begin
					rd_en			<= 1;
					rd_addr			<= age_addr;
					state			<= STATE_AGE_0;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Table entry aging

			//Read is running
			STATE_AGE_0: begin
				if(!rd_en)
					state			<= STATE_AGE_1;
			end	//end STATE_AGE_0

			//Data available.
			STATE_AGE_1: begin

				//Write back the cache line.
				//Default to not changing anything
				wr_en				<= rd_data.valid;
				wr_addr				<= rd_addr;
				wr_data				<= rd_data;

				//If it's too old, invalidate it.
				if(rd_data.valid && (rd_data.age > max_age) ) begin
					wr_data.valid	<= 0;

					$display("[%t] Cache entry for IP %d.%d.%d.%d at MAC %02x:%02x:%02x:%02x:%02x:%02x has aged out",
						$time(),
						rd_data.ip[31:24], rd_data.ip[23:16], rd_data.ip[15:8], rd_data.ip[7:0],
						rd_data.mac[47:40], rd_data.mac[39:32], rd_data.mac[31:24],
						rd_data.mac[23:16], rd_data.mac[15:8], rd_data.mac[7:0]);
				end

				//If not, it just got older
				else if(rd_data.valid)
					wr_data.age		<= rd_data.age + 1'h1;

				//Either way we're ready to do the next line.
				//Go back to idle first, and let higher priority stuff run if needed.
				age_addr			<= rd_addr + 1'h1;
				state				<= STATE_IDLE;

			end	//end STATE_AGE_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Address lookup

			//First phase of a lookup. Read just kicked off, no data for us yet.
			//Start reading the next set.
			STATE_LOOKUP_0: begin
				rd_en					<= 1;
				rd_addr[WAY_BITS-1:0]	<= rd_addr[WAY_BITS-1:0] + 1'h1;
				state					<= STATE_LOOKUP_1;
			end	//end STATE_LOOKUP_0

			//Second phase of a lookup. First read is in pipeline register, no data for us yet.
			//Start reading the next set.
			STATE_LOOKUP_1: begin
				rd_en					<= 1;
				rd_addr[WAY_BITS-1:0]	<= rd_addr[WAY_BITS-1:0] + 1'h1;
				state					<= STATE_LOOKUP_2;
			end	//end STATE_LOOKUP_1

			//Read data is ready!
			STATE_LOOKUP_2: begin

				//We've checked every way of the cache. Go test straggling results but don't request more
				if(addr_at_end)
					state		<= STATE_LOOKUP_3;

				//Request another cache way
				else begin
					rd_en					<= 1;
					rd_addr[WAY_BITS-1:0]	<= rd_addr[WAY_BITS-1:0] + 1'h1;
				end

			end	//end STATE_LOOKUP_2

			//Check the last result
			STATE_LOOKUP_3: begin
				state				<= STATE_LOOKUP_4;
			end	//end STATE_LOOKUP_3

			STATE_LOOKUP_4: begin

				//We're done checking at this point. Go back to idle, no matter what happened
				lookup_done		<= 1;
				state			<= STATE_IDLE;

			end	//end STATE_LOOKUP_4

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Address learning

			//First phase of a lookup. Read just kicked off, no data for us yet.
			//Start reading the next set.
			STATE_LEARN_0: begin
				rd_en					<= 1;
				rd_addr[WAY_BITS-1:0]	<= rd_addr[WAY_BITS-1:0] + 1'h1;
				state					<= STATE_LEARN_1;
			end	//end STATE_LEARN_0

			//Second phase of a lookup. First read is in pipeline register, no data for us yet.
			//Start reading the next set.
			STATE_LEARN_1: begin
				rd_en					<= 1;
				rd_addr[WAY_BITS-1:0]	<= rd_addr[WAY_BITS-1:0] + 1'h1;
				state					<= STATE_LEARN_2;
			end	//end STATE_LEARN_1

			//Read data is ready!
			STATE_LEARN_2: begin

				//We've checked every way of the cache. Go test straggling results but don't request more
				if(addr_at_end)
					state		<= STATE_LEARN_3;

				//Request another cache way
				else begin
					rd_en					<= 1;
					rd_addr[WAY_BITS-1:0]	<= rd_addr[WAY_BITS-1:0] + 1'h1;
				end

			end	//end STATE_LEARN_2

			//Check the last result
			STATE_LEARN_3: begin
				state				<= STATE_LEARN_4;
			end	//end STATE_LEARN_3

			STATE_LEARN_4: begin
				state				<= STATE_LEARN_5;
			end	//end STATE_LEARN_4

			STATE_LEARN_5: begin

				//We didn't find the address in the table. Need to learn it!
				$display("[%t] Not in cache - learning IP %d.%d.%d.%d at MAC %02x:%02x:%02x:%02x:%02x:%02x",
					$time(),
					learn_ip[31:24], learn_ip[23:16], learn_ip[15:8], learn_ip[7:0],
					learn_mac[47:40], learn_mac[39:32], learn_mac[31:24],
					learn_mac[23:16], learn_mac[15:8], learn_mac[7:0]
					);

				wr_en			<= 1;
				wr_addr			<= best_addr;
				wr_data.valid	<= 1;
				wr_data.age		<= 0;
				wr_data.ip		<= learn_ip;
				wr_data.mac		<= learn_mac;


				//Done
				learn_pending	<= 0;
				state			<= STATE_IDLE;

			end	//end STATE_LEARN_5

		endcase

		//Check if we found the target
		//This is at the end since it can happen in a couple of states
		if(ip_match) begin

			//Lookups
			if( (state == STATE_LOOKUP_2) || (state == STATE_LOOKUP_3) || (state == STATE_LOOKUP_4) ) begin
				lookup_done	<= 1;
				lookup_hit	<= 1;
				lookup_mac	<= rd_data.mac;
				state		<= STATE_IDLE;

				$display("[%t] Hit! IP %d.%d.%d.%d is at MAC %02x:%02x:%02x:%02x:%02x:%02x",
					$time(),
					lookup_ip[31:24], lookup_ip[23:16], lookup_ip[15:8], lookup_ip[7:0],
					rd_data.mac[47:40], rd_data.mac[39:32], rd_data.mac[31:24],
					rd_data.mac[23:16], rd_data.mac[15:8], rd_data.mac[7:0]
					);
			end

			//Learning. If we found the entry already in the cache, update it and go back to idle.
			if( (state == STATE_LEARN_2) || (state == STATE_LEARN_3) || (state == STATE_LEARN_4) ) begin
				$display("[%t] Hit! Refreshing IP %d.%d.%d.%d at MAC %02x:%02x:%02x:%02x:%02x:%02x",
					$time(),
					learn_ip[31:24], learn_ip[23:16], learn_ip[15:8], learn_ip[7:0],
					learn_mac[47:40], learn_mac[39:32], learn_mac[31:24],
					learn_mac[23:16], learn_mac[15:8], learn_mac[7:0]
					);

				wr_en			<= 1;
				wr_addr			<= rd_data_addr;
				wr_data.valid	<= 1;
				wr_data.age		<= 0;
				wr_data.ip		<= learn_ip;
				wr_data.mac		<= learn_mac;

				learn_pending	<= 0;
				state			<= STATE_IDLE;
			end
		end

		//If we did NOT find the target, see if this cache line is worth replacing.
		else begin
			if( (state == STATE_LEARN_2) || (state == STATE_LEARN_3) || (state == STATE_LEARN_4) ) begin

				//Empty line? Can't get better than that!
				if(!rd_data.valid) begin
					best_age	<= 15'h7FFF;
					best_addr	<= rd_data_addr;
				end

				//Valid line, but older than our previous oldest? It might have to get bumped
				else if(rd_data.age >= best_age) begin
					best_age	<= rd_data.age;
					best_addr	<= rd_data_addr;
				end

				//Valid line, but newer? Keep it.

			end
		end

	end

endmodule
