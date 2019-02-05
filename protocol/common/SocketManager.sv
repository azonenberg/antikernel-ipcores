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

`include "SocketBus.svh"

/**
	@brief Manage mappings of {srcIP, srcport, dstport} to socket IDs
 */
module SocketManager #(
	parameter	WAYS		= 4,	//number of concurrent lookups
	parameter	LATENCY		= 2,	//number of cycles we can spend on a lookup
	parameter	BINS		= 256,	//number of hash bins
	localparam	COL_BITS	= $clog2(WAYS),
	localparam	LAT_BITS	= $clog2(LATENCY),
	localparam	WAY_SIZE	= LATENCY * BINS,
	localparam	WAY_BITS	= $clog2(WAY_SIZE),
	localparam	CACHE_SIZE	= WAYS * WAY_SIZE,
	localparam	CACHE_BITS	= $clog2(CACHE_SIZE)
) (

	input wire						clk,

	//Look up a socket (handling incoming packet)
	input wire						lookup_en,
	input wire socketstate_t		lookup_headers,
	output logic					lookup_done		= 0,
	output logic					lookup_hit		= 0,
	output logic[CACHE_BITS-1:0]	lookup_sockid	= 0,

	//Insert a new entry into the table
	input wire						insert_en,
	input wire socketstate_t		insert_headers,
	output logic					insert_done		= 0,
	output logic[CACHE_BITS-1:0]	insert_sockid	= 0,
	output logic					insert_fail		= 0,

	//Destroy a table entry
	input wire						remove_en,
	input wire[CACHE_BITS-1:0]		remove_sockid,
	output logic					remove_done		= 0,

	//Flush stale sockets
	input wire						aging_tick,
	input wire[9:0]					max_age			//entries older than this are deleted during the next aging pass
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual cache memory

	typedef struct packed
	{
		logic			valid;
		logic[9:0]		age;
		socketstate_t	headers;
	} sockentry_t;

	localparam LINE_WIDTH = $bits(sockentry_t);

	//Write data is directed to a specific block, rest are left untouched
	logic[WAYS-1:0]		wr_en	= 0;
	logic[WAY_BITS-1:0]	wr_addr	= 0;
	sockentry_t			wr_entry = {LINE_WIDTH{1'h0}};

	//Reads come from the whole cache at once, can't address only one way
	logic				rd_en	= 0;
	logic[WAY_BITS-1:0]	rd_addr	= 0;
	sockentry_t			rd_entry[WAYS-1:0];

	genvar g;
	generate

		for(g=0; g<WAYS; g=g+1) begin

			wire[LINE_WIDTH-1:0]	rd_entry_raw;
			assign rd_entry[g] = sockentry_t'(rd_entry_raw);

			MemoryMacro #(
				.WIDTH(LINE_WIDTH),
				.DEPTH(WAY_SIZE),
				.USE_BLOCK(1),
				.OUT_REG(1),
				.DUAL_PORT(1),
				.INIT_VALUE({LINE_WIDTH{1'h0}})
			) cache_mem (

				.porta_clk(clk),
				.porta_en(wr_en[g]),
				.porta_addr(wr_addr),
				.porta_we(wr_en[g]),
				.porta_din(wr_entry),
				.porta_dout(),

				.portb_clk(clk),
				.portb_en(rd_en),
				.portb_addr(rd_addr),
				.portb_we(1'b0),
				.portb_din({LINE_WIDTH{1'h0}}),
				.portb_dout(rd_entry_raw)
			);

		end

	endgenerate

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Lookup helpers

	//Calculate the hash of the incoming packets
	wire[15:0] lookup_hash_raw =
		lookup_headers.client_port ^
		lookup_headers.server_port ^
		lookup_headers.address[31:16] ^
		lookup_headers.address[15:0];

	wire[15:0] insert_hash_raw =
		insert_headers.client_port ^
		insert_headers.server_port ^
		insert_headers.address[31:16] ^
		insert_headers.address[15:0];

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	enum logic[3:0]
	{
		STATE_IDLE		= 4'h0,
		STATE_LOOKUP_0	= 4'h1,
		STATE_LOOKUP_1	= 4'h2,
		STATE_INSERT_0	= 4'h3,
		STATE_INSERT_1	= 4'h4,
		STATE_REMOVE_0	= 4'h5
	} state = STATE_IDLE;

	logic[WAY_BITS-1:0]	rd_addr_ff	= 0;

	integer i;
	always_ff @(posedge clk) begin

		rd_en		<= 0;
		wr_en		<= 0;
		lookup_done	= 0;
		lookup_hit	<= 0;
		insert_done	= 0;
		insert_fail	<= 0;
		remove_done	<= 1;

		rd_addr_ff	<= rd_addr;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for stuff to happen

			STATE_IDLE: begin

				//New packet arrived!
				if(lookup_en) begin
					`ifdef SIMULATION
						$display("[%t] [%m] Looking up %0d.%0d.%0d.%0d:%0d -> localhost:%0d (hash %x)",
							$time(),
							lookup_headers.address[31:24],
							lookup_headers.address[23:16],
							lookup_headers.address[15:8],
							lookup_headers.address[7:0],
							lookup_headers.client_port,
							lookup_headers.server_port,
							lookup_hash_raw);
					`endif

					rd_en	<= 1;
					rd_addr	<= { lookup_hash_raw[WAY_BITS-1:LAT_BITS], {LAT_BITS{1'h0}} };
					state	<= STATE_LOOKUP_0;
				end

				//Process SYN+ACK
				else if(insert_en) begin
					`ifdef SIMULATION
						$display("[%t] [%m] Inserting %0d.%0d.%0d.%0d:%0d -> localhost:%0d (hash %x)",
							$time(),
							insert_headers.address[31:24],
							insert_headers.address[23:16],
							insert_headers.address[15:8],
							insert_headers.address[7:0],
							insert_headers.client_port,
							insert_headers.server_port,
							insert_hash_raw);
					`endif

					rd_en	<= 1;
					rd_addr	<= { insert_hash_raw[WAY_BITS-1:LAT_BITS], {LAT_BITS{1'h0}} };
					state	<= STATE_INSERT_0;
				end

				//Process FIN/RST
				else if(remove_en) begin
					`ifdef SIMULATION
						$display("[%t] [%m] Closing socket %x",
							$time(),
							remove_sockid);
					`endif

					wr_en[remove_sockid[COL_BITS-1:0]]	<= 1;
					wr_addr								<= remove_sockid[CACHE_BITS-1 : COL_BITS];
					wr_entry							<= {$bits(sockentry_t){1'b0}};
					remove_done							<= 1;

				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read path

			//Pipeline delay, nothing to read yet
			STATE_LOOKUP_0: begin
				rd_en	<= 1;
				rd_addr	<= rd_addr + 1'h1;
				state	<= STATE_LOOKUP_1;
			end	//end STATE_LOOKUP_0

			//Check all WAYS memory outputs and see if any of them match what we're looking for
			STATE_LOOKUP_1: begin

				rd_en	<= 1;
				rd_addr	<= rd_addr + 1'h1;

				for(i=0; i<WAYS; i=i+1) begin

					//Stop looking if we got a hit
					if(lookup_done) begin
					end

					//Tag must match on a valid entry to hit
					else if(rd_entry[i].valid && (rd_entry[i].headers == lookup_headers) ) begin

						`ifdef SIMULATION
							$display("[%t] [%m] Hit (way %0d, address %x), returning socket %x",
								$time(), i, rd_addr_ff, {rd_addr_ff, i[COL_BITS-1:0]});
						`endif

						lookup_done		= 1;
						lookup_hit		<= 1;
						lookup_sockid	<= {rd_addr_ff, i[COL_BITS-1:0]};

						//TODO: if we hit, update the aging timer
						state			<= STATE_IDLE;
					end

				end

				//If we've just read the LAST candidate entry, with no hits, give up
				if(!lookup_done && (rd_addr[LAT_BITS-1:0] == {LAT_BITS{1'h1}}) ) begin

					`ifdef SIMULATION
						$display("[%t] [%m] Not found in cache", $time());
					`endif

					lookup_done	= 1;
					lookup_hit	<= 0;
					state		<= STATE_IDLE;
				end

			end	//end STATE_LOOKUP_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Insert path

			//Pipeline delay, nothing to read yet
			STATE_INSERT_0: begin
				rd_en	<= 1;
				rd_addr	<= rd_addr + 1'h1;
				state	<= STATE_INSERT_1;
			end	//end STATE_INSERT_0

			//Check all WAYS memory outputs and see if any of them are empty
			STATE_INSERT_1: begin

				rd_en	<= 1;
				rd_addr	<= rd_addr + 1'h1;

				for(i=0; i<WAYS; i=i+1) begin

					//Stop looking if we got a hit
					if(insert_done) begin
					end

					//Free slot? Insert there
					else if(!rd_entry[i].valid) begin

						`ifdef SIMULATION
							$display("[%t] [%m] Inserting new entry (way %0d, address %x), returning socket %x",
								$time(), i, rd_addr_ff, {rd_addr_ff, i[COL_BITS-1:0]});
						`endif

						insert_done			= 1;
						insert_sockid		<= {rd_addr_ff, i[COL_BITS-1:0]};

						wr_en[i]			<= 1;
						wr_addr				<= rd_addr_ff;
						wr_entry.valid		<= 1;
						wr_entry.age		<= 0;
						wr_entry.headers	<= insert_headers;

						state				<= STATE_IDLE;
					end

				end

				//If we've just read the LAST candidate entry, with no hits, give up
				if(!insert_done && (rd_addr[LAT_BITS-1:0] == {LAT_BITS{1'h1}}) ) begin

					`ifdef SIMULATION
						$display("[%t] [%m] No free table entries", $time());
					`endif

					insert_done	= 1;
					insert_fail	<= 1;
					state		<= STATE_IDLE;
				end

			end	//end STATE_INSERT_1

		endcase

	end

endmodule
