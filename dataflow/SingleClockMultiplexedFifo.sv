`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief A set of multiple single-clock FIFOs sharing a single memory array.

	Each FIFO has independent pointers and non-overlapping data storage within the array, however a single data bus
	provides access to all of them.
 */
module SingleClockMultiplexedFifo #(
	parameter WIDTH 	= 32,
	parameter DEPTH		= 512,
	parameter CHANNELS	= 4,

	//total memory size across all buffers
	localparam MDEPTH	= DEPTH * CHANNELS,

	localparam ADDR_BITS	= $clog2(DEPTH),
	localparam MADDR_BITS	= $clog2(MDEPTH),
	localparam CHANNEL_BITS	= $clog2(CHANNELS),

	//set true to use block RAM, false for distributed RAM
	parameter USE_BLOCK = 1

	//OUT_REG mode 1 is the only supported state for now
	//TODO: support OUT_REG=2
) (
	input wire						clk,

	input wire						wr,
	input wire[CHANNEL_BITS-1:0]	wr_channel,
	input wire[WIDTH-1:0]			wr_data,

	input wire						rd,
	input wire[CHANNEL_BITS-1:0]	rd_channel,
	output wire[WIDTH-1:0]			rd_data,

	output logic					overflow = 0,
	output logic					underflow = 0,

	output logic[CHANNELS-1:0]		empty,
	output logic[CHANNELS-1:0]		full,

	input wire						reset
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pointers

	logic[ADDR_BITS:0] rpos[CHANNELS-1:0];
	logic[ADDR_BITS:0] wpos[CHANNELS-1:0];

	initial begin
		for(integer i=0; i<CHANNELS; i=i+1) begin
			rpos[i] = 0;
			wpos[i] = 0;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memory buffer

	MemoryMacro #(
		.WIDTH(WIDTH),
		.DEPTH(MDEPTH),
		.USE_BLOCK(1),
		.OUT_REG(1),
		.DUAL_PORT(1),
		.TRUE_DUAL(0),
		.PORTA_WRONLY(1)
	) mem (
		.porta_clk(clk),
		.porta_en(wr),
		.porta_addr({wr_channel, wpos[wr_channel][ADDR_BITS-1:0]}),
		.porta_we(wr),
		.porta_din(wr_data),
		.porta_dout(),

		.portb_clk(clk),
		.portb_en(rd),
		.portb_addr({rd_channel, rpos[rd_channel][ADDR_BITS-1:0]}),
		.portb_we(1'b0),
		.portb_din({WIDTH{1'b0}}),
		.portb_dout(rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Control logic

	logic[ADDR_BITS:0] irpos[CHANNELS-1:0];
	logic[ADDR_BITS:0] iwpos[CHANNELS-1:0];

	always_comb begin
		for(integer i=0; i<CHANNELS; i=i+1) begin
			irpos[i]	= rpos[i] + 1'd1;
			iwpos[i]	= wpos[i] + 1'd1;

			empty[i]	= (rpos == wpos);	//if write pointer is at read pointer we're empty

			//If write pointer is at far end of buffer, we're full.
			//Overlapping pointers are easily detected: they're equal mod DEPTH, but not equal
			full[i]		= (wpos[i][ADDR_BITS-1:0] == rpos[i][ADDR_BITS-1:0]) &&
						  (wpos[i][ADDR_BITS] != rpos[i][ADDR_BITS]);
		end
	end

	/*
	//The number of values currently ready to read
	assign rsize				= wpos - rpos;

	//The number of spaces available for us to write to
	assign wsize				= DEPTH[ADDR_BITS:0] + rpos - wpos;
	*/

	always_ff @(posedge clk) begin

		overflow	<= 0;
		underflow	<= 0;

		//Read
		if(rd) begin

			//Empty? Can't do anything
			if(empty[rd_channel]) begin
				underflow <= 1;
				`ifdef SIMULATION
				$display("[%m] WARNING: Underflow occurred!");
				`endif
			end

			//All is well, bump stuff
			else
				rpos[rd_channel] <= irpos[rd_channel];

		end

		//Write only
		if(wr) begin

			//Full? Error
			if(full[wr_channel]) begin
				overflow <= 1;
				`ifdef SIMULATION
				$display("[%m] WARNING: Overflow occurred!");
				`endif
			end

			//No, just write
			else
				wpos[wr_channel] <= iwpos[wr_channel];

		end

		//read during write when empty not legal

		//Reset takes precedence over everything else
		if(reset) begin
			for(integer i=0; i<CHANNELS; i=i+1) begin
				rpos[i] <= 0;
				wpos[i] <= 0;
			end
		end

	end

endmodule
