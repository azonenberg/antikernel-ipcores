`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2022 Andrew D. Zonenberg                                                                          *
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
	@brief A single-clock FIFO

	Note that the reset line clears the FIFO to the empty state, regardless of the power-on init value
 */
module SingleClockFifo #(
	parameter WIDTH = 32,
	parameter DEPTH = 512,

	localparam ADDR_BITS = $clog2(DEPTH),

	//set true to use block RAM, false for distributed RAM
	parameter USE_BLOCK = 1,

	//Specifies the register mode for outputs.
	//When FALSE:
	// * dout updates on the clk edge after a write if the fifo is empty
	// * read dout whenever empty is false, then strobe rd to advance pointer
	//When TRUE:
	// * dout updates on the clk edge after a read when the fifo has data in it
	// * assert rd, read dout the following cycle
	// * dout is stable until next rd pulse
	parameter OUT_REG = 1,

	//Initialize to address (takes precedence over INIT_FILE)
	parameter INIT_ADDR = 0,

	//Initialization file (set to empty string to fill with zeroes)
	parameter INIT_FILE = "",

	//Default if neither is set is to initialize to zero

	//Set to true for the FIFO to begin in the "full" state
	parameter INIT_FULL = 0
)(
	input wire					clk,

	input wire					wr,
	input wire[WIDTH-1:0]		din,

	input wire					rd,
	output wire[WIDTH-1:0]		dout,

	output logic				overflow = 0,
	output logic				underflow = 0,

	output wire					empty,
	output wire					full,

	output wire[ADDR_BITS:0]	rsize,
	output wire[ADDR_BITS:0]	wsize,

	input wire					reset

	`ifdef FORMAL
	,
	output logic[WIDTH*DEPTH-1 : 0] dout_formal
	`endif
    );

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Control logic

	logic[ADDR_BITS:0]			rpos = 0;					//extra bit for full/empty detect
	logic[ADDR_BITS:0] 			wpos = INIT_FULL ? DEPTH : 0;

	wire[ADDR_BITS:0]			irpos = rpos + 1'd1;
	wire[ADDR_BITS:0]			iwpos = wpos + 1'd1;

	assign empty				= (rpos == wpos);			//if write pointer is at read pointer we're empty

	//If write pointer is at far end of buffer, we're full.
	//Overlapping pointers are easily detected: they're equal mod DEPTH, but not equal
	assign full					=	(wpos[ADDR_BITS-1:0] == rpos[ADDR_BITS-1:0]) &&
									(wpos[ADDR_BITS] != rpos[ADDR_BITS]);

	//The number of values currently ready to read
	assign rsize				= wpos - rpos;

	//The number of spaces available for us to write to
	assign wsize				= DEPTH[ADDR_BITS:0] + rpos - wpos;

	always_ff @(posedge clk) begin
		overflow <= 0;
		underflow <= 0;

		//Read
		if(rd) begin

			//Empty? Can't do anything
			if(empty) begin
				underflow <= 1;
				`ifdef SIMULATION
				$display("[SingleClockFifo] %m WARNING: Underflow occurred!");
				`endif
			end

			//All is well, bump stuff
			else begin
				rpos <= irpos;
			end

		end

		//Write only
		if(wr) begin

			//Full? Error
			if(full) begin
				overflow <= 1;
				`ifdef SIMULATION
				$display("[SingleClockFifo] %m WARNING: Overflow occurred!");
				`endif
			end

			//No, just write
			else begin
				wpos <= iwpos;
			end

		end

		//read during write when empty not legal

		//Reset takes precedence over everything else
		if(reset) begin
			rpos <= 0;
			wpos <= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The memory

	`ifdef FORMAL
	//Apply offset so dout_concat is relative to our sliding window.
	//We want the word about to be popped to be at the MSB position.
	wire[WIDTH*DEPTH-1 : 0] dout_concat;
	wire[ADDR_BITS:0] read_offset = wpos;
	`endif

	MemoryMacro #(
		.WIDTH(WIDTH),
		.DEPTH(DEPTH),
		.DUAL_PORT(1),
		.TRUE_DUAL(0),
		.USE_BLOCK(USE_BLOCK),
		.OUT_REG(OUT_REG),
		.PORTA_WRONLY(1),
		.INIT_ADDR(INIT_ADDR),
		.INIT_FILE(INIT_FILE)
	) mem (
		.porta_clk(clk),
		.porta_en(wr),
		.porta_addr(wpos[ADDR_BITS-1 : 0]),
		.porta_we(!full),
		.porta_din(din),
		.porta_dout(),

		.portb_clk(clk),
		.portb_en(rd),
		.portb_addr(rpos[ADDR_BITS-1 : 0]),
		.portb_we(1'b0),
		.portb_din({WIDTH{1'b0}}),
		.portb_dout(dout)

		`ifdef FORMAL
		, .dout_concat_offset(read_offset[ADDR_BITS-1 : 0])
		, .dout_concat(dout_concat)
		`endif
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Formal verification helper

	`ifdef FORMAL

		integer i;
		always_comb begin

			for(i=0; i<DEPTH; i=i+1) begin

				//If we're in the valid part of the memory, output it
				if(i < rsize)
					dout_formal[i*WIDTH +: WIDTH]	<= dout_concat[ (DEPTH - 1 - i)*WIDTH +: WIDTH];

				//Nope, output zeroes
				else
					dout_formal[i*WIDTH +: WIDTH]	<= {WIDTH{1'b0}};

			end
		end

	`endif

endmodule
