`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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
	@brief Alternate implementation of SingleClockFifo using shift registers as the storage element.

	May use less FPGA resources in some cases.

	Note that the reset line clears the FIFO to the empty state, regardless of the power-on init value

	RESOURCE COMPARISON
		Core/config				LUT		FF		Slice
		RAM 32x4				x
		SRL 32x4				x
 */
module SingleClockShiftRegisterFifo(
	clk,
	wr, din,
	rd, dout,
	overflow, underflow, empty, full, rsize, wsize, reset

	`ifdef FORMAL
	, dout_formal
	`endif
    );

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter declarations

	parameter WIDTH = 32;
	parameter DEPTH = 32;

	//number of bits in the address bus
	`include "../synth_helpers/clog2.vh"
	localparam ADDR_BITS = clog2(DEPTH);

	//USE_BLOCK not supported for anything but zero
	//Keep the parameter around for drop-in compatibility with SingleClockFifo
	parameter USE_BLOCK = 0;

	//Specifies the register mode for outputs.
	//When FALSE:
	// * dout updates on the clk edge after a write if the fifo is empty
	// * read dout whenever empty is false, then strobe rd to advance pointer
	//When TRUE:
	// * dout updates on the clk edge after a read when the fifo has data in it
	// * assert rd, read dout the following cycle
	// * dout is stable until next rd pulse
	parameter OUT_REG = 1;

	// INIT_* not supported, always initialized to empty for the time being

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IO declarations

	input wire					clk;

	input wire					wr;
	input wire[WIDTH-1:0]		din;

	input wire					rd;
	output wire[WIDTH-1:0]		dout;

	output reg					overflow = 0;
	output reg					underflow = 0;

	output wire					empty;
	output wire					full;

	output wire[ADDR_BITS:0]	rsize;
	output wire[ADDR_BITS:0]	wsize;

	input wire					reset;

	`ifdef FORMAL
	output reg[WIDTH*DEPTH-1 : 0] dout_formal;
	`endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sanity checks

	initial begin
		if(DEPTH > 32) begin
			$display("ERROR: [SingleClockShiftRegisterFIFO] DEPTH must be <= 32");
			$finish;
		end

		if(USE_BLOCK) begin
			$display("ERROR: [SingleClockShiftRegisterFIFO] USE_BLOCK must be zero");
			$finish;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control logic

	//Read pointer is cleared to -1 (empty)
	localparam					EMPTY = {1'b1, {ADDR_BITS{1'b1}}};

	//Read pointer
	reg[ADDR_BITS:0]			rpos = EMPTY;

	//Incremented read pointer if we're writing but not reading
	wire[ADDR_BITS:0]			irpos = rpos + 1'd1;

	//Decremented read pointer if we're reading but not writing
	wire[ADDR_BITS:0]			drpos = rpos - 1'd1;

	assign empty				= (rpos == EMPTY);			//if read pointer is at address -1, we have nothing to read
	assign full					= (rpos == DEPTH - 1'b1);	//if read pointer is at far end of shreg, we're full

	//The number of values currently ready to read
	//rpos = -1			empty
	//rpos = 0			1 word
	//etc
	assign rsize				= irpos;

	//The number of spaces available for us to write to
	//rpos = -1			DEPTH
	//rpos = 0			DEPTH-1
	//etc
	assign wsize				= DEPTH[ADDR_BITS:0] - 1'd1 - rpos;

	//Filtered read/write flags (only move pointer if the requested operation was legal)
	wire rd_gated				= (rd && !empty);
	wire wr_gated				= (wr && !full);

	//Main state logic
	always @(posedge clk) begin

		overflow <= 0;
		underflow <= 0;

		//Read
		if(rd) begin

			//Empty? Can't do anything
			if(empty) begin
				underflow <= 1;
				`ifdef SIMULATION
				$display("[SingleClockShiftRegisterFifo] %m WARNING: Underflow occurred!");
				`endif
			end

			//Read pointer goes DOWN since we now are reading earlier in the buffer (there's less stuff in it).
			//Do *not* bump pointer if we're also writing, since then the shreg is moving and we don't have to
			else if(!wr_gated)
				rpos <= drpos;

		end

		//Write only
		if(wr) begin

			//Full? Error
			if(full) begin
				overflow <= 1;
				`ifdef SIMULATION
				$display("[SingleClockShiftRegisterFifo] %m WARNING: Overflow occurred!");
				`endif
			end

			//Read pointer goes UP since we are now reading later in the buffer (there's more stuff in it).
			//Do *not* bump pointer if we're also reading
			else if(!rd_gated)
				rpos <= irpos;

		end

		//read during write when empty not legal

		//Reset takes precedence over everything else
		if(reset)
			rpos <= EMPTY;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The memory

	//Readout data
	wire[WIDTH-1:0]				dout_raw;

	//Optional output register
	reg[WIDTH-1:0]				dout_ff = 0;
	always @(posedge clk) begin
		if(rd)
			dout_ff	<= dout_raw;
	end

	//Mux the output
	assign						dout = OUT_REG ? dout_ff : dout_raw;

	//Trim off top bit and pad read address out to 5 bits as needed
	wire[ADDR_BITS+5 : 0]		rpos_padded = {5'b0, rpos[ADDR_BITS-1:0]};

	`ifdef FORMAL
	wire[WIDTH*DEPTH-1 : 0] dout_concat;
	`endif

	//The actual shreg block
	ShiftRegisterMacro #(
		.WIDTH(WIDTH),
		.DEPTH(DEPTH),
		.ADDR_BITS(5)
	) shreg (
		.clk(clk),
		.addr(rpos_padded[4:0]),
		.din(din),
		.ce(wr_gated),
		.dout(dout_raw)

		`ifdef FORMAL
		, .dout_concat(dout_concat)
		`endif
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Formal verification helper

	`ifdef FORMAL

		integer i;
		always @(*) begin

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
