`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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
	@brief	APB conversion block converting a stream of 64-bit writes to multiple 32-bit writes

	64-bit reads are not yet supported, and will be converted to a 32-bit read for now.
	Partial width (strobed) writes are not yet supported.
 */
module APB_WriteBuffer64To32(
	APB.completer 		up,
	APB.requester		down
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Validate data widths

	if(up.DATA_WIDTH != 64)
		apb_bus_width_is_invalid();
	if(down.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();
	if(up.ADDR_WIDTH != down.ADDR_WIDTH)
		apb_address_width_inconsistent();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Forward clock and reset, tie off unused signals

	assign up.pruser	= 0;
	assign up.pbuser	= 0;

	assign down.pprot 	= 0;
	assign down.pwakeup = 0;
	assign down.pauser	= 0;
	assign down.pwuser	= 0;

	assign down.preset_n = up.preset_n;
	assign down.pclk = up.pclk;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The FIFO

	localparam FIFO_WIDTH = 64 + up.ADDR_WIDTH;

	wire					fifo_full;
	wire					fifo_empty;
	logic					fifo_wr;
	logic					fifo_rd;
	wire[up.ADDR_WIDTH-1:0]	fifo_raddr;
	wire[63:0]				fifo_rdata;

	wire					fifo_overflow;

	SingleClockFifo #(
		.WIDTH(FIFO_WIDTH),
		.DEPTH(512),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) fifo (
		.clk(up.pclk),

		.wr(fifo_wr),
		.din({up.paddr, up.pwdata}),

		.rd(fifo_rd),
		.dout({fifo_raddr, fifo_rdata}),

		.overflow(fifo_overflow),
		.underflow(),
		.empty(fifo_empty),
		.full(fifo_full),
		.rsize(),
		.wsize(),

		.reset(!up.preset_n)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Upstream interface logic

	enum logic[2:0]
	{
		DOWN_STATE_IDLE,
		DOWN_STATE_FIRST,
		DOWN_STATE_FIRST_WAIT,
		DOWN_STATE_GAP,
		DOWN_STATE_SECOND,
		DOWN_STATE_SECOND_WAIT,
		DOWN_STATE_READ,
		DOWN_STATE_READ_WAIT
	} down_state = DOWN_STATE_IDLE;

	logic	writing;
	logic	reading;
	always_comb begin

		//we do not propagate errors upstream
		up.pslverr	= 0;

		//clear single cycle stuff
		up.pready	= 0;
		fifo_wr		= 0;
		up.prdata	= 0;

		//ACK writes as soon as we have fifo space
		writing		= up.penable && up.psel && up.pwrite;
		if(writing && !fifo_full) begin
			up.pready	= 1;
			fifo_wr		= 1;
		end

		//Respond to reads (for now, translate to 32-bit single read and tie off LSBs
		reading		= up.penable && up.psel && !up.pwrite;
		if(reading && (down_state == DOWN_STATE_READ_WAIT) && down.pready) begin
			up.pready	= 1;
			up.prdata	= { 32'h0, down.prdata};
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Downstream interface logic

	always_comb begin

		fifo_rd	= !fifo_empty && (down_state == DOWN_STATE_IDLE);

		down.pwrite		= 0;
		down.penable	= 0;
		down.psel		= 0;
		down.pstrb		= 0;
		down.pwdata		= 0;
		down.paddr		= 0;

		case(down_state)

			DOWN_STATE_FIRST: begin
				down.paddr		= fifo_raddr;
				down.pwdata		= fifo_rdata[63:32];
				down.penable	= 1;
				down.pwrite		= 1;
				down.pstrb 		= 4'b1111;
			end

			DOWN_STATE_FIRST_WAIT: begin
				down.paddr		= fifo_raddr;
				down.pwdata		= fifo_rdata[63:32];
				down.penable	= 1;
				down.psel		= 1;
				down.pwrite		= 1;
				down.pstrb 		= 4'b1111;
			end

			DOWN_STATE_SECOND: begin
				down.paddr		= fifo_raddr + 4;
				down.pwdata		= fifo_rdata[31:0];
				down.penable	= 1;
				down.pwrite		= 1;
				down.pstrb 		= 4'b1111;
			end

			DOWN_STATE_SECOND_WAIT: begin
				down.paddr		= fifo_raddr + 4;
				down.pwdata		= fifo_rdata[31:0];
				down.penable	= 1;
				down.psel		= 1;
				down.pwrite		= 1;
				down.pstrb 		= 4'b1111;
			end

			DOWN_STATE_READ: begin
				down.paddr		= up.paddr;
				down.penable	= 1;
			end

			DOWN_STATE_READ_WAIT: begin
				down.paddr		= up.paddr;
				down.penable	= 1;
				down.psel		= 1;
			end

			default: begin
			end

		endcase
	end

	always_ff @(posedge down.pclk or negedge down.preset_n) begin

		if(!down.preset_n) begin
			down_state		<= DOWN_STATE_IDLE;
		end

		else begin

			case(down_state)

				//Start an operation
				DOWN_STATE_IDLE: begin

					//Dispatch reads
					if(fifo_rd)
						down_state	<= DOWN_STATE_FIRST;

					//All writes must have committed before we can do a read
					else if(reading && fifo_empty)
						down_state	<= DOWN_STATE_READ;
				end

				//Start the first write
				DOWN_STATE_FIRST:	down_state	<= DOWN_STATE_FIRST_WAIT;
				DOWN_STATE_FIRST_WAIT: begin
					if(down.pready)
						down_state	<= DOWN_STATE_GAP;
				end

				DOWN_STATE_GAP: 	down_state	<= DOWN_STATE_SECOND;
				DOWN_STATE_SECOND:	down_state	<= DOWN_STATE_SECOND_WAIT;
				DOWN_STATE_SECOND_WAIT: begin
					if(down.pready)
						down_state	<= DOWN_STATE_IDLE;
				end

				DOWN_STATE_READ:	down_state	<= DOWN_STATE_READ_WAIT;
				DOWN_STATE_READ_WAIT: begin
					if(down.pready)
						down_state	<= DOWN_STATE_IDLE;
				end

			endcase

		end

	end

endmodule
