`default_nettype none
`timescale 1ns/1ps
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
	@author	Andrew D. Zonenberg
	@brief	FIFO buffer using hard FIFO IP in Xilinx 7 series

	This module supports ganging multiple hard FIFO IPs and will report empty if any are empty, or full if any are full.

	Depth expansion is not yet supported.

	Generally drop in equivalent for CrossClockFifo with USE_BLOCK=1, OUT_REG=1

	For now, always uses 512x72 configuration in hardware
 */
module CrossClockHardFifo #(
	parameter WIDTH			= 16,
	parameter DEPTH			= 16	//,

	//localparam ADDR_BITS	= $clog2(DEPTH)
) (
	input wire					wr_clk,
	input wire					wr_en,
	input wire[WIDTH-1:0]		wr_data,
	//output wire[ADDR_BITS:0]	wr_size,
	output wire					wr_full,
	output wire					wr_overflow,
	input wire					wr_reset,

	input wire					rd_clk,
	input wire					rd_en,
	output wire[WIDTH-1:0]		rd_data,
	//output wire[ADDR_BITS:0]	rd_size,
	output wire					rd_empty,
	output wire					rd_underflow
);

	(* keep = "yes" *)
	logic[7:0]	rst_shreg	= 8'hff;
	logic		rst			= 1;
	always_ff @(posedge wr_clk) begin

		if(rst)
			rst_shreg	<= {rst_shreg[6:0], 1'b0};

		if(wr_reset)
			rst_shreg	<= 8'hff;

		rst				<= rst_shreg[7];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO primitive instances

	localparam BLOCK_WIDTH = 72;
	localparam BLOCK_DATA = 64;
	localparam BLOCK_PARITY = 8;
	localparam NUM_BLOCKS = int'($ceil(1.0 * WIDTH / BLOCK_WIDTH));

	wire[NUM_BLOCKS-1:0]	wr_full_int;
	wire[NUM_BLOCKS-1:0]	wr_overflow_int;

	wire[NUM_BLOCKS-1:0]	rd_empty_int;
	wire[NUM_BLOCKS-1:0]	rd_underflow_int;

	initial begin
		if(DEPTH > 512)
			$fatal(1, "CrossClockHardFifo currently assumes 512x72 configuration");
	end

	for(genvar g=0; g<NUM_BLOCKS; g=g+1) begin

		FIFO36E1 #(
			.ALMOST_FULL_OFFSET(8'h5),
			.ALMOST_EMPTY_OFFSET(8'h5),
			.FIRST_WORD_FALL_THROUGH("FALSE"),
			.DO_REG(1'b1),
			.DATA_WIDTH(BLOCK_WIDTH),
			.FIFO_MODE("FIFO36_72"),
			.EN_SYN("FALSE"),
			.SRVAL(72'h0),
			.INIT(72'h0)
		) fifo (
			.DI(wr_data[g*BLOCK_WIDTH +: BLOCK_DATA]),
			.DIP(wr_data[g*BLOCK_WIDTH + BLOCK_DATA +: BLOCK_PARITY]),
			.WREN(wr_en && !rst),
			.WRCLK(wr_clk),
			.FULL(wr_full_int[g]),
			.ALMOSTFULL(),
			.WRCOUNT(),
			.WRERR(wr_overflow_int[g]),

			.RDEN(rd_en && !rst),
			.RDCLK(rd_clk),
			.RST(rst),
			.RSTREG(1'b0),
			.REGCE(1'b1),
			.DO(rd_data[g*BLOCK_WIDTH +: BLOCK_DATA]),
			.DOP(rd_data[g*BLOCK_WIDTH + BLOCK_DATA +: BLOCK_PARITY]),
			.EMPTY(rd_empty_int[g]),
			.ALMOSTEMPTY(),
			.RDCOUNT(),
			.RDERR(rd_underflow_int[g])
		);

	end

	assign wr_full 		= |wr_full_int;
	assign wr_overflow	= |wr_overflow_int;
	assign rd_empty		= |rd_empty_int;
	assign rd_underflow	= |rd_underflow_int;

endmodule
