`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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
	@brief Single-clock FIFO for AXI stream data
 */
module AXIS_FIFO #(
	parameter FIFO_DEPTH	= 256,
	parameter USE_BLOCK		= 1,

	localparam ADDR_BITS	= $clog2(FIFO_DEPTH)
)(
	AXIStream.receiver			axi_rx,
	AXIStream.transmitter		axi_tx,

	output wire[ADDR_BITS:0]	wr_size
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Validate buses are the same size

	if(axi_rx.DATA_WIDTH != axi_tx.DATA_WIDTH)
		axi_bus_width_inconsistent();
	if(axi_rx.USER_WIDTH != axi_tx.USER_WIDTH)
		axi_bus_width_inconsistent();
	if(axi_rx.ID_WIDTH != axi_tx.ID_WIDTH)
		axi_bus_width_inconsistent();
	if(axi_rx.DEST_WIDTH != axi_tx.DEST_WIDTH)
		axi_bus_width_inconsistent();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pass-through on control signals

	assign axi_tx.aclk		= axi_rx.aclk;
	assign axi_tx.twakeup	= axi_rx.twakeup;
	assign axi_tx.areset_n	= axi_rx.areset_n;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The FIFO

	localparam MID_WIDTH = (axi_rx.ID_WIDTH == 0) ? 1 : axi_rx.ID_WIDTH;
	localparam MUSER_WIDTH = (axi_rx.USER_WIDTH == 0) ? 1 : axi_rx.USER_WIDTH;
	localparam MDEST_WIDTH = (axi_rx.DEST_WIDTH == 0) ? 1 : axi_rx.DEST_WIDTH;
	localparam STROBE_WIDTH = (axi_rx.DATA_WIDTH/8);

	typedef struct packed
	{
		logic[axi_rx.DATA_WIDTH-1:0]	tdata;
		logic[MUSER_WIDTH-1:0]			tuser;
		logic[MID_WIDTH-1:0]			tid;
		logic[MDEST_WIDTH-1:0]			tdest;
		logic[STROBE_WIDTH-1:0]			tstrb;
		logic[STROBE_WIDTH-1:0]			tkeep;
		logic							tlast;
	} FifoContents;

	FifoContents wr_data;
	always_comb begin
		wr_data.tdata	= axi_rx.tdata;
		wr_data.tuser	= axi_rx.tuser;
		wr_data.tid		= axi_rx.tid;
		wr_data.tdest	= axi_rx.tdest;
		wr_data.tstrb	= axi_rx.tstrb;
		wr_data.tkeep	= axi_rx.tkeep;
		wr_data.tlast	= axi_rx.tlast;
	end

	logic				rd_en	= 0;
	wire[ADDR_BITS:0]	rd_size;
	FifoContents 		rd_data;

	SingleClockFifo #(
		.WIDTH(axi_rx.DATA_WIDTH + MUSER_WIDTH + MID_WIDTH + MDEST_WIDTH + 2*STROBE_WIDTH + 1),
		.DEPTH(FIFO_DEPTH),
		.USE_BLOCK(USE_BLOCK),
		.OUT_REG(1)	//TODO: support OUT_REG=2
	) fifo (
		.clk(axi_rx.aclk),

		.wr(axi_rx.tvalid && axi_rx.tready),
		.din(wr_data),
		.wsize(wr_size),
		.full(),
		.overflow(),
		.reset(!axi_rx.areset_n),

		.rd(rd_en),
		.dout(rd_data),
		.rsize(rd_size),
		.empty(),
		.underflow()
	);

	assign axi_rx.tready = (wr_size > 1);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pop logic

	assign axi_tx.tstrb = rd_data.tstrb;
	assign axi_tx.tkeep = rd_data.tkeep;
	assign axi_tx.tlast = rd_data.tlast && axi_tx.tvalid;
	assign axi_tx.tdata = rd_data.tdata;
	assign axi_tx.tid	= rd_data.tid;
	assign axi_tx.tdest = rd_data.tdest;
	assign axi_tx.tuser = rd_data.tuser;

	logic	rd_en_ff	= 0;

	//manage pop
	always_comb begin
		rd_en	= 0;

		//if nothing to read, we have nothing to do
		if(rd_size == 0) begin
		end

		//if we have data, only pop if the existing data was accepted, or nothing is on deck
		else if(axi_tx.tready || !axi_tx.tvalid)
			rd_en	= 1;

	end

	//manage TVALID
	always_ff @(posedge axi_tx.aclk or negedge axi_tx.areset_n) begin
		if(!axi_tx.areset_n) begin
			axi_tx.tvalid	<= 0;
			rd_en_ff		<= 0;
		end

		else begin

			rd_en_ff		<= rd_en;

			if(axi_tx.tready)
				axi_tx.tvalid	<= 0;
			if(rd_en)
				axi_tx.tvalid	<= 1;
		end
	end

endmodule
