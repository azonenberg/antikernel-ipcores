`timescale 1ns/1ps
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

import EthernetBus::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief FIFO for buffering incoming Ethernet frames (in an AXI-Stream bus) and allowing readout over APB
 */
module APB_AXIS_EthernetRxBuffer(

	//The APB bus
	APB.completer 				apb,

	//The incoming frame data (must be in same clock domain as APB)
	//TODO: switch rx_framelen_fifo to dual clock to relax this requirement?
	AXIStream.receiver			axi_rx,
	input wire					eth_link_up,

	//Status flag (usually connected to MCU IRQ line)
	output logic				rx_frame_ready
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[11:0]
	{
		REG_RX_BUF			= 12'h0000,	//start of receive buffer
										//(must be mapped at zero so we can use paddr directly as FIFO read index)

		REG_RX_POP			= 12'h0fc0,	//write any value to pop the current frame
		REG_RX_LEN			= 12'h0fe0	//frame length, in bytes
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	wire[31:0]			rxfifo_rd_data;
	logic				rxheader_rd_en;
	wire				rxheader_rd_empty;
	wire[10:0]			rxheader_rd_data;
	logic				rxfifo_pop_packet;
	logic[12:0]			rxfifo_packet_size;

	logic				apb_pready_next;

	//Combinatorial readback, but with one cycle of latency because of registered reads
	always_comb begin

		apb_pready_next	= apb.psel && apb.penable && !apb.pready;

		rx_frame_ready = !rxheader_rd_empty;

		apb.prdata	= 0;
		apb.pslverr	= 0;

		//this is one cycle after the request is valid due to pipeline latency on reads
		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin

				if(apb.paddr == REG_RX_LEN)
					apb.prdata	= { 21'h0, rxheader_rd_data[10:0] };

				//can't read from pop register
				else if(apb.paddr == REG_RX_POP)
					apb.pslverr = 1;

				//anything else is reading from the rx buffer
				//(no endian swap needed, AXI byte ordering matches what the MCU expects)
				else
					apb.prdata = rxfifo_rd_data;
			end

			//write
			else begin
				if(apb.paddr != REG_RX_POP)
					apb.pslverr	 = 1;
			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			apb.pready			<= 0;
			rxheader_rd_en		<= 0;
			rxfifo_pop_packet	<= 0;
			rxfifo_packet_size	<= 0;
		end

		//Normal path
		else begin

			rxheader_rd_en		<= 0;
			rxfifo_pop_packet	<= 0;

			//Register request flags
			//address/write data don't need to be registered, they'll be kept stable
			apb.pready		<= apb_pready_next;

			//Pop at the end of a packet
			if(apb.pready && apb.pwrite && (apb.paddr == REG_RX_POP) ) begin

				//Header is no longer valid
				rxheader_rd_en		<= 1;

				//Pop it (convert length from bytes to words, rounding up)
				rxfifo_pop_packet	<= 1;
				if(rxheader_rd_data[1:0])
					rxfifo_packet_size	<= rxheader_rd_data[10:2] + 1;
				else
					rxfifo_packet_size	<= rxheader_rd_data[10:2];
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main FIFO and push logic

	//No need to handle drop requests because we're on the other side of an EthernetRxClockCrossing
	//which will drop anything with a bad FCS itself

	logic		rxfifo_wr_en		= 0;
	logic		rxfifo_wr_commit	= 0;
	logic[31:0]	rxfifo_wr_data		= 0;

	wire[12:0]	rxfifo_wr_size;
	wire[12:0]	rxfifo_rd_size;
	logic		rxfifo_wr_drop;

	wire		fifo_reset;
	assign		fifo_reset	= !apb.preset_n || !eth_link_up && !axi_rx.areset_n;

	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(4096)	//at least 8 packets worth
	) rx_cdc_fifo (
		.wr_clk(axi_rx.aclk),
		.wr_en(rxfifo_wr_en),
		.wr_data(rxfifo_wr_data),
		.wr_reset(fifo_reset),
		.wr_size(rxfifo_wr_size),
		.wr_commit(rxfifo_wr_commit),
		.wr_rollback(rxfifo_wr_drop),

		.rd_clk(apb.pclk),
		.rd_en(apb_pready_next),				//read all the time even if actually reading a status register
												//(reading too much is harmless)
		.rd_offset({2'b0, apb.paddr[11:2]}),	//reading 32 bit words so use word address index
		.rd_pop_single(1'b0),
		.rd_pop_packet(rxfifo_pop_packet),
		.rd_packet_size(rxfifo_packet_size),
		.rd_data(rxfifo_rd_data),
		.rd_size(rxfifo_rd_size),
		.rd_reset(fifo_reset)
	);

	//PUSH SIDE
	logic dropping = 0;
	logic[10:0] framelen = 0;

	wire		header_wfull;
	wire[5:0]	rxheader_rd_size;
	wire[5:0]	rxheader_wr_size;
	wire		rxheader_wr_en;

	assign rxheader_wr_en = axi_rx.tvalid && axi_rx.tready && axi_rx.tlast && !axi_rx.tuser[0] && (framelen != 0);

	SingleClockFifo #(
		.WIDTH(11),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(0)
	) rx_framelen_fifo (
		.clk(axi_rx.aclk),

		.wr(rxheader_wr_en),
		.din(framelen),
		.wsize(rxheader_wr_size),
		.full(header_wfull),
		.overflow(),
		.reset(fifo_reset),

		.rd(rxheader_rd_en),
		.dout(rxheader_rd_data),
		.rsize(rxheader_rd_size),
		.empty(rxheader_rd_empty),
		.underflow()
	);

	always_ff @(posedge axi_rx.aclk or negedge axi_rx.areset_n) begin

		if(!axi_rx.areset_n) begin
			rxfifo_wr_en		<= 0;
			rxfifo_wr_data		<= 0;
			rxfifo_wr_drop		<= 0;
			rxfifo_wr_commit	<= 0;
			framelen			<= 0;
			axi_rx.tready		<= 0;
		end

		else begin

			rxfifo_wr_drop		<= axi_rx.tvalid && axi_rx.tready && axi_rx.tuser[0];
			rxfifo_wr_commit	<= axi_rx.tlast && !axi_rx.tuser[0] && !rxfifo_wr_drop;
			rxfifo_wr_en		<= 0;

			axi_rx.tready		<= (rxheader_wr_size > 1) && (rxfifo_wr_size > 1);

			//Push data as needed
			if(axi_rx.tready && axi_rx.tvalid && axi_rx.tstrb && !axi_rx.tuser[0]) begin
				rxfifo_wr_en	<= 1;
				rxfifo_wr_data	<= axi_rx.tdata;

				case(axi_rx.tstrb)
					4'b0001:	framelen <= framelen + 1;
					4'b0011:	framelen <= framelen + 2;
					4'b0111:	framelen <= framelen + 3;
					4'b1111:	framelen <= framelen + 4;
					default:	framelen <= framelen + 4;	//all other values reserved, we don't support gaps in data
				endcase
			end

			//TODO: drop if the frame gets > MTU?

			//Clear frame length after a push or drop
			if(rxfifo_wr_commit || rxfifo_wr_drop)
				framelen	<= 0;

		end
	end

endmodule
