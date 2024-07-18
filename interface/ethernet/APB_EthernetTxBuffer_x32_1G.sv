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

import EthernetBus::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief FIFO for shifting Ethernet frames from the APB clock domain to the management PHY clock domain

	TODO: add overflow checks since at 10/100 speed we might be able to fill the FIFO faster than it drains
 */
module APB_EthernetTxBuffer_x32_1G(
	APB.completer 			apb,

	input wire				tx_clk,
	input wire				link_up,
	input wire				tx_ready,
	output EthernetTxBus	tx_bus
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
	// Register map

	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		REG_STAT	= 'h0000,		//[0] = link up flag
		REG_COMMIT	= 'h0004,		//Write any value to send the current frame
		REG_LENGTH	= 'h0008,		//Write expected frame length (in bytes) here before writing to TX buffer
		REG_TX_BUF	= 'h0010		//Write any address >= here to write to transmit buffer
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronizers for status signals

	wire	link_up_sync;

	ThreeStageSynchronizer sync_link_up(
		.clk_in(tx_clk),
		.din(link_up),
		.clk_out(apb.pclk),
		.dout(link_up_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Serialize the 32-bit data coming in from APB to an 8-bit stream

	logic		wr_en 			= 0;
	logic[7:0]	wr_data 		= 0;
	logic		wr_commit		= 0;

	logic[23:0]	pending_bytes		= 0;
	logic[1:0]	pending_bytes_valid	= 0;

	logic[10:0]	tx_wr_packetlen	= 0;

	wire 		tx_header_full;
	wire 		tx_fifo_full;

	//Combinatorial readback
	always_comb begin

		//Accept transactions immediately unless we have pending data to push, or the buffers are full
		apb.pready	= apb.psel && apb.penable && (pending_bytes_valid == 0) && !tx_header_full && !tx_fifo_full;

		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//write
			if(apb.pwrite) begin

				//Can't write to status register
				if(apb.paddr == REG_STAT)
					apb.pslverr		= 1;

				//everything else is in sequential block

			end

			//read
			else begin

				//Status register readback
				if(apb.paddr == REG_STAT)
					apb.prdata	= { 31'h0, link_up_sync };

				//No other readback allowed (FIFO is write only)
				else
					apb.pslverr	= 1;

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			wr_en				<= 0;
			wr_data				<= 0;
			wr_commit			<= 0;
			tx_wr_packetlen		<= 0;
			pending_bytes		<= 0;
			pending_bytes_valid	<= 0;
		end

		//Normal path
		else begin

			wr_en			<= 0;
			wr_commit		<= 0;

			//Increment word count as we push
			if(wr_en)
				tx_wr_packetlen	<= tx_wr_packetlen + 1;

			//Push rest of a word
			if(pending_bytes_valid) begin
				wr_en				<= 1;
				wr_data				<= pending_bytes[7:0];
				pending_bytes		<= { 8'h0, pending_bytes[23:16] };
				pending_bytes_valid	<= pending_bytes_valid - 1;
			end

			//Reset state after a push completes
			if(wr_commit) begin
				tx_wr_packetlen		<= 0;
				pending_bytes		<= 0;
				pending_bytes_valid	<= 0;
			end

			if(apb.pready && apb.pwrite) begin

				//Commit an in-progress packet
				if(apb.paddr == REG_COMMIT)
					wr_commit		<= 1;

				//Write to the transmit buffer
				else if(apb.paddr >= REG_TX_BUF) begin

					//Push the LSB either way
					wr_en			<= 1;
					wr_data			<= apb.pwdata[7:0];

					//Other bytes valid too? Save and push later
					//(assume contiguous byte range)
					if(apb.pstrb[3] == 1) begin
						pending_bytes		<= apb.pwdata[31:8];
						pending_bytes_valid	<= 3;
					end
					else if(apb.pstrb[2] == 1) begin
						pending_bytes		<= apb.pwdata[23:8];
						pending_bytes_valid	<= 2;
					end
					else if(apb.pstrb[1] == 1) begin
						pending_bytes		<= apb.pwdata[15:8];
						pending_bytes_valid	<= 1;
					end

				end

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual FIFOs

	wire		rd_reset;
	assign		rd_reset = !link_up;

	wire		wr_reset;

	logic		txfifo_rd_en			= 0;
	wire[7:0]	txfifo_rd_data;

	//Tie off unused high bits
	assign tx_bus.data[31:8] = 0;
	assign tx_bus.bytes_valid = 1;

	ThreeStageSynchronizer sync_fifo_rst(
		.clk_in(apb.pclk),
		.din(rd_reset),
		.clk_out(tx_clk),
		.dout(wr_reset)
	);
	CrossClockFifo #(
		.WIDTH(8),
		.DEPTH(4096),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) tx_cdc_fifo (
		.wr_clk(apb.pclk),
		.wr_en(wr_en),
		.wr_data(wr_data),
		.wr_size(),
		.wr_full(tx_fifo_full),
		.wr_overflow(),
		.wr_reset(wr_reset),

		.rd_clk(tx_clk),
		.rd_en(txfifo_rd_en),
		.rd_data(tx_bus.data[7:0]),
		.rd_size(),
		.rd_empty(),
		.rd_underflow(),
		.rd_reset(rd_reset)
	);

	logic		txheader_rd_en				= 0;
	wire[10:0]	txheader_rd_data;
	wire		txheader_rd_empty;

	CrossClockFifo #(
		.WIDTH(11),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) tx_framelen_fifo (
		.wr_clk(apb.pclk),
		.wr_en(wr_commit),
		.wr_data(tx_wr_packetlen),
		.wr_size(),
		.wr_full(tx_header_full),
		.wr_overflow(),
		.wr_reset(wr_reset),

		.rd_clk(tx_clk),
		.rd_en(txheader_rd_en),
		.rd_data(txheader_rd_data),
		.rd_size(),
		.rd_empty(txheader_rd_empty),
		.rd_underflow(),
		.rd_reset(rd_reset)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pop logic

	enum logic[1:0]
	{
		TX_STATE_IDLE 		= 0,
		TX_STATE_POP		= 1,
		TX_STATE_SENDING	= 2
	} tx_state = TX_STATE_IDLE;

	logic[10:0] tx_count = 0;
	always_ff @(posedge tx_clk) begin

		tx_bus.start		<= 0;
		tx_bus.data_valid	<= txfifo_rd_en;
		txheader_rd_en		<= 0;
		txfifo_rd_en		<= 0;

		case(tx_state)

			TX_STATE_IDLE: begin

				if(!txheader_rd_empty && tx_ready && !txheader_rd_en) begin
					txheader_rd_en	<= 1;
					tx_state		<= TX_STATE_POP;
				end

			end

			TX_STATE_POP: begin
				tx_bus.start	<= 1;
				tx_count		<= 1;
				txfifo_rd_en	<= 1;
				tx_state		<= TX_STATE_SENDING;
			end

			TX_STATE_SENDING: begin

				if(tx_count >= txheader_rd_data) begin
					tx_state		<= TX_STATE_IDLE;
				end
				else begin
					txfifo_rd_en	<= 1;
					tx_count		<= tx_count + 1;
				end

			end

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA
/*
	ila_0 ila(
		.clk(apb.pclk),
		.probe0(apb.penable),
		.probe1(apb.psel),
		.probe2(apb.pready),
		.probe3(apb.paddr),
		.probe4(apb.pwrite),
		.probe5(apb.pwdata),
		.probe6(apb.pstrb),
		.probe7(tx_wr_packetlen),
		.probe8(wr_en),
		.probe9(wr_data),
		.probe10(wr_commit),
		.probe11(pending_bytes),
		.probe12(pending_bytes_valid)
	);*/

endmodule
