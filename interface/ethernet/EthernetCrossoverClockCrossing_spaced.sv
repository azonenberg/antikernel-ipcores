`timescale 1ns / 1ps
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

`include "EthernetBus.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief	Converts an EthernetRxBus in one clock domain to an EthernetTxBus in another.

	Space is added between the data bytes.
 */
module EthernetCrossoverClockCrossing_spaced(

	input wire					rx_clk,
	input wire EthernetRxBus	rx_bus,

	input wire					tx_clk,
	input wire					tx_ready,
	output EthernetTxBus		tx_bus
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side frame FIFO and push logic

	logic		rx_fifo_rd_en			= 0;
	logic[9:0]	rx_fifo_rd_offset		= 0;
	logic		rx_fifo_rd_pop_single	= 0;
	logic		rx_fifo_rd_pop_packet	= 0;

	logic		rx_header_rd_en			= 0;
	wire[10:0]	rx_header_rd_data;
	wire		rx_header_rd_empty;
	wire[31:0]	rx_fifo_rd_data;

	logic		rx_dropping				= 0;
	logic[10:0]	rx_wr_frame_len			= 0;
	wire[10:0]	rx_fifo_wr_size;
	logic[9:0]	rx_fifo_rd_packet_size	= 0;

	wire		rx_fifo_commit			= rx_bus.commit && !rx_dropping;
	wire		rx_header_full;

	always_ff @(posedge rx_clk) begin

		//Track frame length and make sure we have space for a max sized frame
		if(rx_bus.start) begin
			rx_wr_frame_len	<= 0;
			rx_dropping	<= (rx_fifo_wr_size < 375) || rx_header_full;
		end

		if(rx_bus.data_valid)
			rx_wr_frame_len	<= rx_wr_frame_len + rx_bus.bytes_valid;

	end

	wire[6:0] rx_header_wr_size;

	//Header FIFO (big enough to hold a full rx_header_fifo worth of minimum sized frames)
	CrossClockFifo #(
		.WIDTH(11),
		.DEPTH(64),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) rx_header_fifo (
		.wr_clk(rx_clk),
		.wr_en(rx_fifo_commit),
		.wr_data(rx_wr_frame_len),
		.wr_size(rx_header_wr_size),	//debug
		.wr_full(rx_header_full),
		.wr_overflow(),
		.wr_reset(1'b0),

		.rd_clk(tx_clk),
		.rd_en(rx_header_rd_en),
		.rd_data(rx_header_rd_data),
		.rd_size(),
		.rd_empty(rx_header_rd_empty),
		.rd_underflow(),
		.rd_reset(1'b0)
	);

	//32 bits * 1024 rows = 4 kB.
	//We want to be able to hold a couple of frames since the 100mbit RMII interface is a lot slower than 1000base-T.
	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(1024)
	) rx_data_fifo (
		.wr_clk(rx_clk),
		.wr_en(rx_bus.data_valid && !rx_dropping),
		.wr_data(rx_bus.data),
		.wr_reset(1'b0),
		.wr_size(rx_fifo_wr_size),
		.wr_commit(rx_fifo_commit),
		.wr_rollback(rx_bus.drop),

		.rd_clk(tx_clk),
		.rd_en(rx_fifo_rd_en),
		.rd_offset(rx_fifo_rd_offset),
		.rd_pop_single(rx_fifo_rd_pop_single),
		.rd_pop_packet(rx_fifo_rd_pop_packet),
		.rd_data(rx_fifo_rd_data),
		.rd_packet_size(rx_fifo_rd_packet_size),
		.rd_size(),
		.rd_reset()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO pop logic

	enum logic[3:0]
	{
		STATE_IDLE,
		STATE_HEADER_WAIT,
		STATE_DATA_WAIT,
		STATE_DATA_FIRST,
		STATE_DATA_COUNT
	} rx_state = STATE_IDLE;

	logic[3:0] rx_count			= 0;
	logic[10:0] rx_bytes_read	= 0;

	always_ff @(posedge tx_clk) begin

		rx_header_rd_en				<= 0;
		rx_fifo_rd_en				<= 0;
		rx_fifo_rd_pop_single		<= 0;
		rx_fifo_rd_pop_packet		<= 0;

		tx_bus.start		<= 0;
		tx_bus.data_valid	<= 0;
		tx_bus.bytes_valid	<= 0;

		case(rx_state)

			//Wait for a header to be ready to pop, and the MAC to be ready for a new frame
			STATE_IDLE: begin

				if(!rx_header_rd_empty && tx_ready) begin
					rx_bytes_read	<= 0;
					rx_header_rd_en	<= 1;
					rx_state		<= STATE_HEADER_WAIT;
				end

			end	//end STATE_IDLE

			//Wait for header to pop, start reading the first data word to save time
			//(we know there's SOMETHING there if we have a header)
			//(The single cycle delay is to cover potential variation in FIFO CDC propagation)
			STATE_HEADER_WAIT: begin
				rx_fifo_rd_en			<= 1;
				rx_fifo_rd_offset		<= 0;
				rx_state				<= STATE_DATA_WAIT;
				tx_bus.start	<= 1;
			end	//end STATE_HEADER_WAIT

			//Wait for first data word
			STATE_DATA_WAIT: begin
				rx_state				<= STATE_DATA_FIRST;
				rx_count				<= 0;
			end	//end STATE_DATA_WAIT

			//Data is ready
			STATE_DATA_FIRST: begin
				tx_bus.data_valid		<= 1;
				rx_count				<= rx_count + 1;

				if( (rx_header_rd_data - rx_bytes_read) > 4)
					tx_bus.bytes_valid	<= 4;
				else
					tx_bus.bytes_valid	<= rx_header_rd_data - rx_bytes_read;

				tx_bus.data				<= rx_fifo_rd_data;
				rx_state				<= STATE_DATA_COUNT;
			end	//end STATE_DATA_FIRST

			//Waiting for MAC to process the data
			STATE_DATA_COUNT: begin
				rx_count				<= rx_count + 1;

				if(rx_count == 14) begin
					rx_fifo_rd_en		<= 1;
					rx_fifo_rd_offset	<= rx_fifo_rd_offset + 1;
				end
				if(rx_count == 15) begin

					if( (rx_bytes_read + 4) >= rx_header_rd_data) begin
						rx_fifo_rd_pop_packet		<= 1;

						if(rx_header_rd_data[1:0])
							rx_fifo_rd_packet_size	<= rx_header_rd_data[10:2] + 1;
						else
							rx_fifo_rd_packet_size	<= rx_header_rd_data[10:2];

						rx_state				<= STATE_IDLE;
					end

					else begin
						rx_state			<= STATE_DATA_FIRST;
						rx_bytes_read		<= rx_bytes_read + 4;
					end
				end
			end	//end STATE_DATA_COUNT

		endcase

	end

endmodule
