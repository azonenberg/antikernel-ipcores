`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2021 Andrew D. Zonenberg                                                                          *
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
	@brief Converter from EthernetBus to RMII (at fixed 100 Mbps)
 */
module RMIIToEthernetBusBridge(

	//Clocks
	input wire					clk_50mhz,
	input wire					gmii_rxc,

	//Host side data bus
	input wire EthernetRxBus	host_rx_bus,

	//RMII bus
	output wire					rmii_refclk,
	output wire					rmii_rx_en,
	output wire[1:0]			rmii_rxd
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side frame FIFO (RGMII -> RMII) and push logic

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

	wire		rx_fifo_commit			= host_rx_bus.commit && !rx_dropping;
	wire		rx_header_full;

	always_ff @(posedge gmii_rxc) begin

		//Track frame length and make sure we have space for a max sized frame
		if(host_rx_bus.start) begin
			rx_wr_frame_len	<= 0;
			rx_dropping	<= (rx_fifo_wr_size < 375) || rx_header_full;
		end

		if(host_rx_bus.data_valid)
			rx_wr_frame_len	<= rx_wr_frame_len + host_rx_bus.bytes_valid;

	end

	wire[6:0] rx_header_wr_size;

	//Header FIFO (big enough to hold a full rx_header_fifo worth of minimium sized frames)
	CrossClockFifo #(
		.WIDTH(11),
		.DEPTH(64),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) rx_header_fifo (
		.wr_clk(gmii_rxc),
		.wr_en(rx_fifo_commit),
		.wr_data(rx_wr_frame_len),
		.wr_size(rx_header_wr_size),	//debug
		.wr_full(rx_header_full),
		.wr_overflow(),
		.wr_reset(1'b0),

		.rd_clk(clk_50mhz),
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
		.wr_clk(gmii_rxc),
		.wr_en(host_rx_bus.data_valid && !rx_dropping),
		.wr_data(host_rx_bus.data),
		.wr_reset(1'b0),
		.wr_size(rx_fifo_wr_size),
		.wr_commit(rx_fifo_commit),
		.wr_rollback(host_rx_bus.drop),

		.rd_clk(clk_50mhz),
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
	// RX side FIFO pop logic

	enum logic[3:0]
	{
		RX_STATE_IDLE,
		RX_STATE_HEADER_WAIT,
		RX_STATE_DATA_WAIT,
		RX_STATE_DATA_FIRST,
		RX_STATE_DATA_COUNT
	} rx_state = RX_STATE_IDLE;

	logic[3:0] rx_count			= 0;
	logic[9:0] rx_bytes_read	= 0;

	wire mac_tx_ready;
	EthernetTxBus mac_tx_bus_100m;

	always_ff @(posedge clk_50mhz) begin

		rx_header_rd_en				<= 0;
		rx_fifo_rd_en				<= 0;
		rx_fifo_rd_pop_single		<= 0;
		rx_fifo_rd_pop_packet		<= 0;

		mac_tx_bus_100m.start		<= 0;
		mac_tx_bus_100m.data_valid	<= 0;
		mac_tx_bus_100m.bytes_valid	<= 0;

		case(rx_state)

			//Wait for a header to be ready to pop, and the MAC to be ready for a new frame
			RX_STATE_IDLE: begin

				if(!rx_header_rd_empty && mac_tx_ready) begin
					rx_bytes_read	<= 0;
					rx_header_rd_en	<= 1;
					rx_state		<= RX_STATE_HEADER_WAIT;
				end

			end	//end RX_STATE_IDLE

			//Wait for header to pop, start reading the first data word to save time
			//(we know there's SOMETHING there if we have a header)
			//(The single cycle delay is to cover potential variation in FIFO CDC propagation)
			RX_STATE_HEADER_WAIT: begin
				rx_fifo_rd_en			<= 1;
				rx_fifo_rd_offset		<= 0;
				rx_state				<= RX_STATE_DATA_WAIT;
				mac_tx_bus_100m.start	<= 1;
			end	//end RX_STATE_HEADER_WAIT

			//Wait for first data word
			RX_STATE_DATA_WAIT: begin
				rx_state				<= RX_STATE_DATA_FIRST;
				rx_count				<= 0;
			end	//end RX_STATE_DATA_WAIT

			//Data is ready
			RX_STATE_DATA_FIRST: begin
				mac_tx_bus_100m.data_valid	<= 1;
				rx_count					<= rx_count + 1;

				if( (rx_header_rd_data - rx_bytes_read) > 4)
					mac_tx_bus_100m.bytes_valid	<= 4;
				else
					mac_tx_bus_100m.bytes_valid	<= rx_header_rd_data - rx_bytes_read;

				mac_tx_bus_100m.data		<= rx_fifo_rd_data;
				rx_state					<= RX_STATE_DATA_COUNT;
			end	//end RX_STATE_DATA_FIRST

			//Waiting for MAC to process the data
			RX_STATE_DATA_COUNT: begin
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

						rx_state				<= RX_STATE_IDLE;
					end

					else begin
						rx_state			<= RX_STATE_DATA_FIRST;
						rx_bytes_read		<= rx_bytes_read + 4;
					end
				end
			end	//end RX_STATE_DATA_COUNT

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write logic

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The MAC

	/*
		Invert the transmitted reference clock since we drive data on the positive edge

		Inversion leads to data toggling on the falling edge, providing close to 10ns of setup and hold margin for
		the MAC on the MCU. RMII v1.2 spec requires 4ns setup / 2ns hold which should be trivial.
	 */
	assign rmii_refclk = !clk_50mhz;

	SimpleRMIIMAC mac(
		.clk_50mhz(clk_50mhz),

		.mac_tx_bus(mac_tx_bus_100m),
		.mac_tx_ready(mac_tx_ready),

		.rmii_rx_en(rmii_rx_en),
		.rmii_rxd(rmii_rxd)
	);

endmodule
