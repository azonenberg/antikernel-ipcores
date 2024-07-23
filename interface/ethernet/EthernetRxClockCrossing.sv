`timescale 1ns / 1ps
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
	@brief Clock domain crossing FIFO for Ethernet MAC data

	Sits between a TriSpeedEthernetMAC or XGEthernetMAC and the core of the design. Shifts MAC data into the main
	system clock domain.
 */
module EthernetRxClockCrossing(

	//Incoming frames from the MAC
	input wire					gmii_rxc,
	input wire EthernetRxBus	mac_rx_bus,

	//Outgoing frames to the L2 decoder
	input wire					sys_clk,
	output EthernetRxBus		cdc_rx_bus = {$bits(EthernetRxBus){1'b0}},

	//Performance counters
	output logic[63:0]			perf_rx_cdc_frames			= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The FIFO

	logic		rxfifo_wr_en		= 0;
	logic[34:0]	rxfifo_wr_data		= 0;

	logic		rxfifo_rd_en			= 0;
	logic		rxfifo_rd_pop_packet	= 0;
	wire[10:0]	rxfifo_rd_size;
	logic[10:0]	rxfifo_rd_offset		= 0;
	logic[10:0]	rxfifo_rd_packet_size	= 0;

	logic[2:0]	rxfifo_rd_bytes_valid;
	logic[31:0]	rxfifo_rd_data;

	//An all-zero word indicates packet boundaries
	CrossClockPacketFifo #(
		.WIDTH(35),		//3 bits valid + 32 data
		.DEPTH(1024),	//at least 2 packets worth
		.USE_BLOCK(1),
		.OUT_REG(2)
	) rx_cdc_fifo (
		.wr_clk(gmii_rxc),
		.wr_en(rxfifo_wr_en),
		.wr_data(rxfifo_wr_data),
		.wr_reset(1'b0),
		.wr_size(),
		.wr_commit(mac_rx_bus.commit),
		.wr_rollback(mac_rx_bus.drop),

		.rd_clk(sys_clk),
		.rd_en(rxfifo_rd_en),
		.rd_offset(rxfifo_rd_offset),
		.rd_pop_single(1'b0),
		.rd_pop_packet(rxfifo_rd_pop_packet),
		.rd_packet_size(rxfifo_rd_packet_size),
		.rd_data( {rxfifo_rd_bytes_valid, rxfifo_rd_data} ),
		.rd_size(rxfifo_rd_size),
		.rd_reset(1'b0)
	);

	//PUSH SIDE
	always_comb begin

		//Frame delimiter
		if(mac_rx_bus.start) begin
			rxfifo_wr_en	<= 1;
			rxfifo_wr_data	<= 35'h0;
		end

		//Nope, push data as needed
		else begin
			rxfifo_wr_en	<= mac_rx_bus.data_valid;
			rxfifo_wr_data	<= { mac_rx_bus.bytes_valid, mac_rx_bus.data };
		end

	end

	//POP SIDE
	enum logic[3:0]
	{
		RXFIFO_STATE_IDLE				= 4'h0,
		RXFIFO_STATE_WAIT_FOR_HEADER_0	= 4'h1,
		RXFIFO_STATE_WAIT_FOR_HEADER_1	= 4'h2,
		RXFIFO_STATE_WAIT_FOR_HEADER_2	= 4'h3,
		RXFIFO_STATE_WAIT_FOR_HEADER_3	= 4'h4,
		RXFIFO_STATE_PACKET_0			= 4'h5,
		RXFIFO_STATE_PACKET_1			= 4'h6,
		RXFIFO_STATE_PACKET_2			= 4'h7,
		RXFIFO_STATE_PACKET_3			= 4'h8
	} rxfifo_pop_state = RXFIFO_STATE_WAIT_FOR_HEADER_0;

	//True if we're idle but have a valid word from the previous packet
	logic	last_word_valid = 0;

	logic	data_valid_adv	= 0;

	logic	rxfifo_has_packet	= 0;

	always_ff @(posedge sys_clk) begin
		rxfifo_rd_en				<= 0;
		rxfifo_rd_pop_packet		<= 0;
		cdc_rx_bus.start			<= 0;
		cdc_rx_bus.data_valid		<= 0;
		cdc_rx_bus.commit			<= 0;

		//Push data down the pipeline
		cdc_rx_bus.bytes_valid		<= rxfifo_rd_bytes_valid;
		cdc_rx_bus.data				<= rxfifo_rd_data;
		cdc_rx_bus.data_valid		<= (rxfifo_pop_state == RXFIFO_STATE_PACKET_2) && (rxfifo_rd_bytes_valid != 0);

		rxfifo_has_packet			<= (rxfifo_rd_size > 2);

		case(rxfifo_pop_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// First packet after reset - wait for a header

			RXFIFO_STATE_WAIT_FOR_HEADER_0: begin

				cdc_rx_bus.data_valid	<= 0;

				//Wait for pop to complete before doing anything
				if(rxfifo_rd_pop_packet) begin
				end

				//Data in the fifo! Go read a word.
				else if(rxfifo_has_packet) begin
					rxfifo_rd_en		<= 1;
					rxfifo_rd_offset	<= 0;
					rxfifo_pop_state	<= RXFIFO_STATE_WAIT_FOR_HEADER_1;
				end

			end	//end RXFIFO_STATE_WAIT_FOR_HEADER_0

			RXFIFO_STATE_WAIT_FOR_HEADER_1: begin
				rxfifo_pop_state	<= RXFIFO_STATE_WAIT_FOR_HEADER_2;
			end	//end RXFIFO_STATE_WAIT_FOR_HEADER_1

			RXFIFO_STATE_WAIT_FOR_HEADER_2: begin
				rxfifo_pop_state	<= RXFIFO_STATE_WAIT_FOR_HEADER_3;
			end

			RXFIFO_STATE_WAIT_FOR_HEADER_3: begin

				//FIFO data available. Should be all zeroes (inter-frame gap).
				if( (rxfifo_rd_bytes_valid == 0) )
					rxfifo_pop_state		<= RXFIFO_STATE_IDLE;

				//Something nonsensical, throw it out
				else begin
					rxfifo_pop_state		<= RXFIFO_STATE_WAIT_FOR_HEADER_0;
					rxfifo_rd_pop_packet	<= 1;
					rxfifo_rd_packet_size	<= 1;
				end

			end	//end RXFIFO_STATE_WAIT_FOR_HEADER_3

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Sit around and wait for new stuff to arrive

			RXFIFO_STATE_IDLE: begin

				//If there's anything in the FIFO, there's an entire packet ready for us to handle.
				//Kick off the RX decoder.
				if(rxfifo_has_packet && !rxfifo_rd_pop_packet) begin
					cdc_rx_bus.start		<= 1;
					rxfifo_rd_offset		<= 1;

					//If we didn't get a valid word at the end of the previous packet, read now
					if(!last_word_valid) begin
						rxfifo_rd_en		<= 1;
					end
					rxfifo_pop_state		<= RXFIFO_STATE_PACKET_0;
				end

			end	//end RXFIFO_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Crunch packet bodies

			//Request the read
			//Pop at 1 word per clock regardless of push rate
			RXFIFO_STATE_PACKET_0: begin
				rxfifo_rd_en		<= 1;
				rxfifo_rd_offset	<= rxfifo_rd_offset + 1;
				rxfifo_pop_state	<= RXFIFO_STATE_PACKET_1;
			end	//end RXFIFO_STATE_PACKET_0

			//Read second word of packet (packet must be at least 2 words long anyway)
			RXFIFO_STATE_PACKET_1: begin

				//First word is en route. Pop the second
				rxfifo_rd_en			<= 1;
				rxfifo_rd_offset		<= rxfifo_rd_offset + 1;
				rxfifo_pop_state		<= RXFIFO_STATE_PACKET_2;
			end	//end RXFIFO_STATE_PACKET_1

			//Data words are ready, deal with them
			RXFIFO_STATE_PACKET_2: begin

				//If we hit a value other than 4 (i.e. partial word) this is the end of the packet
				if(rxfifo_rd_bytes_valid != 4)
					rxfifo_pop_state		<= RXFIFO_STATE_PACKET_3;

				//If we hit the end of the packet and there's nothing left in the FIFO, commit this one
				else if(rxfifo_rd_size == 0)
					rxfifo_pop_state		<= RXFIFO_STATE_PACKET_3;

				//Nope, normal operation
				else begin
					rxfifo_rd_en			<= 1;
					rxfifo_rd_offset		<= rxfifo_rd_offset + 1;

					//Valid data - forward it to layer 2 and keep going
					cdc_rx_bus.data_valid	<= 1;

				end

			end	//end RXFIFO_STATE_PACKET_2

			RXFIFO_STATE_PACKET_3: begin
				rxfifo_rd_pop_packet	<= 1;

				//Bounds check: shouldn't happen but make sure we don't run into negative size territory
				if(rxfifo_rd_offset >= rxfifo_rd_size)
					rxfifo_rd_packet_size	<= rxfifo_rd_size;
				else
					rxfifo_rd_packet_size	<= rxfifo_rd_offset;

				cdc_rx_bus.commit		<= 1;
				perf_rx_cdc_frames		<= perf_rx_cdc_frames + 1'h1;
				rxfifo_pop_state		<= RXFIFO_STATE_WAIT_FOR_HEADER_0;
			end	//end RXFIFO_STATE_PACKET_3

		endcase

	end

endmodule
