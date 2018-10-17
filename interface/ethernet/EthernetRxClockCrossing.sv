`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
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
	@brief Clock domain crossing FIFO for Ethernet MAC data

	Sits between a TriSpeedEthernetMAC or XGEthernetMAC and the core of the design. Shifts MAC data into the main
	system clock domain.
 */
module EthernetRxClockCrossing(

	//Incoming frames from the MAC
	input wire			gmii_rxc,
	input wire			rx_frame_start,
	input wire			rx_frame_data_valid,
	input wire[2:0]		rx_frame_bytes_valid,
	input wire[31:0]	rx_frame_data,
	input wire			rx_frame_commit,
	input wire			rx_frame_drop,

	//Outgoing frames to the L2 decoder
	input wire			sys_clk,
	output logic		rx_cdc_frame_start			= 0,
	output logic		rx_cdc_frame_data_valid		= 0,
	output logic[2:0]	rx_cdc_frame_bytes_valid	= 0,
	output logic[31:0]	rx_cdc_frame_data			= 0,
	output logic		rx_cdc_frame_commit			= 0
	);

	logic		rxfifo_wr_en		= 0;
	logic[34:0]	rxfifo_wr_data		= 0;

	logic		rxfifo_rd_en			= 0;
	logic		rxfifo_rd_pop_single	= 0;
	wire[2:0]	rxfifo_rd_bytes_valid;
	wire[31:0]	rxfifo_rd_data;
	wire[10:0]	rxfifo_rd_size;

	//An all-zero word indicates packet boundaries
	CrossClockPacketFifo #(
		.WIDTH(35),		//3 bytes valid + 32 data
		.DEPTH(1024)	//at least 2 packets worth
	) rx_cdc_fifo (
		.wr_clk(gmii_rxc),
		.wr_en(rxfifo_wr_en),
		.wr_data(rxfifo_wr_data),
		.wr_reset(1'b0),
		.wr_size(),
		.wr_commit(rx_frame_commit),
		.wr_rollback(rx_frame_drop),

		.rd_clk(sys_clk),
		.rd_en(rxfifo_rd_en),
		.rd_offset(10'h0),
		.rd_pop_single(rxfifo_rd_pop_single),
		.rd_pop_packet(1'b0),
		.rd_packet_size(10'h0),
		.rd_data( {rxfifo_rd_bytes_valid, rxfifo_rd_data} ),
		.rd_size(rxfifo_rd_size),
		.rd_reset(1'b0)
	);

	//PUSH SIDE
	always_comb begin

		//Frame delimiter
		if(rx_frame_start) begin
			rxfifo_wr_en	<= 1;
			rxfifo_wr_data	<= 35'h0;
		end

		//Nope, push data as needed
		else begin
			rxfifo_wr_en	<= rx_frame_data_valid;
			rxfifo_wr_data	<= { rx_frame_bytes_valid, rx_frame_data };
		end

	end

	//POP SIDE
	enum logic[3:0]
	{
		RXFIFO_STATE_IDLE				= 4'h0,
		RXFIFO_STATE_WAIT_FOR_HEADER_0	= 4'h1,
		RXFIFO_STATE_WAIT_FOR_HEADER_1	= 4'h2,
		RXFIFO_STATE_WAIT_FOR_HEADER_2	= 4'h3,
		RXFIFO_STATE_PACKET_0			= 4'h4,
		RXFIFO_STATE_PACKET_1			= 4'h5,
		RXFIFO_STATE_PACKET_2			= 4'h6
	} rxfifo_pop_state = RXFIFO_STATE_WAIT_FOR_HEADER_0;

	always_ff @(posedge sys_clk) begin
		rxfifo_rd_en				<= 0;
		rxfifo_rd_pop_single		<= 0;
		rx_cdc_frame_start			<= 0;
		rx_cdc_frame_data_valid		<= 0;
		rx_cdc_frame_bytes_valid	<= 0;
		rx_cdc_frame_commit			<= 0;

		case(rxfifo_pop_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// First packet after reset - wait for a header

			RXFIFO_STATE_WAIT_FOR_HEADER_0: begin

				//Data in the fifo! Go read a word.
				if(rxfifo_rd_size != 0) begin
					rxfifo_rd_en		<= 1;
					rxfifo_pop_state	<= RXFIFO_STATE_WAIT_FOR_HEADER_1;
				end

			end	//end RXFIFO_STATE_WAIT_FOR_HEADER_0

			RXFIFO_STATE_WAIT_FOR_HEADER_1: begin
				rxfifo_pop_state	<= RXFIFO_STATE_WAIT_FOR_HEADER_2;
			end	//end RXFIFO_STATE_WAIT_FOR_HEADER_1

			RXFIFO_STATE_WAIT_FOR_HEADER_2: begin

				rxfifo_rd_pop_single	<= 1;

				//FIFO data available. Should be all zeroes. Ignore anything else.
				if( (rxfifo_rd_bytes_valid == 0) )
					rxfifo_pop_state		<= RXFIFO_STATE_IDLE;

				//Something nonsensical, ignore it
				else
					rxfifo_pop_state		<= RXFIFO_STATE_WAIT_FOR_HEADER_0;

			end	//end RXFIFO_STATE_WAIT_FOR_HEADER_2

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Sit around and wait for new stuff to arrive

			RXFIFO_STATE_IDLE: begin

				//If there's anything in the FIFO, there's an entire packet ready for us to handle.
				//Kick off the RX decoder.
				if( (rxfifo_rd_size != 0) && !rxfifo_rd_pop_single) begin
					rx_cdc_frame_start		<= 1;
					rxfifo_rd_en			<= 1;
					rxfifo_rd_pop_single	<= 1;
					rxfifo_pop_state		<= RXFIFO_STATE_PACKET_0;
				end

			end	//end RXFIFO_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Crunch packet bodies

			//Request the read
			//Pop at 1 word per clock regardless of push rate
			RXFIFO_STATE_PACKET_0: begin

				//First word is en route. Pop the second
				if(rxfifo_rd_size > 1) begin
					rxfifo_rd_en			<= 1;
					rxfifo_rd_pop_single	<= 1;
					rxfifo_pop_state		<= RXFIFO_STATE_PACKET_1;
				end

			end	//end RXFIFO_STATE_PACKET_0

			//Data words are ready, deal with them
			RXFIFO_STATE_PACKET_1: begin

				//If we hit an all-zeroes word we've just popped the inter-frame gap for the next packet.
				//Jump straight to the idle state
				if(rxfifo_rd_bytes_valid == 0) begin
					rx_cdc_frame_commit			<= 1;
					rxfifo_pop_state			<= RXFIFO_STATE_IDLE;
				end

				//If we hit the end of the packet and there's nothing left in the FIFO, commit this one
				else if(rxfifo_rd_size == 0) begin

					//Push the last data word
					rx_cdc_frame_data_valid		<= 1;
					rx_cdc_frame_bytes_valid	<= rxfifo_rd_bytes_valid;
					rx_cdc_frame_data			<= rxfifo_rd_data;

					//Commit it next cycle
					rxfifo_pop_state			<= RXFIFO_STATE_PACKET_2;
				end

				else begin

					//If the FIFO is going to be emptied this clock, don't pop it (there's a pending read already)
					if(rxfifo_rd_size == 1) begin
					end

					//Not empty, pop the next word
					else begin
						rxfifo_rd_en			<= 1;
						rxfifo_rd_pop_single	<= 1;
					end

					//Valid data - forward it to layer 2 and keep going
					rx_cdc_frame_data_valid		<= 1;
					rx_cdc_frame_bytes_valid	<= rxfifo_rd_bytes_valid;
					rx_cdc_frame_data			<= rxfifo_rd_data;

				end


			end	//end RXFIFO_STATE_PACKET_1

			RXFIFO_STATE_PACKET_2: begin
				rx_cdc_frame_commit		<= 1;
				rxfifo_pop_state		<= RXFIFO_STATE_WAIT_FOR_HEADER_0;
			end	//end RXFIFO_STATE_PACKET_2

		endcase

	end

endmodule
