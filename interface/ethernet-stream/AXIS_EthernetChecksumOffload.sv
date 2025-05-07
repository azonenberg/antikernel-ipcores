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
	@brief Ethernet checksum offload
 */
module AXIS_EthernetChecksumOffload(

	//Global state
	input wire				link_up_aclk,

	//TX/RX buffer interfaces
	AXIStream.receiver		buf_tx,
	AXIStream.transmitter	mac_tx
);

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Forwarding FIFO

	logic		fifo_rd	= 0;
	wire[7:0]	fifo_rd_data;

	wire		fifo_empty;
	wire		fifo_full;
	wire[12:0]	fifo_wsize;

	SingleClockFifo #(
		.WIDTH(8),
		.DEPTH(4096)
	) fwd_fifo (
		.clk(clk),

		.wr(buf_tx_bus.data_valid),
		.din(buf_tx_bus.data[7:0]),

		.rd(fifo_rd),
		.dout(fifo_rd_data),

		.overflow(),
		.underflow(),
		.empty(fifo_empty),
		.full(fifo_full),
		.wsize(fifo_wsize),
		.rsize(),
		.reset(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Metadata FIFO

	logic			meta_wr_en		= 0;
	logic[10:0]		meta_wr_len		= 0;
	logic[15:0]		meta_wr_csum	= 0;
	logic[10:0]		meta_wr_offset	= 0;

	wire			meta_empty;
	logic			meta_rd_en		= 0;
	wire[10:0]		meta_rd_len;
	wire[15:0]		meta_rd_csum;
	wire[10:0]		meta_rd_offset;

	wire			meta_fifo_full;

	SingleClockFifo #(
		.WIDTH(38),
		.DEPTH(32)
	) meta_fifo (
		.clk(clk),

		.wr(meta_wr_en),
		.din({meta_wr_len, meta_wr_csum, meta_wr_offset}),

		.rd(meta_rd_en),
		.dout({meta_rd_len, meta_rd_csum, meta_rd_offset}),

		.overflow(),
		.underflow(),
		.empty(meta_empty),
		.full(meta_fifo_full),
		.wsize(),
		.rsize(),
		.reset(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO readback

	enum logic[3:0]
	{
		RD_STATE_IDLE,
		RD_STATE_POP,
		RD_STATE_FORWARD,
		RD_STATE_LAST
	} rd_state = RD_STATE_IDLE;

	logic[10:0]	rd_count	= 0;

	always_ff @(posedge clk) begin

		meta_rd_en	<= 0;
		fifo_rd		<= 0;

		mac_tx_bus.start		<= 0;
		mac_tx_bus.data_valid	<= 0;

		case(rd_state)

			//Forward stuff
			RD_STATE_IDLE: begin
				if(mac_tx_ready && !meta_empty) begin

					meta_rd_en			<= 1;
					fifo_rd				<= 1;
					mac_tx_bus.start	<= 1;
					rd_state			<= RD_STATE_POP;
					rd_count			<= 0;

				end
			end

			RD_STATE_POP: begin
				fifo_rd					<= 1;
				rd_state				<= RD_STATE_FORWARD;
				rd_count				<= rd_count + 1;
			end

			RD_STATE_FORWARD: begin
				rd_count				<= rd_count + 1;

				//End of packet?
				if( (rd_count + 1) >= meta_rd_len ) begin
					rd_state			<= RD_STATE_LAST;
				end
				else
					fifo_rd				<= 1;

				//Forward data
				mac_tx_bus.data_valid	<= 1;
				mac_tx_bus.data			<= {24'h0, fifo_rd_data};

				//Inject checksum (if we have it)
				//For now only support one checksum per packet (i.e. TCP or UDP layer)
				if(meta_rd_offset != 0) begin
					if(meta_rd_offset == rd_count)
						mac_tx_bus.data	<= meta_rd_csum[15:8];
					else if((meta_rd_offset + 1) == rd_count )
						mac_tx_bus.data	<= meta_rd_csum[7:0];
				end

			end

			RD_STATE_LAST: begin
				mac_tx_bus.data_valid	<= 1;
				mac_tx_bus.data			<= {24'h0, fifo_rd_data};
				rd_state				<= RD_STATE_IDLE;
			end

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Checksum calculation

	logic		csum_process	= 0;
	logic[31:0]	csum_din		= 0;
	wire[15:0]	csum_sum;
	wire[15:0]	csum_sum_raw;

	InternetChecksum32bit csum_calc(
		.clk(clk),

		.load(1'b0),
		.reset(buf_tx_bus.start),
		.process(csum_process),
		.din(csum_din),
		.sumout(csum_sum_raw),
		.csumout()
	);
	assign csum_sum = ~csum_sum_raw;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parse frame headers to figure out what type it is

	enum logic[3:0]
	{
		STATE_IDLE,
		STATE_MAC_HEADER,
		STATE_IPV4_HEADER,
		STATE_TCP_V4_PACKET,
		STATE_UDP_V4_PACKET,
		STATE_LEN,
		STATE_SAVE_CSUM,
		STATE_UNKNOWN,
		STATE_IGNORE
	} state = STATE_IDLE;

	logic[10:0]	count				= 0;
	logic[15:0] ethertype			= 0;
	logic[7:0]	ip_proto			= 0;

	always_ff @(posedge clk) begin

		//Default to being ready to accept new packets
		buf_tx_ready	<= 1;

		//If we're still in the tail end of processing a packet, don't accept a new one
		if( (state != STATE_UNKNOWN) && (state != STATE_IGNORE) && (state != STATE_IDLE) )
			buf_tx_ready	<= 0;

		//Backpressure if we're not ready to accept any more data
		if( (fifo_wsize < 1500) || meta_fifo_full)
			buf_tx_ready	<= 0;

		//Clear single cycle flags
		csum_process	<= 0;
		meta_wr_en		<= 0;

		//Process payload data
		if(buf_tx_bus.data_valid) begin

			//frame is one byte longer
			meta_wr_len	<= meta_wr_len + 1;

			case(state)

				//wait for frame to start
				STATE_IDLE: begin
				end

				//reading 802.3 header
				STATE_MAC_HEADER: begin

					count	<= count + 1;

					//Look for ethertype
					if(count == 12)
						ethertype[15:8]	= buf_tx_bus.data;
					if(count == 13) begin
						ethertype[7:0]	= buf_tx_bus.data;

						//It's an IPv4 packet
						if(ethertype == 16'h0800) begin
							state	<= STATE_IPV4_HEADER;
							count	<= 0;
						end

						//not something we know how to offload, pass through unmodified
						else
							state	<= STATE_UNKNOWN;

					end
				end	//STATE_MAC_HEADER

				//reading IPv4 header
				STATE_IPV4_HEADER: begin
					count	<= count + 1;

					case(count)

						//Look for upper layer protocol
						9: begin
							ip_proto	<= buf_tx_bus.data;

							//Checksum the IP protocol number
							csum_din	<= {23'h0, buf_tx_bus.data};
							csum_process	<= 1;
						end

						//IPv4 source address
						12:	csum_din[31:24]	<= buf_tx_bus.data;
						13:	csum_din[23:16]	<= buf_tx_bus.data;
						14:	csum_din[15:8]	<= buf_tx_bus.data;
						15: begin
							csum_din[7:0]	<= buf_tx_bus.data;
							csum_process	<= 1;
						end

						//IPv4 dest address
						16:	csum_din[31:24]	<= buf_tx_bus.data;
						17:	csum_din[23:16]	<= buf_tx_bus.data;
						18:	csum_din[15:8]	<= buf_tx_bus.data;

						//TODO: support IP header options
						//For now, assume sender isn't sending any options
						19: begin

							//Checksum the end of the address
							csum_din[7:0]	<= buf_tx_bus.data;
							csum_process	<= 1;

							count		<= 0;

							if(ip_proto == 8'h6)
								state	<= STATE_TCP_V4_PACKET;
							else if(ip_proto == 8'h11)
								state	<= STATE_UDP_V4_PACKET;
							else
								state	<= STATE_UNKNOWN;
						end

					endcase

				end	//STATE_IPV4_HEADER

				STATE_TCP_V4_PACKET: begin

					count	<= count + 1;

					//Append data to the checksum
					case(count[1:0])
						0:	csum_din[31:24]	<= buf_tx_bus.data;
						1:	csum_din[23:16]	<= buf_tx_bus.data;
						2:	csum_din[15:8]	<= buf_tx_bus.data;
						3: begin
							csum_din[7:0]	<= buf_tx_bus.data;
							csum_process	<= 1;
						end
					endcase

					//If this is bytes 16 or 17, it's the input checksum
					//Save it, but don't checksum whatever garbage the stack put there
					if(count == 16) begin
						meta_wr_offset		<= (meta_wr_len + 1'h1);	//save position for eventual checksum insertion
						csum_din[31:24]		<= 0;
					end
					if(count == 17)
						csum_din[23:16]		<= 0;

				end

				STATE_UDP_V4_PACKET: begin
					count	<= count + 1;

					//Append data to the checksum
					case(count[1:0])
						0:	csum_din[31:24]	<= buf_tx_bus.data;
						1:	csum_din[23:16]	<= buf_tx_bus.data;
						2:	csum_din[15:8]	<= buf_tx_bus.data;
						3: begin
							csum_din[7:0]	<= buf_tx_bus.data;
							csum_process	<= 1;
						end
					endcase

					//If this is bytes 6 or 7, it's the input checksum
					//Save it, but don't checksum whatever garbage the stack put there
					if(count == 6) begin
						meta_wr_offset		<= (meta_wr_len + 1'h1);	//save position for eventual checksum insertion
						csum_din[15:8]		<= 0;
					end
					if(count == 7)
						csum_din[7:0]		<= 0;

				end	//STATE_UDP_V4_PACKET

			endcase

		end

		//End of a packet?
		else begin

			case(state)

				//Checksum the last partial word
				STATE_TCP_V4_PACKET: begin

					//Trim off any incomplete trailing bytes
					case(count[1:0])
						0:	csum_din[31:0]	<= 0;
						1:	csum_din[23:0]	<= 0;
						2:	csum_din[15:0]	<= 0;
						3:	csum_din[7:0]	<= 0;
					endcase

					csum_process	<= 1;
					state			<= STATE_LEN;
				end

				//Checksum the last partial word
				STATE_UDP_V4_PACKET: begin

					//Trim off any incomplete trailing bytes
					case(count[1:0])
						0:	csum_din[31:0]	<= 0;
						1:	csum_din[23:0]	<= 0;
						2:	csum_din[15:0]	<= 0;
						3:	csum_din[7:0]	<= 0;
					endcase

					csum_process	<= 1;
					state			<= STATE_LEN;
				end

				//Append the final packet length
				STATE_LEN: begin

					//Checksum the packet length
					csum_din		<= count;
					csum_process	<= 1;
					state			<= STATE_SAVE_CSUM;
				end

				STATE_SAVE_CSUM: begin
					if(!csum_process) begin

						//Save the checksum
						meta_wr_en		<= 1;
						meta_wr_csum	<= csum_sum;

						state			<= STATE_IGNORE;
					end
				end

				STATE_UNKNOWN: begin
					meta_wr_en		<= 1;
					meta_wr_csum	<= 0;
					meta_wr_offset	<= 0;
					state			<= STATE_IGNORE;
				end

			endcase

		end

		if(buf_tx_bus.start) begin
			state		<= STATE_MAC_HEADER;
			count		<= 0;
			meta_wr_len	<= 0;
		end

	end
	*/

endmodule
