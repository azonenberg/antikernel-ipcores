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
	@brief Elastic buffer for matching RX and TX clock domains

	Also, build the layer-2 headers (FCS and padding, if any, are added by the MAC)
 */
module EthernetTransmitElasticBuffer(

	input wire[47:0]	our_mac_address,

	//Inbound transmit bus from the protocol stack
	input wire			tx_l2_clk,
	input wire			tx_l2_start,
	input wire			tx_l2_data_valid,
	input wire[2:0]		tx_l2_bytes_valid,
	input wire[31:0]	tx_l2_data,
	input wire			tx_l2_commit,
	input wire			tx_l2_drop,
	input wire[47:0]	tx_l2_dst_mac,
	input wire[15:0]	tx_l2_ethertype,

	//Outbound transmit bus to the MAC
	input wire			xgmii_tx_clk,
	output reg			tx_frame_start			= 0,
	output reg			tx_frame_data_valid		= 0,
	output reg[2:0]		tx_frame_bytes_valid	= 0,
	output reg[31:0]	tx_frame_data			= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration

	`include "../../synth_helpers/clog2.vh"

	parameter PACKET_DEPTH	= 8192;		//Packet-data FIFO is 32 bits wide x this many words
										//Default 8192 = 32768 bytes
										//(21 standard frames, 3 jumbo frames, 512 min-sized frames)

	parameter HEADER_DEPTH	= 512;		//Depth of header FIFO, in packets

	localparam PACKET_BITS	= clog2(PACKET_DEPTH);
	localparam HEADER_BITS	= clog2(HEADER_DEPTH);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Packet data FIFO

	reg						fifo_wr_en			= 0;
	reg[31:0]				fifo_wr_data		= 0;
	reg						fifo_wr_commit		= 0;
	reg						fifo_wr_rollback	= 0;

	wire[PACKET_BITS:0]		fifo_wr_size;

	reg						fifo_rd_en			= 0;
	reg[PACKET_BITS-1:0]	fifo_rd_offset		= 0;
	reg						fifo_pop_packet		= 0;
	reg[PACKET_BITS:0]		fifo_pop_size		= 0;
	wire[31:0]				fifo_rd_data;
	wire[PACKET_BITS:0]		fifo_rd_size;

	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(PACKET_DEPTH)
	) payload_fifo (
		.wr_clk(tx_l2_clk),
		.wr_en(fifo_wr_en),
		.wr_data(fifo_wr_data),
		.wr_reset(1'b0),
		.wr_size(fifo_wr_size),
		.wr_commit(fifo_wr_commit),
		.wr_rollback(fifo_wr_rollback),

		.rd_clk(xgmii_tx_clk),
		.rd_en(fifo_rd_en),
		.rd_offset(fifo_rd_offset),
		.rd_pop_single(1'b0),
		.rd_pop_packet(fifo_pop_packet),
		.rd_packet_size(fifo_pop_size),
		.rd_data(fifo_rd_data),
		.rd_size(fifo_rd_size),
		.rd_reset(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Packet header FIFO

	/*
		77:64				Length (bytes)
		63:48				Ethertype
		47:0				Dest MAC
	 */

	reg						header_wr_en		= 0;
	reg						header_wr_en_ff		= 0;
	reg[13:0]				packet_wr_len		= 0;

	wire[HEADER_BITS:0]		header_wr_size;

	reg						header_pop			= 0;
	reg						header_rd_en		= 0;
	reg						header_rd_en_ff		= 0;

	wire[HEADER_BITS:0]		header_rd_size;
	wire[77:0]				header_rd_data;
	reg[77:0]				header_rd_data_ff	= 0;

	wire[13:0]				header_rd_framelen	= header_rd_data_ff[77:64];
	wire[15:0]				header_rd_ethertype	= header_rd_data_ff[63:48];
	wire[47:0]				header_rd_dstmac	= header_rd_data_ff[47:0];

	CrossClockPacketFifo #(
		.WIDTH(78),
		.DEPTH(HEADER_DEPTH)
	) header_fifo (
		.wr_clk(tx_l2_clk),
		.wr_en(header_wr_en),
		.wr_data( {packet_wr_len, tx_l2_ethertype, tx_l2_dst_mac } ),
		.wr_reset(1'b0),
		.wr_size(header_wr_size),
		.wr_commit(header_wr_en_ff),
		.wr_rollback(1'b0),

		.rd_clk(xgmii_tx_clk),
		.rd_en(header_rd_en),
		.rd_offset(1'b0),
		.rd_pop_single(header_pop),
		.rd_pop_packet(1'b0),
		.rd_packet_size({HEADER_BITS{1'b0}}),
		.rd_data(header_rd_data),
		.rd_size(header_rd_size),
		.rd_reset(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Write control logic

	reg		packet_active	= 0;

	always @(posedge tx_l2_clk) begin

		//Clear single-cycle flags
		fifo_wr_en			<= 0;
		fifo_wr_commit		<= 0;
		fifo_wr_rollback	<= 0;
		header_wr_en		<= 0;

		fifo_wr_data		<= tx_l2_data;

		header_wr_en_ff		<= header_wr_en;

		//Wait for a new packet to start.
		if(!packet_active) begin

			if(tx_l2_start) begin

				packet_wr_len		<= 0;

				//Only accept a new packet if we have room in both buffers
				//We need one slot of header plus one minimum packet worth of space in the payload buffer.
				//Larger packets need more and will be dropped if we run out of space.
				if( (header_wr_size >= 1) && (fifo_wr_size >= 8) )
					packet_active		<= 1;

				//Not enough space to store the packet. Drop it.
				//TODO: performance counters?
				else begin
				end

			end

		end

		//Packet is currently in progress.
		//For now, assume the layer-3 stacks will not attempt to send a >9000 byte packet.
		//TODO: how to recover if this happens?
		else begin

			//If the current packet finishes, push headers and commit the packet data
			if(tx_l2_commit) begin
				fifo_wr_commit		<= 1;
				header_wr_en		<= 1;
				packet_active		<= 0;
			end

			//If the current packet is aborted, discard any in-progress stuff
			else if(tx_l2_drop) begin
				fifo_wr_rollback	<= 1;
				packet_active		<= 0;
			end

			//Write packet data
			else if(tx_l2_data_valid) begin

				//If we're out of buffer space, drop the packet and discard partially written data
				if(fifo_wr_size <= 1) begin
					fifo_wr_rollback	<= 1;
					packet_active		<= 0;
				end

				//We have room for the next word, go push it
				else begin
					packet_wr_len		<= packet_wr_len + tx_l2_bytes_valid;
					fifo_wr_data		<= tx_l2_data;
					fifo_wr_en			<= 1;
				end

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read control logic

	reg			tx_active			= 0;

	reg[13:0]	tx_bytes_left		= 0;
	reg[2:0]	tx_count			= 0;
	reg[31:0]	fifo_rd_data_ff		= 0;
	reg[15:0]	fifo_rd_data_ff2	= 0;

	always @(posedge xgmii_tx_clk) begin

		tx_frame_start			<= 0;
		tx_frame_data_valid		<= 0;
		tx_frame_bytes_valid	<= 0;

		fifo_rd_en				<= 0;
		fifo_pop_packet			<= 0;
		header_pop				<= 0;
		header_rd_en			<= 0;

		fifo_rd_data_ff			<= fifo_rd_data;
		fifo_rd_data_ff2		<= fifo_rd_data_ff[15:0];
		header_rd_en_ff			<= header_rd_en;
		header_rd_data_ff		<= header_rd_data;

		//Wait for new frames to be ready to send
		if(!tx_active) begin

			//If we are currently reading headers, they'll be ready next cycle
			if(header_rd_en_ff) begin
				tx_count		<= 0;
				tx_active		<= 1;
				tx_frame_start	<= 1;

				//Pop the header so we have buffer space for the next packet
				header_pop		<= 1;

			end

			//wait for read
			else if(header_rd_en) begin
				//no action needed, just wait
			end

			//If there's headers and data ready to go, read them.
			else if( (header_rd_size > 0) && (fifo_rd_size > 0) )
				header_rd_en	<= 1;

		end

		//Currently forwarding a frame
		else begin

			case(tx_count)

				//Send first 4 bytes of dest MAC
				0: begin
					tx_frame_data_valid		<= 1;
					tx_frame_bytes_valid	<= 4;
					tx_frame_data			<= header_rd_dstmac[47:16];

					//Request read of the first message data word so it's ready when we need it
					fifo_rd_en				<= 1;
					fifo_rd_offset			<= 0;

					tx_count				<= 1;
				end

				//Send last 2 bytes of dest MAC and first two of source
				1: begin
					tx_frame_data_valid		<= 1;
					tx_frame_bytes_valid	<= 4;
					tx_frame_data			<=
					{
						header_rd_dstmac[15:0],
						our_mac_address[47:32]
					};

					//Request read of the second message data word so it's ready when we need it
					fifo_rd_en				<= 1;
					fifo_rd_offset			<= fifo_rd_offset + 1'h1;

					tx_count				<= 2;
				end

				//Send last 4 bytes of source MAC
				2: begin
					tx_frame_data_valid		<= 1;
					tx_frame_bytes_valid	<= 4;
					tx_frame_data			<= our_mac_address[31:0];

					//Request read of the third message data word so it's ready when we need it
					fifo_rd_en				<= 1;
					fifo_rd_offset			<= fifo_rd_offset + 1'h1;

					tx_count				<= 3;
				end

				//Send ethertype plus first two data bytes
				//TODO: support insertion of 802.1q tags here
				3: begin

					tx_frame_data_valid		<= 1;
					tx_frame_bytes_valid	<= 4;
					tx_frame_data			<= { header_rd_ethertype, fifo_rd_data_ff[31:16] };

					//We sent two bytes but the rest are still coming
					tx_bytes_left			<= header_rd_framelen - 14'd2;

					//Request read of the fourth message data word so it's ready when we need it
					fifo_rd_en				<= 1;
					fifo_rd_offset			<= fifo_rd_offset + 1'h1;

					tx_count				<= 4;

				end

				//Send subsequent data bytes
				4: begin

					tx_frame_data_valid			<= 1;
					tx_frame_data				<= { fifo_rd_data_ff2[15:0], fifo_rd_data_ff[31:16] };

					if(tx_bytes_left < 4) begin
						tx_frame_bytes_valid	<= tx_bytes_left;
						tx_bytes_left			<= 0;

						//Pop the FIFO.
						//Round size up to words
						fifo_pop_packet			<= 1;
						if(header_rd_framelen[1:0])
							fifo_pop_size		<= header_rd_framelen[13:2] + 1'h1;
						else
							fifo_pop_size		<= header_rd_framelen[13:2];

						tx_active				<= 0;
						tx_count				<= 0;
					end

					//Send the next 4 bytes
					else begin

						tx_frame_bytes_valid	<= 4;
						tx_bytes_left			<= tx_bytes_left - 14'h4;

						//Read the next FIFO block if we need more data
						//If we have six bytes or less left, though, stop
						if(tx_bytes_left >= 6) begin
							fifo_rd_en			<= 1;
							fifo_rd_offset		<= fifo_rd_offset + 1'h1;
						end

					end

				end

			endcase

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	wire	trig_out;
	reg		trig_out_ack	= 0;

	/*
	always @(posedge xgmii_tx_clk) begin
		trig_out_ack	<= trig_out;
	end

	ila_0 ila(
		.clk(xgmii_tx_clk),

		.probe0(tx_l2_start),
		.probe0(tx_l2_data_valid),
		.probe0(tx_l2_bytes_valid),
		.probe0(tx_l2_data),
		.probe0(tx_l2_commit),
		.probe0(tx_l2_drop),
		.probe0(tx_l2_dst_mac),
		.probe0(tx_l2_ethertype),

		.probe0(tx_frame_start),
		.probe1(tx_frame_data_valid),
		.probe2(tx_frame_bytes_valid),
		.probe3(tx_frame_data),
		.probe4(tx_bytes_left),
		.probe5(tx_count),
		.probe6(fifo_rd_data),
		.probe7(fifo_rd_data_ff),
		.probe8(fifo_rd_data_ff2),
		.probe9(header_pop),
		.probe10(header_rd_en),
		.probe11(header_rd_en_ff),
		.probe12(header_rd_size),

		.probe13(fifo_rd_en),
		.probe14(fifo_rd_offset),
		.probe15(fifo_pop_packet),
		.probe16(fifo_pop_size),
		.probe17(fifo_rd_data),
		.probe18(fifo_rd_size),

		.trig_out(trig_out),
		.trig_out_ack(trig_out_ack)
	);
	*/

endmodule
