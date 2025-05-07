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

import EthernetBus::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief 10/100/1000 Mbps Ethernet MAC with 32 bit AXI-Stream interface

	Both TX and RX stream data is a continuous aligned stream except for the last word of the packet, which may contain
	zero or more null bytes for packets whose length is not a multiple of 4 bytes.

	Multiple streams or destinations are not supported (TID / TDEST must be zero width)

	TX side:
		ACLK must be a constant 125 MHz regardless of data rate
		GMII TX data is synchronous to TX ACLK

		No TUSER signals are used.

		Packet may be sent faster or slower than the PHY signaling rate. Packets are buffered internally to the MAC and
		will not be sent out on the GMII bus until they have been fully buffered.

	RX side:
		gmii_rx_clock is echoed as ACLK
		TREADY is ignored and back pressure is not supported. An external FIFO between the MAC and other logic is
		required if backpressure must be supported.
		TUSER[0]	error flag valid only when TLAST is asserted
					If true, data should be ignored (packet contained a CRC error or in-band error notification)
					If false, packet was valid
 */
module AXIS_TriSpeedEthernetMAC #(
	parameter RX_CRC_DISABLE	= 0,
	parameter TX_FIFO_USE_BLOCK	= 1,

	parameter DEBUG_LANE		= 0	//TODO remove this
)(
	//GMII buses
	input wire					gmii_rx_clk,
	input wire GmiiBus			gmii_rx_bus,

	input wire					gmii_tx_clk,
	output GmiiBus				gmii_tx_bus 		= {1'b0, 1'b0, 8'b0},

	//Link state flags
	//Synchronous to RX clock (TODO put this on APB too)
	input wire					link_up,
	input wire lspeed_t			link_speed,

	//AXI interfaces
	AXIStream.receiver			axi_tx,
	AXIStream.transmitter		axi_rx
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX CRC calculation

	enum logic[3:0]
	{
		RX_STATE_IDLE		= 4'h0,
		RX_STATE_DROP		= 4'h1,
		RX_STATE_PREAMBLE	= 4'h2,
		RX_STATE_FRAME_DATA	= 4'h3,
		RX_STATE_CRC		= 4'h4
	}
	rx_state = RX_STATE_IDLE;

	//Do it 8 bits wide, every clock, to save area
	wire		rx_crc_reset	= (rx_state == RX_STATE_PREAMBLE);
	wire		rx_crc_update	= gmii_rx_bus.en && (rx_state == RX_STATE_FRAME_DATA) && gmii_rx_bus.dvalid;
	wire[31:0]	rx_crc_calculated;

	CRC32_Ethernet rx_crc_calc(
		.clk(gmii_rx_clk),
		.reset(rx_crc_reset),
		.update(rx_crc_update),
		.din(gmii_rx_bus.data),
		.crc_flipped(rx_crc_calculated)
	);

	//Delay by 5 cycles so the CRC is there when we want to use it
	reg[31:0]	rx_crc_calculated_ff	= 0;
	reg[31:0]	rx_crc_calculated_ff2	= 0;
	reg[31:0]	rx_crc_calculated_ff3	= 0;
	reg[31:0]	rx_crc_calculated_ff4	= 0;
	reg[31:0]	rx_crc_calculated_ff5	= 0;

	always_ff @(posedge gmii_rx_clk) begin
		if(gmii_rx_bus.dvalid) begin
			rx_crc_calculated_ff5	<= rx_crc_calculated_ff4;
			rx_crc_calculated_ff4	<= rx_crc_calculated_ff3;
			rx_crc_calculated_ff3	<= rx_crc_calculated_ff2;
			rx_crc_calculated_ff2	<= rx_crc_calculated_ff;
			rx_crc_calculated_ff	<= rx_crc_calculated;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// AXI config

	assign axi_rx.aclk		= gmii_rx_clk;
	assign axi_rx.areset_n	= link_up;
	assign axi_rx.twakeup	= 1;
	assign axi_rx.tid		= 0;	//TID not used
	assign axi_rx.tdest		= 0;	//TDEST not used

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main RX datapath

	logic[1:0]	rx_bytepos				= 0;
	logic[31:0]	rx_pending_data			= 0;

	logic		rx_frame_data_valid_adv	= 0;
	logic[31:0]	rx_frame_data_adv		= 0;

	wire[31:0]	rx_crc_expected;
	assign rx_crc_expected	= { rx_pending_data[7:0], rx_pending_data[15:8], rx_pending_data[23:16], rx_pending_data[31:24] };

	always_ff @(posedge gmii_rx_clk) begin

		axi_rx.tvalid			<= 0;
		axi_rx.tkeep			<= 0;
		axi_rx.tstrb			<= 0;
		axi_rx.tdata			<= 0;
		axi_rx.tuser			<= 0;
		axi_rx.tlast			<= 0;

		if(gmii_rx_bus.dvalid) begin

			case(rx_state)

				//Wait for a new frame to start
				RX_STATE_IDLE: begin

					//Ignore rx_er outside of a packet

					//Something is here!
					if(gmii_rx_bus.en) begin

						//Should be a preamble (55 55 55 ...)
						if( (gmii_rx_bus.data == 8'h55) && !gmii_rx_bus.er )
							rx_state			<= RX_STATE_PREAMBLE;

						//Anything else is a problem, ignore it
						else
							rx_state			<= RX_STATE_DROP;
					end

				end	//end RX_STATE_IDLE

				//Wait for SFD
				RX_STATE_PREAMBLE: begin

					//Drop frame if it truncates before the SFD
					if(!gmii_rx_bus.en)
						rx_state				<= RX_STATE_IDLE;

					//Tell the upper layer we are starting the frame when we hit the SFD.
					//No point in even telling them about runt packets that end during the preamble.
					else if(gmii_rx_bus.data == 8'hd5) begin
						rx_bytepos				<= 0;
						rx_state				<= RX_STATE_FRAME_DATA;
						rx_frame_data_valid_adv	<= 0;
					end

					//Still preamble, keep going
					else if(gmii_rx_bus.data == 8'h55) begin
					end

					//Anything else before the SFD is an error, drop it.
					//Don't have to tell upper layer as we never even told them a frame was coming.
					else
						rx_state				<= RX_STATE_DROP;

				end	//end RX_STATE_PREAMBLE

				//Actual packet data
				RX_STATE_FRAME_DATA: begin

					//End of frame - push any fractional message word that might be waiting
					if(!gmii_rx_bus.en) begin
						rx_state				<= RX_STATE_CRC;

						axi_rx.tvalid			<= (rx_bytepos != 0);
						axi_rx.tkeep			<= 4'b1111;
						axi_rx.tdata			<= rx_frame_data_adv;

						case(rx_bytepos)
							0: begin
								//should never happen, leave tvalid as 0
							end

							1:	axi_rx.tstrb	<= 4'b0001;
							2:	axi_rx.tstrb	<= 4'b0011;
							3:	axi_rx.tstrb	<= 4'b0111;
						endcase

					end

					//Frame data still coming
					else begin

						rx_bytepos						<= rx_bytepos + 1'h1;

						case(rx_bytepos)
							0:	rx_pending_data			<= { 23'h0, gmii_rx_bus.data };
							1:	rx_pending_data[15:8]	<= gmii_rx_bus.data;
							2:	rx_pending_data[23:16]	<= gmii_rx_bus.data;
							3:	rx_pending_data[31:24]	<= gmii_rx_bus.data;
						endcase

						//We've received a full word!
						if(rx_bytepos == 3) begin

							//Save this word in the buffer for next time around
							rx_frame_data_valid_adv	<= 1;
							rx_frame_data_adv		<= { gmii_rx_bus.data, rx_pending_data[23:0] };

							//Send the PREVIOUS word to the host
							//We need a pipeline delay because of the CRC - don't want to get the CRC confused with application layer data!
							if(rx_frame_data_valid_adv) begin
								axi_rx.tvalid		<= 1;
								axi_rx.tdata		<= rx_frame_data_adv;
								axi_rx.tkeep		<= 4'b1111;
								axi_rx.tstrb		<= 4'b1111;
							end
						end
					end
				end	//end RX_STATE_FRAME_DATA

				RX_STATE_CRC: begin
					rx_state			<= RX_STATE_IDLE;

					axi_rx.tlast		<= 1;

					//Validate the CRC (details depend on length of the packet)
					if( (rx_crc_calculated_ff5 == rx_crc_expected) || RX_CRC_DISABLE )
						axi_rx.tuser	<= 0;
					else
						axi_rx.tuser	<= 1;

				end

				//If skipping a frame due to a fault, ignore everything until the frame ends
				RX_STATE_DROP: begin
					if(!gmii_rx_bus.en)
						rx_state		<= RX_STATE_IDLE;
				end	//end RX_STATE_DROP

			endcase

		end

	end

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize link speed into TX clock domain
	// Use separate syncs, OK if they glitch for a cycle during transition

	lspeed_t	link_speed_sync;

	ThreeStageSynchronizer sync_link_speed_0(
		.clk_in(gmii_rx_clk),
		.din(link_speed[0]),
		.clk_out(gmii_tx_clk),
		.dout(link_speed_sync[0])
	);
	ThreeStageSynchronizer sync_link_speed_1(
		.clk_in(gmii_rx_clk),
		.din(link_speed[1]),
		.clk_out(gmii_tx_clk),
		.dout(link_speed_sync[1])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX stuff

	enum logic[3:0]
	{
		TX_STATE_IDLE		= 4'h0,
		TX_STATE_PREAMBLE	= 4'h1,
		TX_STATE_FRAME_DATA	= 4'h2,
		TX_STATE_PADDING	= 4'h3,
		TX_STATE_CRC_0		= 4'h4,
		TX_STATE_CRC_1		= 4'h5,
		TX_STATE_IFG		= 4'h6
	}
	tx_state = TX_STATE_IDLE;

	logic[3:0] tx_count	= 0;

	//Need a larger FIFO to support 10/100 mode since packets always show up at gigabit rate
	//Has to have room for an entire packet. No jumbo frame support for the moment.
	logic		tx_fifo_pop	= 0;
	wire[7:0]	tx_fifo_rdata;
	wire[11:0]	tx_fifo_rsize;

	logic[10:0]	tx_frame_len = 0;

	SingleClockFifo #(
		.WIDTH(8),
		.DEPTH(2048),
		.USE_BLOCK(TX_FIFO_USE_BLOCK)
	) tx_fifo (
		.clk(gmii_tx_clk),

		.wr(tx_bus.data_valid),
		.din(tx_bus.data[7:0]),

		.rd(tx_fifo_pop),
		.dout(tx_fifo_rdata),

		.overflow(),
		.underflow(),
		.empty(),
		.full(),
		.rsize(tx_fifo_rsize),
		.wsize(),
		.reset(tx_bus.start)		//wipe any existing junk when a frame starts
	);

	logic		dvalid_adv			= 0;

	logic		tx_en			= 0;
	logic[7:0]	tx_data			= 0;
	wire		tx_crc_update	= ( (tx_state == TX_STATE_FRAME_DATA) || (tx_state == TX_STATE_PADDING) ) && dvalid_adv;
	wire[31:0]	tx_crc;
	logic[7:0]	tx_crc_din;

	always_comb begin
		if(tx_state == TX_STATE_FRAME_DATA)
			tx_crc_din	<= tx_fifo_rdata;
		else
			tx_crc_din	<= 0;
	end

	CRC32_Ethernet tx_crc_calc(
		.clk(gmii_tx_clk),
		.reset(tx_bus.start),
		.update(tx_crc_update),
		.din(tx_crc_din),
		.crc_flipped(tx_crc)
	);

	//Generate pulses every byte
	logic[7:0]	gmii_speed_count	= 0;
	always_ff @(posedge gmii_tx_clk) begin

		case(link_speed_sync)
			LINK_SPEED_10M: begin

				if(gmii_speed_count >= 99) begin
					dvalid_adv			<= 1;
					gmii_speed_count	<= 0;
				end

				else begin
					dvalid_adv			<= 0;
					gmii_speed_count	<= gmii_speed_count + 1'h1;
				end

			end

			LINK_SPEED_100M: begin

				if(gmii_speed_count >= 9) begin
					dvalid_adv			<= 1;
					gmii_speed_count	<= 0;
				end

				else begin
					dvalid_adv			<= 0;
					gmii_speed_count	<= gmii_speed_count + 1'h1;
				end

			end

			LINK_SPEED_1000M: begin
				dvalid_adv			<= 1;
				gmii_speed_count	<= 0;
			end

		endcase

	end

	logic	start_pending	= 0;
	always_ff @(posedge gmii_tx_clk) begin

		tx_fifo_pop	<= 0;

		gmii_tx_bus.dvalid	<= dvalid_adv;

		if(tx_bus.start)
			start_pending	<= 1;

		if(dvalid_adv) begin

			gmii_tx_bus.en		<= 0;
			gmii_tx_bus.er		<= 0;
			gmii_tx_bus.data	<= 0;

			//Pipeline delay on GMII TX bus, so we have time to compute the CRC
			tx_en				<= 0;
			tx_data				<= 0;
			gmii_tx_bus.en		<= tx_en;
			gmii_tx_bus.data	<= tx_data;

			if(tx_state != TX_STATE_IDLE)
				tx_frame_len	<= tx_frame_len + 1'h1;

			case(tx_state)

				//If a new frame is starting, begin the preamble while buffering the message content
				TX_STATE_IDLE: begin
					tx_frame_len		<= 0;

					if(tx_bus.start || start_pending) begin
						tx_ready		<= 0;
						tx_en			<= 1;
						tx_data			<= 8'h55;1
						tx_count		<= 1;
						tx_state		<= TX_STATE_PREAMBLE;
						tx_frame_len	<= 1;
						start_pending	<= 0;
					end
				end	//end TX_STATE_IDLE

				//Send the preamble
				TX_STATE_PREAMBLE: begin

					tx_en			<= 1;
					tx_data			<= 8'h55;

					tx_count		<= tx_count + 1'h1;

					//Start popping message data a cycle early in gig mode to allow for FIFO latency
					if(link_speed_sync == LINK_SPEED_1000M) begin
						if(tx_count >= 6)
							tx_fifo_pop	<= 1;
					end

					//In 10/100 mode, we have plenty of time so don't advance the pop
					else if(tx_count == 7)
						tx_fifo_pop	<= 1;

					if(tx_count == 7) begin
						tx_data		<= 8'hd5;
						tx_count	<= 0;
						tx_state	<= TX_STATE_FRAME_DATA;
					end

				end	//end TX_STATE_PREAMBLE

				TX_STATE_FRAME_DATA: begin

					//Not last word? Pop it
					//Can also pop if size == 1 and we're not popping
					if( (tx_fifo_rsize > 1) || ( (tx_fifo_rsize == 1) && !tx_fifo_pop) )
						tx_fifo_pop	<= 1;

					//Don't finish up if there's still one last byte of data being popped
					else if(tx_fifo_pop) begin
					end

					//Packet must be at least 66 bytes including preamble
					//Add padding if we didn't get there yet
					else begin
						if(tx_frame_len > 66)
							tx_state	<= TX_STATE_CRC_0;
						else if(tx_fifo_rsize == 0)				//wait for last byte before paddingy
							tx_state	<= TX_STATE_PADDING;
					end

					tx_en	<= 1;
					tx_data	<= tx_fifo_rdata;

				end	//end TX_STATE_FRAME_DATA

				//Wait for CRC calculation
				TX_STATE_CRC_0: begin
					tx_state	<= TX_STATE_CRC_1;
				end	//end TX_STATE_CRC_0

				//Actually send the CRC
				TX_STATE_CRC_1: begin

					//Transmit directly (no forwarding)
					gmii_tx_bus.en	<= 1;

					tx_count	<= tx_count + 1'h1;

					if(tx_count == 3) begin
						tx_count	<= 0;
						tx_state	<= TX_STATE_IFG;
					end

					case(tx_count)
						0:	gmii_tx_bus.data	<= tx_crc[31:24];
						1:	gmii_tx_bus.data	<= tx_crc[23:16];
						2:	gmii_tx_bus.data	<= tx_crc[15:8];
						3:	gmii_tx_bus.data	<= tx_crc[7:0];

					endcase
				end	//end TX_STATE_CRC_1

				//Pad frame out to 68 bytes including preamble but not FCS
				TX_STATE_PADDING: begin
					tx_en	<= 1;
					tx_data	<= 0;

					if(tx_frame_len > 66)
						tx_state	<= TX_STATE_CRC_0;

				end	//end TX_STATE_PADDING

				//Inter-frame gap (min 12 octets)
				TX_STATE_IFG: begin
					tx_count		<= tx_count + 1'h1;

					if(tx_count == 11) begin
						tx_ready	<= 1;
						tx_state	<= TX_STATE_IDLE;
					end

				end	//end TX_STATE_IFG

			endcase

		end

	end
	*/

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	if(DEBUG_LANE == 0) begin

	end

endmodule
