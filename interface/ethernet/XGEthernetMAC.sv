`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2022 Andrew D. Zonenberg                                                                          *
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

`include "GmiiBus.svh"
`include "EthernetBus.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief 10-Gigabit Ethernet MAC

	Pretty simple, just convert control codes to status flags and insert/verify checksums

	Conventions
		rx_bus.start is asserted before, not simultaneous with, first assertion of rx_bus.data_valid
		rx_bus.bytes_valid is always 4 until last word in the packet, at which point it may take any value
		rx_bus.commit is asserted after, not simultaneous with, last assertion of rx_bus.data_valid
 */
module XGEthernetMAC(

	//XMGII bus
	input wire					xgmii_rx_clk,
	input wire XgmiiBus			xgmii_rx_bus,

	input wire					xgmii_tx_clk,
	output XgmiiBus				xgmii_tx_bus	= 0,

	//Link state flags (reset stuff as needed when link is down)
	input wire					link_up,

	//Data bus to upper layer stack (aligned to XGMII RX clock)
	//Streaming bus, don't act on this data until rx_bus.commit goes high
	output EthernetRxBus		rx_bus			= 0,

	//Data bus from upper layer stack (aligned to XGMII TX clock)
	//Well-formed layer 2 frames minus padding (if needed) and CRC.
	//No commit/drop flags, everything that comes in here gets sent.
	input wire EthernetTxBus	tx_bus

	//TODO: performance counters
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet protocol constants

	//Pull in XGMII table (shared with PCS core)
	`include "XGMII_CtlChars.svh"

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XGMII RX control character decoding

	wire[3:0] lane_has_end =
	{
		xgmii_rx_bus.ctl[3] && (xgmii_rx_bus.data[31:24] == XGMII_CTL_END),
		xgmii_rx_bus.ctl[2] && (xgmii_rx_bus.data[23:16] == XGMII_CTL_END),
		xgmii_rx_bus.ctl[1] && (xgmii_rx_bus.data[15:8]  == XGMII_CTL_END),
		xgmii_rx_bus.ctl[0] && (xgmii_rx_bus.data[7:0]   == XGMII_CTL_END)
	};

	wire[3:0] lane_has_error =
	{
		xgmii_rx_bus.ctl[3] && (xgmii_rx_bus.data[31:24] == XGMII_CTL_ERROR),
		xgmii_rx_bus.ctl[2] && (xgmii_rx_bus.data[23:16] == XGMII_CTL_ERROR),
		xgmii_rx_bus.ctl[1] && (xgmii_rx_bus.data[15:8]  == XGMII_CTL_ERROR),
		xgmii_rx_bus.ctl[0] && (xgmii_rx_bus.data[7:0]   == XGMII_CTL_ERROR)
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XGMII RX logic

	enum logic[1:0]
	{
		STATE_IDLE		= 0,
		STATE_PREAMBLE	= 1,
		STATE_BODY		= 2
	} rx_state			= STATE_IDLE;

	logic		last_was_preamble	= 0;

	wire[31:0]	rx_crc_dout;
	logic[31:0]	crc_expected;
	logic[31:0]	crc_expected_ff;
	logic[31:0]	crc_expected_ff2;

	//Combinatorially figure out how many bytes of data in the current block are valid
	always_comb begin
		rx_bus.bytes_valid			= 0;
		crc_expected				= 0;

		rx_bus.data_valid			= 0;

		if(rx_state == STATE_BODY) begin

			//Feed everything before the end into the CRC engine
			//(except for the FCS)
			if(lane_has_end[3]) begin
				//entire rxd_ff is FCS, don't crc it at all
				crc_expected		= rx_bus.data;
			end
			else if(lane_has_end[2]) begin
				rx_bus.bytes_valid	= 1;
				rx_bus.data_valid	= 1;
				crc_expected		= { rx_bus.data[23:0], xgmii_rx_bus.data[31:24] };
			end
			else if(lane_has_end[1]) begin
				rx_bus.bytes_valid	= 2;
				rx_bus.data_valid	= 1;
				crc_expected		= { rx_bus.data[15:0], xgmii_rx_bus.data[31:16] };
			end
			else if(lane_has_end[0]) begin
				rx_bus.bytes_valid	= 3;
				rx_bus.data_valid	= 1;
				crc_expected		= { rx_bus.data[7:0], xgmii_rx_bus.data[31:8] };
			end

			//Packet is NOT ending this block. Feed the last 4 bytes to the CRC engine.
			//(We can't feed the current 4 bytes in yet, as they might be part of the crc!)
			//Also, make sure not to hash the preamble or SFD!
			else if(!last_was_preamble) begin
				rx_bus.bytes_valid		= 4;
				rx_bus.data_valid		= 1;
			end
		end

	end

	logic					fcs_pending_0	= 0;
	logic					fcs_pending_1	= 0;

	always @(posedge xgmii_rx_clk) begin

		rx_bus.start		<= 0;
		rx_bus.commit		<= 0;
		rx_bus.drop			<= 0;

		if(xgmii_rx_bus.valid) begin

			last_was_preamble	<= 0;

			rx_bus.data			<= xgmii_rx_bus.data;
			crc_expected_ff		<= crc_expected;
			crc_expected_ff2	<= crc_expected_ff;

			fcs_pending_0		<= 0;
			fcs_pending_1		<= fcs_pending_0;

			//Pipeline checksum processing by two cycles for timing.
			//This buys us some time, but we have to be careful when packets are at minimum spacing.
			if(fcs_pending_1) begin
				if(crc_expected_ff2 == rx_crc_dout)
					rx_bus.commit	<= 1;
				else
					rx_bus.drop		<= 1;
			end

			case(rx_state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// IDLE - wait for a frame to start

				STATE_IDLE: begin

					//Ignore idles and errors
					//Look for start of frame (can only occur in leftmost XGMII lane)
					if(xgmii_rx_bus.ctl[3] && (xgmii_rx_bus.data[31:24] == XGMII_CTL_START) )
						rx_state		<= STATE_PREAMBLE;

				end	//end STATE_IDLE

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// PREAMBLE - Ethernet preamble, but no data for upper layers yet

				STATE_PREAMBLE: begin

					//Delay rx_bus.start by one cycle
					//to give checksum calculation a chance to complete
					rx_bus.start		<= 1;

					//We should have exactly one XGMII clock in this stage
					//and it should be data 55 55 55 D5. Anything else is an error, drop the frame.
					if( (xgmii_rx_bus.ctl != 0) || (xgmii_rx_bus.data != 32'h555555d5) )
						rx_state		<= STATE_IDLE;

					//If we get here we're good, go on to the body
					else
						rx_state		<= STATE_BODY;

					last_was_preamble	<= 1;

				end	//end STATE_PREAMBLE

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// BODY - data to be processed by upper protocol layers

				STATE_BODY: begin

					//If we hit the end of the packet, stop
					//This can happen in any lane as packet lengths are not guaranteed to be 32-bit aligned
					if(lane_has_end) begin
						fcs_pending_0	<= 1;
						rx_state		<= STATE_IDLE;
					end

				end	//end STATE_BODY

			endcase

			//Handle errors in the middle of the packet (just drop it)
			//This overrides and any all other processing
			if(lane_has_error && (rx_state != STATE_IDLE) ) begin
				rx_state		<= STATE_IDLE;
				rx_bus.drop		<= 1;

				fcs_pending_0	<= 0;
				fcs_pending_1	<= 0;
			end
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive CRC32

	CRC32_Ethernet_x32_variable_lat2 rx_crc(

		.clk(xgmii_rx_clk),
		.reset(rx_bus.start),

		.din_len(rx_bus.bytes_valid),
		.din(rx_bus.data),
		.crc_out(rx_crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XGMII TX logic

	wire[31:0]	tx_crc_dout;
	logic[2:0]	tx_crc_bytes_valid			= 0;
	logic[31:0]	tx_crc_din					= 0;

	logic[13:0]	running_frame_len			= 0;
	logic[2:0]	tx_frame_bytes_valid_ff		= 0;
	logic[2:0]	tx_frame_bytes_valid_ff2	= 0;
	logic[2:0]	tx_frame_bytes_valid_ff3	= 0;

	enum logic[2:0]
	{
		TX_STATE_IDLE		= 3'h0,
		TX_STATE_PREAMBLE	= 3'h1,
		TX_STATE_BODY		= 3'h2,
		TX_STATE_FCS_0		= 3'h3,
		TX_STATE_FCS_1		= 3'h4,
		TX_STATE_FCS_2		= 3'h5,
		TX_STATE_FCS_3		= 3'h6,
		TX_STATE_FCS_4		= 3'h7
	} tx_state				= TX_STATE_IDLE;

	logic[3:0]	xgmii_txc_next1		= 0;
	logic[31:0]	xgmii_txd_next1		= 0;

	logic[3:0]	xgmii_txc_next2		= 0;
	logic[31:0]	xgmii_txd_next2		= 0;

	always_ff @(posedge xgmii_tx_clk) begin

		//Default to sending idles
		xgmii_txc_next1						<= 4'b1111;
		xgmii_txd_next1						<= { XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE };

		//Default to forwarding the transmit data down the pipeline
		xgmii_txc_next2						<= xgmii_txc_next1;
		xgmii_txd_next2						<= xgmii_txd_next1;
		xgmii_tx_bus.ctl					<= xgmii_txc_next2;
		xgmii_tx_bus.data					<= xgmii_txd_next2;

		//Send incoming data words to the CRC engine
		tx_crc_din							<= tx_bus.data;
		tx_crc_bytes_valid					<= 0;

		//Save previous state
		tx_frame_bytes_valid_ff				<= tx_bus.bytes_valid;
		tx_frame_bytes_valid_ff2			<= tx_frame_bytes_valid_ff;
		tx_frame_bytes_valid_ff3			<= tx_frame_bytes_valid_ff2;

		case(tx_state)

			TX_STATE_IDLE: begin

				//Starting a new frame? Send preamble
				if(tx_bus.start) begin
					running_frame_len		<= 0;
					xgmii_txc_next1			<= 4'b1000;
					xgmii_txd_next1			<= { XGMII_CTL_START, 24'h55_55_55 };
					tx_state				<= TX_STATE_PREAMBLE;
				end

			end	//end TX_STATE_IDLE

			TX_STATE_PREAMBLE: begin

				//Send rest of preamble plus SFD
				xgmii_txc_next1				<= 4'b0000;
				xgmii_txd_next1				<= 32'h55_55_55_d5;

				//Calculate CRC for the first four data words as we go
				tx_crc_bytes_valid			<= 4;

				//Preamble doesn't count toward frame length,
				//but the data we're CRCing in the background does
				running_frame_len			<= 4;

				tx_state					<= TX_STATE_BODY;

			end	//end TX_STATE_PREAMBLE

			TX_STATE_BODY: begin

				//Send payload data
				xgmii_txc_next1				<= 4'b0000;
				xgmii_txd_next1				<= tx_crc_din;

				//New data coming!
				if(tx_bus.data_valid) begin

					//If we have a full block of data, crunch it
					if(tx_bus.bytes_valid == 4) begin
						running_frame_len	<= running_frame_len + 3'd4;
						tx_crc_bytes_valid	<= 4;
					end

					//Packet ended, but not at a 4-byte boundary.
					//If we have less than a 64-byte packet including the current block,
					//always process a full 4 bytes (padding if needed, but leaving room for CRC)
					else if(running_frame_len <= 56) begin
						running_frame_len	<= running_frame_len + 3'd4;
						tx_crc_bytes_valid	<= 4;
					end

					//Packet ended at an unaligned boundary, but we're above the minimum packet size.
					//Just crunch this data by itself, then finish
					else begin
						running_frame_len	<= running_frame_len + tx_bus.bytes_valid;
						tx_crc_bytes_valid	<= tx_bus.bytes_valid;
						tx_state			<= TX_STATE_FCS_0;
					end

				end

				//Packet ended at a 4-byte aligned boundary.
				//CRC a block of zeroes if we need to pad.
				else begin
					if(running_frame_len <= 56) begin
						tx_crc_din			<= 32'h0;
						tx_crc_bytes_valid	<= 4;
						running_frame_len	<= running_frame_len + 3'd4;
					end

					else
						tx_state			<= TX_STATE_FCS_0;
				end

			end	//end TX_STATE_BODY

			//Push the last data block down the pipe while waiting for the CRC to be calculated
			TX_STATE_FCS_0: begin

				//Send payload data
				xgmii_txc_next1				<= 4'b0000;
				xgmii_txd_next1				<= tx_crc_din;

				tx_state					<= TX_STATE_FCS_1;

			end	//end TX_STATE_FCS_0: begin

			//Wait for CRC latency
			TX_STATE_FCS_1: begin
				tx_crc_bytes_valid			<= 0;
				tx_state					<= TX_STATE_FCS_2;
			end	//end TX_STATE_FCS_1: begin

			TX_STATE_FCS_2: begin

				//Bodge in the CRC as needed.
				tx_crc_bytes_valid		<= 0;

				xgmii_tx_bus.ctl		<= 4'b0000;

				case(tx_frame_bytes_valid_ff2)

					//Frame just ended on a 4-byte boundary. Send the CRC immediately
					//(without a pipe delay)
					0: 	xgmii_tx_bus.data	<= tx_crc_dout;

					//Frame ended with partial content.
					//Send the remainder of the data, but bodge in part of the CRC
					1: 	xgmii_tx_bus.data	<= { xgmii_txd_next1[31:24], tx_crc_dout[31:8]  };
					2: 	xgmii_tx_bus.data	<= { xgmii_txd_next1[31:16], tx_crc_dout[31:16] };
					3: 	xgmii_tx_bus.data	<= { xgmii_txd_next1[31:8],  tx_crc_dout[31:24] };

					//Frame ended with full content or padding.
					//Send the content, then the CRC next cycle.
					4:	xgmii_tx_bus.data	<= xgmii_txd_next1;

				endcase

				tx_state				<= TX_STATE_FCS_3;
				running_frame_len		<= running_frame_len + 3'd4;

			end	//end TX_STATE_FCS_2

			TX_STATE_FCS_3: begin

				tx_state					<= TX_STATE_IDLE;

				//See how much of the CRC got sent last cycle.
				//Add the rest of it, plus the stop marker if we have room for it
				case(tx_frame_bytes_valid_ff3)

					//CRC was sent, send stop marker plus idles
					0: begin
						xgmii_tx_bus.ctl	<= 4'b1111;
						xgmii_tx_bus.data	<= { XGMII_CTL_END, XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE };
					end

					//Partial CRC was sent, but we have room for the stop marker.
					//Send the rest plus the stop marker.
					1: begin
						xgmii_tx_bus.ctl	<= 4'b0111;
						xgmii_tx_bus.data	<= { tx_crc_dout[7:0], XGMII_CTL_END, XGMII_CTL_IDLE, XGMII_CTL_IDLE };
					end
					2: begin
						xgmii_tx_bus.ctl	<= 4'b0011;
						xgmii_tx_bus.data	<= { tx_crc_dout[15:0], XGMII_CTL_END, XGMII_CTL_IDLE };
					end
					3: begin
						xgmii_tx_bus.ctl	<= 4'b0001;
						xgmii_tx_bus.data	<= { tx_crc_dout[23:0], XGMII_CTL_END};
					end

					//CRC was not sent at all. Send it.
					//Need to send end marker next cycle still.
					4: begin
						xgmii_tx_bus.ctl	<= 4'b0000;
						xgmii_tx_bus.data	<= tx_crc_dout;
						tx_state			<= TX_STATE_FCS_4;
					end

				endcase

			end	//end TX_STATE_FCS_3

			//Send stop marker plus idles (if aligned to 4-byte boundary)
			TX_STATE_FCS_4: begin

				xgmii_tx_bus.ctl			<= 4'b1111;
				xgmii_tx_bus.data			<= { XGMII_CTL_END, XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE };

				running_frame_len			<= 0;

				tx_state					<= TX_STATE_IDLE;

			end	//end TX_STATE_FCS_4

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit CRC32

	CRC32_Ethernet_x32_variable_lat2 tx_crc(

		.clk(xgmii_tx_clk),
		.reset(tx_bus.start),

		.din_len(tx_crc_bytes_valid),
		.din(tx_crc_din),
		.crc_out(tx_crc_dout)
	);

endmodule
