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
	@brief 10-Gigabit Ethernet MAC

	Pretty simple, just convert control codes to status flags and insert/verify checksums

	Conventions
		rx_frame_start is asserted before, not simultaneous with, first assertion of rx_frame_data_valid
		rx_frame_bytes_valid is always 4 until last word in the packet, at which point it may take any value
		rx_frame_commit is asserted after, not simultaneous with, last assertion of rx_frame_data_valid
 */
module XGEthernetMAC(

	//XMGII bus
	input wire			xgmii_rx_clk,
	input wire[3:0]		xgmii_rxc,
	input wire[31:0]	xgmii_rxd,

	input wire			xgmii_tx_clk,
	output reg[3:0]		xgmii_txc				= 0,
	output reg[31:0]	xgmii_txd				= 0,

	//Link state flags (reset stuff as needed when link is down)
	input wire			link_up,

	//Data bus to upper layer stack (aligned to XGMII RX clock)
	//Streaming bus, don't act on this data until rx_frame_commit goes high
	output reg			rx_frame_start			= 0,
	output reg			rx_frame_data_valid		= 0,
	output reg[2:0]		rx_frame_bytes_valid	= 0,
	output reg[31:0]	rx_frame_data			= 0,
	output reg			rx_frame_commit			= 0,
	output reg			rx_frame_drop			= 0,

	//Data bus from upper layer stack (aligned to XGMII TX clock)
	//Well-formed layer 2 frames minus padding (if needed) and CRC.
	//No commit/drop flags, everything that comes in here gets sent.
	input wire			tx_frame_start,
	input wire			tx_frame_data_valid,
	input wire[2:0]		tx_frame_bytes_valid,
	input wire[31:0]	tx_frame_data

	//TODO: performance counters
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet protocol constants

	//Pull in XGMII table (shared with PCS core)
	`include "XGMII_CtlChars.vh"

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XGMII RX control character decoding

	wire[3:0] lane_has_end =
	{
		xgmii_rxc[3] && (xgmii_rxd[31:24] == XGMII_CTL_END),
		xgmii_rxc[2] && (xgmii_rxd[23:16] == XGMII_CTL_END),
		xgmii_rxc[1] && (xgmii_rxd[15:8]  == XGMII_CTL_END),
		xgmii_rxc[0] && (xgmii_rxd[7:0]   == XGMII_CTL_END)
	};

	wire[3:0] lane_has_error =
	{
		xgmii_rxc[3] && (xgmii_rxd[31:24] == XGMII_CTL_ERROR),
		xgmii_rxc[2] && (xgmii_rxd[23:16] == XGMII_CTL_ERROR),
		xgmii_rxc[1] && (xgmii_rxd[15:8]  == XGMII_CTL_ERROR),
		xgmii_rxc[0] && (xgmii_rxd[7:0]   == XGMII_CTL_ERROR)
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XGMII RX logic

	localparam STATE_IDLE		= 0;
	localparam STATE_PREAMBLE	= 1;
	localparam STATE_BODY		= 2;

	reg[1:0]	rx_state		= 0;

	reg			last_was_preamble	= 0;

	wire[31:0]	rx_crc_dout;
	reg[31:0]	crc_expected;
	reg[31:0]	crc_expected_ff;
	reg[31:0]	crc_expected_ff2;

	//Combinatorially figure out how many bytes of data in the current block are valid
	always @(*) begin
		rx_frame_bytes_valid			<= 0;
		crc_expected					<= 0;

		rx_frame_data_valid				<= 0;

		if(rx_state == STATE_BODY) begin

			//Feed everything before the end into the CRC engine
			//(except for the FCS)
			if(lane_has_end[3]) begin
				//entire rxd_ff is FCS, don't crc it at all
				crc_expected			<= rx_frame_data;
			end
			else if(lane_has_end[2]) begin
				rx_frame_bytes_valid	<= 1;
				rx_frame_data_valid		<= 1;
				crc_expected			<= { rx_frame_data[23:0], xgmii_rxd[31:24] };
			end
			else if(lane_has_end[1]) begin
				rx_frame_bytes_valid	<= 2;
				rx_frame_data_valid		<= 1;
				rx_frame_data_valid		<= 1;
				crc_expected			<= { rx_frame_data[15:0], xgmii_rxd[31:16] };
			end
			else if(lane_has_end[0]) begin
				rx_frame_bytes_valid	<= 3;
				rx_frame_data_valid		<= 1;
				rx_frame_data_valid		<= 1;
				crc_expected			<= { rx_frame_data[7:0], xgmii_rxd[31:8] };
			end

			//Packet is NOT ending this block. Feed the last 4 bytes to the CRC engine.
			//(We can't feed the current 4 bytes in yet, as they might be part of the crc!)
			//Also, make sure not to hash the preamble or SFD!
			else if(!last_was_preamble) begin
				rx_frame_bytes_valid	<= 4;
				rx_frame_data_valid		<= 1;
			end
		end

	end

	reg						fcs_pending_0	= 0;
	reg						fcs_pending_1	= 0;

	always @(posedge xgmii_rx_clk) begin

		rx_frame_start		<= 0;

		last_was_preamble	<= 0;

		rx_frame_data		<= xgmii_rxd;
		crc_expected_ff		<= crc_expected;
		crc_expected_ff2	<= crc_expected_ff;

		rx_frame_commit		<= 0;
		rx_frame_drop		<= 0;

		fcs_pending_0		<= 0;
		fcs_pending_1		<= fcs_pending_0;

		//Pipeline checksum processing by two cycles for timing.
		//This buys us some time, but we have to be careful when packets are at minimum spacing.
		if(fcs_pending_1) begin
			if(crc_expected_ff2 == rx_crc_dout)
				rx_frame_commit	<= 1;
			else
				rx_frame_drop	<= 1;
		end

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for a frame to start

			STATE_IDLE: begin

				//Ignore idles and errors
				//Look for start of frame (can only occur in leftmost XGMII lane)
				if(xgmii_rxc[3] && (xgmii_rxd[31:24] == XGMII_CTL_START) )
					rx_state		<= STATE_PREAMBLE;

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// PREAMBLE - Ethernet preamble, but no data for upper layers yet

			STATE_PREAMBLE: begin

				//Delay rx_frame_start by one cycle
				//to give checksum calculation a chance to complete
				rx_frame_start		<= 1;

				//We should have exactly one XGMII clock in this stage
				//and it should be data 55 55 55 D5. Anything else is an error, drop the frame.
				if( (xgmii_rxc != 0) || (xgmii_rxd != 32'h555555d5) )
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
			rx_frame_drop	<= 1;

			fcs_pending_0	<= 0;
			fcs_pending_1	<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive CRC32

	//Pipeline CRC input by one cycle to improve timing
	reg[2:0]		rx_frame_bytes_valid_ff	= 0;
	reg[31:0]		rx_frame_data_ff		= 0;

	always @(posedge xgmii_rx_clk) begin
		rx_frame_bytes_valid_ff	<= rx_frame_bytes_valid;
		rx_frame_data_ff		<= rx_frame_data;
	end

	CRC32_Ethernet_x32_variable rx_crc(

		.clk(xgmii_rx_clk),
		.reset(rx_frame_start),

		.din_len(rx_frame_bytes_valid_ff),
		.din(rx_frame_data_ff),
		.crc_out(rx_crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XGMII TX logic

	wire[31:0]	tx_crc_dout;
	reg[2:0]	tx_crc_bytes_valid			= 0;
	reg[31:0]	tx_crc_din					= 0;

	reg[13:0]	running_frame_len			= 0;
	reg[2:0]	tx_frame_bytes_valid_ff		= 0;
	reg[2:0]	tx_frame_bytes_valid_ff2	= 0;
	reg[2:0]	tx_frame_bytes_valid_ff3	= 0;

	localparam	TX_STATE_IDLE		= 4'h0;
	localparam	TX_STATE_PREAMBLE	= 4'h1;
	localparam	TX_STATE_BODY		= 4'h2;
	localparam	TX_STATE_FCS_0		= 4'h3;
	localparam	TX_STATE_FCS_1		= 4'h4;
	localparam	TX_STATE_FCS_2		= 4'h5;
	localparam	TX_STATE_FCS_3		= 4'h6;

	reg[3:0]	tx_state			= TX_STATE_IDLE;

	reg[3:0]	xgmii_txc_next		= 0;
	reg[31:0]	xgmii_txd_next		= 0;

	always @(posedge xgmii_tx_clk) begin

		//Default to sending idles
		xgmii_txc_next						<= 4'b1111;
		xgmii_txd_next						<= { XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE };

		//Default to forwarding the transmit data down the pipeline
		xgmii_txc							<= xgmii_txc_next;
		xgmii_txd							<= xgmii_txd_next;

		//Send incoming data words to the CRC engine
		tx_crc_din							<= tx_frame_data;
		tx_crc_bytes_valid					<= 0;

		//Save previous state
		tx_frame_bytes_valid_ff				<= tx_frame_bytes_valid;
		tx_frame_bytes_valid_ff2			<= tx_frame_bytes_valid_ff;
		tx_frame_bytes_valid_ff3			<= tx_frame_bytes_valid_ff2;

		case(tx_state)

			TX_STATE_IDLE: begin

				//Starting a new frame? Send preamble
				if(tx_frame_start) begin
					running_frame_len		<= 0;
					xgmii_txc_next			<= 4'b1000;
					xgmii_txd_next			<= { XGMII_CTL_START, 24'h55_55_55 };
					tx_state				<= TX_STATE_PREAMBLE;
				end

			end	//end TX_STATE_IDLE

			TX_STATE_PREAMBLE: begin

				//Send rest of preamble plus SFD
				xgmii_txc_next				<= 4'b0000;
				xgmii_txd_next				<= 32'h55_55_55_d5;

				//Calculate CRC for the first four data words as we go
				tx_crc_bytes_valid			<= 4;

				//Preamble doesn't count toward frame length,
				//but the data we're CRCing in the background does
				running_frame_len			<= 4;

				tx_state					<= TX_STATE_BODY;

			end	//end TX_STATE_PREAMBLE

			TX_STATE_BODY: begin

				//Send payload data
				xgmii_txc_next				<= 4'b0000;
				xgmii_txd_next				<= tx_crc_din;

				//New data coming!
				if(tx_frame_data_valid) begin

					//If we have a full block of data, crunch it
					if(tx_frame_bytes_valid == 4) begin
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
						running_frame_len	<= running_frame_len + tx_frame_bytes_valid;
						tx_crc_bytes_valid	<= tx_frame_bytes_valid;
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
				xgmii_txc_next				<= 4'b0000;
				xgmii_txd_next				<= tx_crc_din;

				tx_state					<= TX_STATE_FCS_1;

			end	//end TX_STATE_FCS_0: begin

			TX_STATE_FCS_1: begin

				//Bodge in the CRC as needed.
				tx_crc_bytes_valid		<= 0;

				xgmii_txc				<= 4'b0000;

				case(tx_frame_bytes_valid_ff2)

					//Frame just ended on a 4-byte boundary. Send the CRC immediately
					//(without a pipe delay)
					0: 	xgmii_txd		<= tx_crc_dout;

					//Frame ended with partial content.
					//Send the remainder of the data, but bodge in part of the CRC
					1: 	xgmii_txd		<= { xgmii_txd_next[31:24], tx_crc_dout[31:8]  };
					2: 	xgmii_txd		<= { xgmii_txd_next[31:16], tx_crc_dout[31:16] };
					3: 	xgmii_txd		<= { xgmii_txd_next[31:8],  tx_crc_dout[31:24] };

					//Frame ended with full content or padding.
					//Send the content, then the CRC next cycle.
					4:	xgmii_txd		<= xgmii_txd_next;

				endcase

				tx_state				<= TX_STATE_FCS_2;
				running_frame_len		<= running_frame_len + 3'd4;

			end	//end TX_STATE_FCS_1

			TX_STATE_FCS_2: begin

				tx_state					<= TX_STATE_IDLE;

				//See how much of the CRC got sent last cycle.
				//Add the rest of it, plus the stop marker if we have room for it
				case(tx_frame_bytes_valid_ff3)

					//CRC was sent, send stop marker plus idles
					0: begin
						xgmii_txc		<= 4'b1111;
						xgmii_txd		<= { XGMII_CTL_END, XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE };
					end

					//Partial CRC was sent, but we have room for the stop marker.
					//Send the rest plus the stop marker.
					1: begin
						xgmii_txc		<= 4'b0111;
						xgmii_txd		<= { tx_crc_dout[7:0], XGMII_CTL_END, XGMII_CTL_IDLE, XGMII_CTL_IDLE };
					end
					2: begin
						xgmii_txc		<= 4'b0011;
						xgmii_txd		<= { tx_crc_dout[15:0], XGMII_CTL_END, XGMII_CTL_IDLE };
					end
					3: begin
						xgmii_txc		<= 4'b0001;
						xgmii_txd		<= { tx_crc_dout[23:0], XGMII_CTL_END};
					end

					//CRC was not sent at all. Send it.
					//Need to send end marker next cycle still.
					4: begin
						xgmii_txc		<= 4'b0000;
						xgmii_txd		<= tx_crc_dout;
						tx_state		<= TX_STATE_FCS_3;
					end

				endcase

			end	//end TX_STATE_FCS_2

			//Send stop marker plus idles (if aligned to 4-byte boundary)
			TX_STATE_FCS_3: begin

				xgmii_txc				<= 4'b1111;
				xgmii_txd				<= { XGMII_CTL_END, XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE };

				running_frame_len		<= 0;

				tx_state				<= TX_STATE_IDLE;

			end	//end TX_STATE_FCS_3

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit CRC32

	CRC32_Ethernet_x32_variable tx_crc(

		.clk(xgmii_tx_clk),
		.reset(tx_frame_start),

		.din_len(tx_crc_bytes_valid),
		.din(tx_crc_din),
		.crc_out(tx_crc_dout)
	);

endmodule
