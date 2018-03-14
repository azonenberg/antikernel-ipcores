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
 */
module XGEthernetMAC(

	//XMGII bus
	input wire			xgmii_rx_clk,
	input wire[3:0]		xgmii_rxc,
	input wire[31:0]	xgmii_rxd,

	//Link state flags (reset stuff as needed when link is down)
	input wire			link_up,

	//Data bus to host
	//Streaming bus, don't act on this data until rx_frame_commit goes high
	output reg			rx_frame_start			= 0,
	output reg			rx_frame_data_valid		= 0,
	output reg[2:0]		rx_frame_bytes_valid	= 0,
	output reg[31:0]	rx_frame_data			= 0,
	output reg			rx_frame_commit			= 0,
	output reg			rx_frame_drop			= 0
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
	localparam STATE_FCS		= 3;

	reg[3:0]	rx_state		= 0;

	reg			last_was_preamble	= 0;

	wire[31:0]	crc_dout;
	reg[31:0]	crc_expected;
	reg[31:0]	crc_expected_ff;

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
				crc_expected			<= { rx_frame_data[23:0], xgmii_rxd[31:24] };
			end
			else if(lane_has_end[1]) begin
				rx_frame_bytes_valid	<= 2;
				rx_frame_data_valid		<= 1;
				crc_expected			<= { rx_frame_data[15:0], xgmii_rxd[31:16] };
			end
			else if(lane_has_end[0]) begin
				rx_frame_bytes_valid	<= 3;
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

	always @(posedge xgmii_rx_clk) begin

		rx_frame_start		<= 0;

		last_was_preamble	<= 0;

		rx_frame_data		<= xgmii_rxd;
		crc_expected_ff		<= crc_expected;

		rx_frame_commit		<= 0;
		rx_frame_drop		<= 0;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for a frame to start

			STATE_IDLE: begin

				//Ignore idles and errors
				//Look for start of frame (can only occur in leftmost XGMII lane)
				if(xgmii_rxc[3] && (xgmii_rxd[31:24] == XGMII_CTL_START) ) begin
					rx_frame_start	<= 1;
					rx_state		<= STATE_PREAMBLE;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// PREAMBLE - Ethernet preamble, but no data for upper layers yet

			STATE_PREAMBLE: begin

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
				if(lane_has_end)
					rx_state		<= STATE_FCS;

			end	//end STATE_BODY

			STATE_FCS: begin
				if(crc_expected_ff == crc_dout)
					rx_frame_commit	<= 1;
				else
					rx_frame_drop	<= 1;

				rx_state			<= STATE_IDLE;
			end

		endcase

		//Handle errors in the middle of the packet (just drop it)
		//This overrides and any all other processing
		if(lane_has_error && (rx_state != STATE_IDLE) ) begin
			rx_state		<= STATE_IDLE;
			rx_frame_drop	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive CRC32

	CRC32_Ethernet_x32_variable rx_crc(

		.clk(xgmii_rx_clk),
		.reset(rx_frame_start),

		.din_len(rx_frame_bytes_valid),
		.din(rx_frame_data),
		.crc_out(crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug logic analyzer

	ila_0 ila(
		.clk(xgmii_rx_clk),

		.probe0(xgmii_rxc),
		.probe1(xgmii_rxd),
		.probe2(rx_state),
		.probe3(rx_frame_start),
		.probe4(rx_frame_bytes_valid),
		.probe5(rx_frame_data),
		.probe6(crc_dout),
		.probe7(last_was_preamble),
		.probe8(lane_has_end),
		.probe9(lane_has_error),
		.probe10(crc_expected_ff),
		.probe11(rx_frame_data_valid),
		.probe12(rx_frame_commit),
		.probe13(rx_frame_drop)
	);

endmodule
