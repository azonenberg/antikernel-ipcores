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

/**
	@brief Serial Chip to Chip Bus link layer

	Valid configurations for SYMBOL_WIDTH:
		* TODO: support 2
		* 4
		* TODO: support 8?

	All control characters other than K28.6 end-of-frame must occur in the lane 0 position.
	This means that both sides of a link must have the same SYMBOL_WIDTH setting and thus use the same amount of
	padding.

	Overall frame format:
		IDLE: K28.5 D16.2 (padded with D0.0 to SYMBOL_WIDTH if necessary)
		START: K character
			K30.7: LL ACK or error
			K28.6: end of frame (not legal outside a frame)
			All other K characters forwarded up to transaction layer
		SEQUENCE: 8 bit unsigned, wraps around
		DATA: transaction layer dependent
		CRC: CRC-8-CCITT/ATM (x^8 + x^2 + x + 1) of upper layer data only
		END: K28.6
		PADDING: Zero or more D0.0 bytes appended after CRC to fill out a full SERDES word
 */
module SCCB_LinkLayer #(

	parameter SYMBOL_WIDTH 	= 4,	//Typical config: 40 bit bus width for 7 series GTP or U+ GTY
									//(4x 8b10b symbols per block)
									//Note, on GTP this requires a 20 bit internal width and TXUSRCLK at 2x the rate
									//of TXUSRCLK2

	parameter BUS_ADDR_WIDTH = 3,	//24 bit address

	localparam DATA_WIDTH	= 8*SYMBOL_WIDTH,	//SERDES data width for 8-bit data portion
	localparam SYMBOL_BITS	= $clog2(SYMBOL_WIDTH+1)
) (
	//SERDES ports
	input wire						rx_clk,
	input wire[SYMBOL_WIDTH-1:0]	rx_kchar,
	input wire[DATA_WIDTH-1:0]		rx_data,
	input wire						rx_data_valid,

	input wire						tx_clk,
	output logic[SYMBOL_WIDTH_1:0]	tx_kchar		= 0,
	output logic[DATA_WIDTH-1:0]	tx_data			= 0,

	//Link layer data outputs (rx_clk domain) to transaction layer
	output logic					rx_ll_start		= 0,	//start of an APB transaction
	output logic					rx_ll_valid		= 0,	//true if data is valid
	output logic[SYMBOL_BITS-1:0]	rx_ll_nvalid	= 0,	//number of valid bytes in rx_ll_data
	output logic[DATA_WIDTH-1:0]	rx_ll_data		= 0,	//upper layer data (left aligned)
															//if rx_ll_start is set, the first word is a K character
	output logic					rx_ll_ll_commit	= 0,	//frame received and is valid
	output logic					rx_ll_drop		= 0		//frame received but checksum was bad, dropped
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX CRC block

	logic		rx_crc_reset	= 0;
	logic[2:0]	rx_crc_din_len	= 0;
	logic[31:0]	rx_crc_din		= 0;
	wire[7:0]	rx_crc_dout;

	CRC8_ATM_x32_variable rx_crc(
		.clk(rx_clk),
		.reset(rx_crc_reset),
		.din_len(rx_crc_din_len),
		.din(rx_crc_din),
		.crc_out(rx_crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive frames

	logic	rx_active	= 0;
	logic	link_up		= 0;

	always_ff @(posedge rx_clk) begin

		//In a frame? If so, look for EOF
		if(rx_active) begin
			//TODO: handle delayed CRC checks etc
		end

		//Not in a frame? Expect a control character at position 0, and nowhere else
		//(LLPs are all >2 bytes long so cannot start and end in same word)
		else if( (rx_kchar[SYMBOL_WIDTH-1:1] == 0) && (rx_kchar[0] == 1) ) begin
			//See what the control character is
		end

		//Link down?

	end

endmodule
