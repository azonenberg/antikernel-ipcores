`default_nettype none
`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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
	@module
	@author Andrew D. Zonenberg
	@brief	UART protocol wrapper for the ILA
 */
module InternalLogicAnalyzer_UARTWrapper #(
	parameter integer			CHANNELS 				= 1,					//Number of channels
	parameter integer			WIDTHS[CHANNELS-1:0] 	= '{0},					//Width of each channel
	parameter integer			NAME_LEN				= 16,					//Maximum length of a name, in characters
	localparam integer			NAME_BITS				= NAME_LEN*8,			//Maximum length of a name, in bits
	parameter[NAME_BITS-1:0]	NAMES[CHANNELS-1:0],							//Display name of each channel
																				//(truncated to X chars)
	parameter integer			MAX_WIDTH				= 1024,					//Maximum legal width of any channel
	parameter integer 			DEPTH					= 1024,					//Number of samples to capture
	localparam integer			ADDR_BITS				= $clog2(DEPTH),		//Number of bits in a pointer
	localparam integer			CHANNEL_BITS			= $clog2(CHANNELS),		//Number of bits in a channel index
	localparam integer			COL_BYTES				= $ceil(MAX_WIDTH/8.0)	//Number of bits in a column index
)(
	input wire								clk,				//Capture clock
	input wire[CHANNELS-1:0][MAX_WIDTH-1:0]	probe_in,			//Probe signals

	input wire[15:0]						clkdiv,
	input wire								uart_rxd,
	output wire								uart_txd,

	input wire								trig_in,
	output wire								trig_out
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sanity check parameters

	initial begin
		if(CHANNELS > 256)
			$fatal(0, "Channel count (%d) must be <256)", CHANNELS);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The UART

	wire		uart_rx_en;
	wire[7:0]	uart_rx_data;

	wire		uart_tx_en;
	wire[7:0]	uart_tx_data;
	wire		uart_tx_done;

	UART uart(
		.clk(clk),
		.clkdiv(clkdiv),
		.rx(uart_rxd),
		.rxactive(),
		.rx_data(uart_rx_data),
		.rx_en(uart_rx_en),
		.tx(uart_txd),
		.tx_data(uart_tx_data),
		.tx_en(uart_tx_en),
		.txactive(),
		.tx_done(uart_tx_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The ILA

	`include "InternalLogicAnalyzer_types.svh"

	logic								use_ext_trig		= 0;
	logic								compare_match_all	= 0;
	logic[ADDR_BITS-1:0]				trig_offset			= 0;
	ila_compare_t[CHANNELS-1:0]			compare_mode;
	logic[CHANNELS-1:0][MAX_WIDTH-1:0]	compare_target;

	//default to no matching on comparators
	initial begin
		for(integer i=0; i<CHANNELS; i++) begin
			compare_mode[i]	= COMPARE_NEVER;
			compare_target[i] = {MAX_WIDTH{1'b0}};
		end
	end

	InternalLogicAnalyzer #(
		.CHANNELS(CHANNELS),
		.WIDTHS(WIDTHS),
		.NAME_LEN(NAME_LEN),
		.NAMES(NAMES),
		.MAX_WIDTH(MAX_WIDTH),
		.DEPTH(DEPTH)
	) la (
		.clk(clk),
		.probe_in(probe_in),
		.compare_target(compare_target),
		.compare_mode(compare_mode),
		.compare_match_all(compare_match_all),
		.use_ext_trig(use_ext_trig),
		.trig_offset(trig_offset),
		.trig_in(trig_in),
		.trig_out(trig_out)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// UART protocol logic

	//All UART packets are a single byte opcode followed by data.
	//All multi-byte fields are big endian ordering.
	typedef enum logic[7:0]
	{
		CMD_NOP					= 0,	//Do nothing
		CMD_SET_MATCH_ALL		= 1,	//1 arg byte: "match all" flag
										//Bit 0 high = trigger AND
										//Bit 0 low = trigger OR
										//Other bits ignored
		CMD_SET_TRIG_OFFSET		= 2,	//2 arg bytes: trigger offset in capture window
		CMD_SET_TRIG_MODE		= 3,	//2 arg bytes
										//Byte 0: channel index
										//Byte 1: trigger mode
		CMD_SET_COMPARE_TARGET	= 4		//Byte 0: channel index
										//Byte 1...N: trigger compare target
	} cmd_t;

	enum logic[3:0]
	{
		STATE_IDLE				= 0,
		STATE_MATCH_ALL			= 1,
		STATE_TRIG_OFFSET		= 2,
		STATE_TRIG_MODE			= 3,
		STATE_COMPARE_TARGET_0	= 4,
		STATE_COMPARE_TARGET_1	= 5
	} state;

	logic[7:0]				rx_count	= 0;
	logic[CHANNEL_BITS-1:0]	cur_channel	= 0;

	always_ff @(posedge clk) begin

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for traffic to show up

			STATE_IDLE: begin

				rx_count	<= 0;

				if(uart_rx_en) begin

					case(uart_rx_data)

						CMD_NOP: begin
						end	//end CMD_NOP

						CMD_SET_MATCH_ALL:		state	<= STATE_MATCH_ALL;
						CMD_SET_TRIG_OFFSET:	state	<= STATE_TRIG_OFFSET;
						CMD_SET_TRIG_MODE:		state	<= STATE_TRIG_MODE;
						CMD_SET_COMPARE_TARGET:	state	<= STATE_COMPARE_TARGET_0;

						default: begin
						end	//default

					endcase

				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// RX configuration

			STATE_MATCH_ALL: begin
				if(uart_rx_en) begin
					compare_match_all	<= uart_rx_data[0];
					state				<= STATE_IDLE;
				end
			end	//end STATE_MATCH_ALL

			STATE_TRIG_OFFSET: begin
				if(uart_rx_en) begin
					rx_count	<= rx_count + 1'h1;
					if(rx_count == 0)
						trig_offset[ADDR_BITS-1:8]	<= uart_rx_data;
					else begin
						trig_offset[7:0]			<= uart_rx_data;
						state						<= STATE_IDLE;
					end
				end
			end	//end STATE_TRIG_OFFSET

			STATE_TRIG_MODE: begin
				if(uart_rx_en) begin
					rx_count	<= rx_count + 1'h1;

					if(rx_count == 0)
						cur_channel					<= uart_rx_data;
					else begin
						compare_mode[cur_channel]	<= ila_compare_t'(uart_rx_data);
						state						<= STATE_IDLE;
					end

				end
			end	//end STATE_TRIG_MODE

			STATE_COMPARE_TARGET_0: begin
				if(uart_rx_en) begin
					cur_channel					<= uart_rx_data;
					rx_count					<= COL_BYTES-1;
					state						<= STATE_COMPARE_TARGET_1;
				end
			end	//end STATE_COMPARE_TARGET_0

			STATE_COMPARE_TARGET_1: begin
				if(uart_rx_en) begin
					rx_count						<= rx_count - 1;

					for(integer i=0; i<COL_BYTES; i++) begin
						if(rx_count == i) begin
							if(i == COL_BYTES-1)
								compare_target[cur_channel][MAX_WIDTH-1 : (COL_BYTES-1)*8]	<= uart_rx_data;
							else
								compare_target[cur_channel][i*8 +: 8]	<= uart_rx_data;
						end
					end

					if(rx_count == 0)
						state					<= STATE_IDLE;
				end
			end	//end STATE_COMPARE_TARGET_1

		endcase

	end

endmodule
