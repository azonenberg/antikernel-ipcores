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

	logic		uart_tx_en		= 0;
	logic[7:0]	uart_tx_data	= 0;
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

	logic								trig_armed			= 0;
	logic								trig_force			= 0;

	ila_status_t						status;

	//default to no matching on comparators
	initial begin
		for(integer i=0; i<CHANNELS; i++) begin
			compare_mode[i]	= COMPARE_NEVER;
			compare_target[i] = {MAX_WIDTH{1'b0}};
		end
	end

	logic					symtab_rd_en	= 0;
	logic[CHANNEL_BITS-1:0]	cur_channel		= 0;
	wire[NAME_BITS-1:0]		symtab_rd_data;

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
		.trig_out(trig_out),
		.trig_armed(trig_armed),
		.trig_force(trig_force),
		.status(status),

		.symtab_rd_en(symtab_rd_en),
		.symtab_rd_addr(cur_channel),
		.symtab_rd_data(symtab_rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// UART protocol logic

	//All UART packets are a single byte opcode followed by data.
	//All multi-byte fields are big endian ordering.
	typedef enum logic[7:0]
	{
		CMD_NOP					= 8'h00,	//Do nothing
		CMD_SET_MATCH_ALL		= 8'h01,	//1 arg byte: "match all" flag
											//Bit 0 high = trigger AND
											//Bit 0 low = trigger OR
											//Other bits ignored
		CMD_SET_TRIG_OFFSET		= 8'h02,	//2 arg bytes: trigger offset in capture window
		CMD_SET_TRIG_MODE		= 8'h03,	//2 arg bytes
											//Byte 0: channel index
											//Byte 1: trigger mode
		CMD_SET_COMPARE_TARGET	= 8'h04,	//Byte 0: channel index
											//Byte 1...N: trigger compare target
		CMD_ARM					= 8'h05,	//Arm the trigger
		CMD_STOP				= 8'h06,	//Stop the trigger
		CMD_FORCE				= 8'h07,	//Force trigger
		CMD_GET_STATUS			= 8'h08,	//Get the current LA status
		CMD_GET_NAME_LEN		= 8'h09,	//Get the length of a name table entry (in bytes)
		CMD_GET_CHANNEL_COUNT	= 8'h0a,	//Get the number of channels
		CMD_GET_NAME			= 8'h0b,	//Get a single entry of the name table
											//1 arg byte: channel number
		CMD_GET_WIDTH			= 8'h0c		//Get the width of a channel
											//1 arg byte: channel number
	} cmd_t;

	enum logic[3:0]
	{
		STATE_IDLE				= 4'h00,
		STATE_MATCH_ALL			= 4'h01,
		STATE_TRIG_OFFSET		= 4'h02,
		STATE_TRIG_MODE			= 4'h03,
		STATE_COMPARE_TARGET_0	= 4'h04,
		STATE_COMPARE_TARGET_1	= 4'h05,
		STATE_TX_HOLD			= 4'h06,
		STATE_GET_NAME_0		= 4'h07,
		STATE_GET_NAME_1		= 4'h08,
		STATE_GET_NAME_2		= 4'h09,
		STATE_GET_WIDTH			= 4'h0a
	} state;

	logic[7:0]				count	= 0;

	always_ff @(posedge clk) begin

		trig_force		<= 0;
		uart_tx_en		<= 0;
		symtab_rd_en	<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for traffic to show up

			STATE_IDLE: begin

				count	<= 0;

				if(uart_rx_en) begin

					case(uart_rx_data)

						CMD_NOP: begin
						end	//end CMD_NOP

						CMD_SET_MATCH_ALL:		state	<= STATE_MATCH_ALL;
						CMD_SET_TRIG_OFFSET:	state	<= STATE_TRIG_OFFSET;
						CMD_SET_TRIG_MODE:		state	<= STATE_TRIG_MODE;
						CMD_SET_COMPARE_TARGET:	state	<= STATE_COMPARE_TARGET_0;
						CMD_ARM:				trig_armed	<= 1;
						CMD_STOP:				trig_armed	<= 0;
						CMD_FORCE:				trig_force	<= 1;

						CMD_GET_STATUS: begin
							uart_tx_en			<= 1;
							uart_tx_data		<= status;
							state				<= STATE_TX_HOLD;
						end

						CMD_GET_NAME_LEN: begin
							uart_tx_en			<= 1;
							uart_tx_data		<= NAME_LEN;
							state				<= STATE_TX_HOLD;
						end

						CMD_GET_CHANNEL_COUNT: begin
							uart_tx_en			<= 1;
							uart_tx_data		<= CHANNELS;
							state				<= STATE_TX_HOLD;
						end

						CMD_GET_NAME:			state	<= STATE_GET_NAME_0;
						CMD_GET_WIDTH:			state	<= STATE_GET_WIDTH;

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
					count	<= count + 1'h1;
					if(count == 0)
						trig_offset[ADDR_BITS-1:8]	<= uart_rx_data;
					else begin
						trig_offset[7:0]			<= uart_rx_data;
						state						<= STATE_IDLE;
					end
				end
			end	//end STATE_TRIG_OFFSET

			STATE_TRIG_MODE: begin
				if(uart_rx_en) begin
					count	<= count + 1'h1;

					if(count == 0)
						cur_channel					<= uart_rx_data;
					else begin
						compare_mode[cur_channel]	<= ila_compare_t'(uart_rx_data);
						state						<= STATE_IDLE;
					end

				end
			end	//end STATE_TRIG_MODE

			STATE_COMPARE_TARGET_0: begin
				if(uart_rx_en) begin
					cur_channel						<= uart_rx_data;
					count							<= COL_BYTES-1;
					state							<= STATE_COMPARE_TARGET_1;
				end
			end	//end STATE_COMPARE_TARGET_0

			STATE_COMPARE_TARGET_1: begin
				if(uart_rx_en) begin
					count						<= count - 1;

					for(integer i=0; i<COL_BYTES; i++) begin
						if(count == i) begin
							if(i == COL_BYTES-1)
								compare_target[cur_channel][MAX_WIDTH-1 : (COL_BYTES-1)*8]	<= uart_rx_data;
							else
								compare_target[cur_channel][i*8 +: 8]	<= uart_rx_data;
						end
					end

					if(count == 0)
						state						<= STATE_IDLE;
				end
			end	//end STATE_COMPARE_TARGET_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Readback

			STATE_TX_HOLD: begin
				if(uart_tx_done)
					state							<= STATE_IDLE;
			end	//end STATE_TX_HOLD

			STATE_GET_NAME_0: begin
				if(uart_rx_en) begin
					cur_channel						<= uart_rx_data;
					symtab_rd_en					<= 1;
					count							<= 0;
					state							<= STATE_GET_NAME_1;
				end
			end	//end STATE_GET_NAME_0

			STATE_GET_NAME_1: begin
				state								<= STATE_GET_NAME_2;
			end	//end STATE_GET_NAME_1

			STATE_GET_NAME_2: begin
				if( (count == 0) || (uart_tx_done) ) begin

					uart_tx_en		<= 1;
					uart_tx_data	<= symtab_rd_data[count*8 +: 8];
					count			<= count + 1;

					if(count+1 == NAME_LEN)
						state	<= STATE_IDLE;
				end
			end	//end STATE_GET_NAME_1

			STATE_GET_WIDTH: begin
				if(uart_rx_en) begin
					uart_tx_en						<= 1;
					for(integer i=0; i<CHANNELS; i=i+1) begin
						if(uart_rx_data == i)
							uart_tx_data			<= WIDTHS[i];
					end
					state							<= STATE_TX_HOLD;
				end
			end

		endcase

	end

endmodule
