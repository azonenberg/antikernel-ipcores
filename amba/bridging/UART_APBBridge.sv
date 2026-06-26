`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg                                                                          *
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
	@brief A bridge from UART to APBv5, tested on Efinix Titanium but should be portable.

	The bridge maps to a 32-bit APB segment mapped starting at 0000_0000.

	Two different protocols, with non-overlapping opcodes, are supported:
	* A low overhead binary protocol meant for automated use (opcodes restricted to high characters > 0x80)
	* A human readable, line oriented ASCII protocol meant for use as a CLI

	Binary protocol:
	* 0x80							Reset the APB segment
	* 0x81 a0 a1 a2 a3 d0 d1 d2 d3	32-bit write of 0xd3d2d1d0 to 0xa3a2a1a0
	* 0xff							No-op

	Text protocol:
 */
module UART_APBBridge(

	//Main system clock used for both the UART IP and as APB PCLK
	input wire			clk,
	input wire			rst_n,

	//Baud rate divisor
	input wire[15:0]	baud_div,

	//The UART pins
	input wire			uart_rx,
	output wire			uart_tx,

	//Internal-facing APB interface
	APB.requester		apb
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Validate width of the bus

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();
	if(apb.ADDR_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The UART

	logic[7:0]	tx_data = 0;
	logic		tx_en = 0;
	wire		tx_done;

	wire		rx_en;
	wire[7:0]	rx_data;

	UART uart(
		.clk(clk),
		.clkdiv(baud_div),

		.rx(uart_rx),
		.rxactive(),
		.rx_data(rx_data),
		.rx_en(rx_en),

		.tx(uart_tx),
		.tx_data(tx_data),
		.tx_en(tx_en),
		.txactive(),
		.tx_done(tx_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB reset management

	assign	apb.pclk		= clk;

	logic	rst_n_int		= 0;
	assign	apb.preset_n	= rst_n_int;
	logic[3:0]	rst_count	= 1;

	logic	soft_reset		= 0;

	always_ff @(posedge apb.pclk or negedge rst_n) begin

		//External reset input
		if(!rst_n) begin
			rst_n_int		<= 0;
			rst_count		<= 1;
		end

		else begin

			//Extend internal reset
			if(!rst_n_int) begin
				rst_count	<= rst_count + 4'h1;
				if(rst_count == 0)
					rst_n_int	<= 1;
			end

			//Soft reset
			if(soft_reset) begin
				rst_n_int	<= 0;
				rst_count	<= 1;
			end

			/*
			//If bus gets stuck, reset it
			if(stuck_release) begin
				rst_n_int	<= 0;
				rst_count	<= 1;
			end
			*/
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused signals

	assign apb.pprot 	= 0;
	assign apb.pwakeup 	= 0;
	assign apb.pauser	= 0;
	assign apb.pwuser	= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	enum logic[7:0]
	{
		STATE_IDLE 			= 8'h00,

		STATE_WRITE_A0		= 8'h01,
		STATE_WRITE_A1		= 8'h02,
		STATE_WRITE_A2		= 8'h03,
		STATE_WRITE_A3		= 8'h04,
		STATE_WRITE_D0		= 8'h05,
		STATE_WRITE_D1		= 8'h06,
		STATE_WRITE_D2		= 8'h07,
		STATE_WRITE_D3		= 8'h08,

		STATE_WRITE_ENABLE	= 8'h09,
		STATE_WRITE_WAIT	= 8'h0a

	} state = STATE_IDLE;

	always_ff @(posedge apb.pclk or negedge rst_n) begin

		if(!rst_n) begin
			tx_data		<= 0;
			tx_en		<= 0;
			soft_reset	<= 0;
			apb.penable	<= 0;
			apb.psel	<= 0;
			apb.paddr	<= 0;
			apb.pwdata	<= 0;
			apb.pwrite	<= 0;
			apb.pstrb	<= 0;
			state		<= STATE_IDLE;
		end

		else begin

			soft_reset	<= 0;

			case(state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// IDLE - wait for incoming commands

				STATE_IDLE: begin

					if(rx_en) begin
						case(rx_data)

							//Reset the APB
							8'h80: begin
								soft_reset	<= 1;
							end

							//Write path
							8'h81: begin
								state		<= STATE_WRITE_A0;
							end

							//Nop
							8'hff: begin
							end

						endcase
					end

				end //STATE_IDLE

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Write path

				STATE_WRITE_A0: begin
					if(rx_en) begin
						apb.paddr[7:0]		<= rx_data;
						state				<= STATE_WRITE_A1;
					end
				end //STATE_WRITE_A0

				STATE_WRITE_A1: begin
					if(rx_en) begin
						apb.paddr[15:8]		<= rx_data;
						state				<= STATE_WRITE_A2;
					end
				end //STATE_WRITE_A1

				STATE_WRITE_A2: begin
					if(rx_en) begin
						apb.paddr[23:15]	<= rx_data;
						state				<= STATE_WRITE_A3;
					end
				end //STATE_WRITE_A2

				STATE_WRITE_A3: begin
					if(rx_en) begin
						apb.paddr[31:24]	<= rx_data;
						state				<= STATE_WRITE_D0;
					end
				end //STATE_WRITE_A3

				STATE_WRITE_D0: begin
					if(rx_en) begin
						apb.pwdata[7:0]		<= rx_data;
						state				<= STATE_WRITE_D1;
					end
				end //STATE_WRITE_D0

				STATE_WRITE_D1: begin
					if(rx_en) begin
						apb.pwdata[15:8]	<= rx_data;
						state				<= STATE_WRITE_D2;
					end
				end //STATE_WRITE_D1

				STATE_WRITE_D2: begin
					if(rx_en) begin
						apb.pwdata[23:15]	<= rx_data;
						state				<= STATE_WRITE_D3;
					end
				end //STATE_WRITE_D2

				STATE_WRITE_D3: begin
					if(rx_en) begin
						apb.pwdata[31:24]	<= rx_data;
						apb.pstrb			<= 4'b1111;
						apb.pwrite			<= 1;
						apb.psel			<= 1;
						state				<= STATE_WRITE_ENABLE;
					end
				end //STATE_WRITE_D3

				STATE_WRITE_ENABLE: begin
					apb.penable		<= 1;
					state			<= STATE_WRITE_WAIT;
				end //STATE_WRITE_ENABLE

				STATE_WRITE_WAIT: begin
					if(apb.pready) begin
						apb.penable	<= 0;
						apb.pwrite	<= 0;
						apb.paddr	<= 0;
						apb.pwdata	<= 0;
						apb.pstrb	<= 0;
						apb.psel	<= 0;
						state		<= STATE_IDLE;
					end
				end //STATE_WRITE_WAIT

			endcase

		end
	end

endmodule
