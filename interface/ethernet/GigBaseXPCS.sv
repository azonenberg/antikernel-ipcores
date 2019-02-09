`default_nettype none
`timescale 1ns/1ps

/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief	1000base-X / SGMII PCS

	No rate matching for now.
	The TX GMII interface must run in the clk_125mhz domain.
 */
module GigBaseXPCS(

	input wire			clk_125mhz,

	//1 = SGMII, 0 = 1000base-X
	input wire			sgmii_mode,

	//RX SERDES interface.
	//Typical usage: 156.25 MHz clock, but average of 125 MHz valid data rate.
	//May also be 125 MHz with rx_data_valid tied high.
	input wire			rx_clk,
	input wire			rx_data_valid,
	input wire			rx_data_is_ctl,
	input wire[7:0]		rx_data,

	//RX status signals
	output logic		link_up		= 0,
	output lspeed_t		link_speed	= LINK_SPEED_1000M,

	//RX GMII interface. Clock is always 125 MHz regardless of link speed.
	output GmiiBus		rx_gmii_bus,

	//TX SERDES interface. 125 MHz.
	output wire			tx_clk,
	output logic		tx_data_is_ctl	= 0,
	output logic[7:0]	tx_data	= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Forward the transmit clock

	assign	tx_clk = clk_125mhz;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Shift the RX data into the 125 MHz clock domain

	//TODO: buffer 4 code groups at a time, drop an idle set if the fifo is too full

	wire	rx_fifo_empty;
	wire	rx_fifo_rd = !rx_fifo_empty;

	logic	rx_fifo_rd_valid	= 0;
	always_ff @(posedge clk_125mhz) begin
		rx_fifo_rd_valid	<= rx_fifo_rd;
	end

	wire		rx_fdata_is_ctl;
	wire[7:0]	rx_fdata;

	CrossClockFifo #(
		.WIDTH(9),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) rx_fifo (
		.wr_clk(rx_clk),
		.wr_en(rx_data_valid),
		.wr_data({rx_data_is_ctl, rx_data}),
		.wr_size(),
		.wr_full(),
		.wr_overflow(),

		.rd_clk(clk_125mhz),
		.rd_en(rx_fifo_rd),
		.rd_data({rx_fdata_is_ctl, rx_fdata}),
		.rd_size(),
		.rd_empty(rx_fifo_empty),
		.rd_underflow()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit mux (select autonegotiation or link-up data)

	logic		tx_aneg_data_is_ctl	= 0;
	logic[7:0]	tx_aneg_data		= 0;

	always_ff @(posedge clk_125mhz) begin

		//TODO
		if(link_up) begin
			tx_data_is_ctl	<= 0;
			tx_data			<= 0;
		end

		else begin
			tx_data_is_ctl	<= tx_aneg_data_is_ctl;
			tx_data			<= tx_aneg_data;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX autonegotiation - parse ordered sets into config registers

	enum logic[1:0]
	{
		RX_ANEG_WAIT_FOR_C	= 0,
		RX_ANEG_C_HEADER	= 1,
		RX_ANEG_REG_0		= 2,
		RX_ANEG_REG_1		= 3
	} rx_aneg_state = RX_ANEG_WAIT_FOR_C;

	logic		rx_aneg_cfg_valid	= 0;
	logic[15:0]	rx_aneg_cfg	 	= 0;

	always_ff @(posedge clk_125mhz) begin

		rx_aneg_cfg_valid	<= 0;

		//1000base-X mode is always gigabit
		if(!sgmii_mode)
			link_speed	<= LINK_SPEED_1000M;

		if(!link_up && rx_fifo_rd_valid) begin

			case(rx_aneg_state)

				//Wait for a K28.5 character (denotes start of /C1/ or /C2/ ordered set)
				RX_ANEG_WAIT_FOR_C: begin
					if(rx_fdata_is_ctl && rx_fdata == 8'hbc)
						rx_aneg_state		<= RX_ANEG_C_HEADER;
				end	//end RX_ANEG_WAIT_FOR_C

				//Expect a D21.5 or D2.2 (for /C1/ or /C2/).
				//For now, accept either (don't require alternating)
				RX_ANEG_C_HEADER: begin
					rx_aneg_state			<= RX_ANEG_WAIT_FOR_C;
					if( ( (rx_fdata == 8'hb5) || (rx_fdata == 8'h42) ) && !rx_fdata_is_ctl )
						rx_aneg_state		<= RX_ANEG_REG_0;
				end	//end RX_ANEG_C_HEADER

				//Read first half of config register
				RX_ANEG_REG_0: begin
					rx_aneg_cfg[15:8]		<= rx_fdata;
					rx_aneg_state			<= RX_ANEG_REG_1;

					if(rx_fdata_is_ctl)
						rx_aneg_state		<= RX_ANEG_WAIT_FOR_C;
				end	//end RX_ANEG_REG_0

				//Read second half of config register
				RX_ANEG_REG_1: begin
					rx_aneg_state			<= RX_ANEG_WAIT_FOR_C;

					if(!rx_fdata_is_ctl) begin
						rx_aneg_cfg[7:0]	<= rx_fdata;
						rx_aneg_cfg_valid	<= 1;
					end
				end	//end RX_ANEG_REG_1

			endcase

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main autonegotiation state machine

	enum logic[2:0]
	{
		ANEG_ENABLE			= 3'h0,
		ANEG_RESTART		= 3'h1,
		ANEG_LINK_UP		= 3'h2,
		ANEG_ABILITY_DETECT	= 3'h3,
		ANEG_ACK_DETECT		= 3'h4,
		ANEG_COMPLETE_ACK	= 3'h5,
		ANEG_IDLE_DETECT	= 3'h6
	} aneg_state = ANEG_ENABLE;

	logic[31:0] link_timer	= 0;

	logic[15:0]	tx_config_reg	= 0;

	//1.6 ms in SGMII mode, 10 ms in 1000base-X mode
	wire[31:0]	link_timer_target = sgmii_mode ? 199999 : 1249999;

	wire		link_timer_done = (link_timer == link_timer_target);

	always_ff @(posedge clk_125mhz) begin

		if(link_timer != 0)
			link_timer	<= link_timer + 1'h1;
		if(link_timer_done)
			link_timer	<= 0;

		case(aneg_state)

			ANEG_ENABLE: begin
				tx_config_reg	<= 0;
				link_timer		<= 1;
				aneg_state		<= ANEG_RESTART;
			end	//end ANEG_ENABLE

			ANEG_RESTART: begin
				if(link_timer_done) begin
					aneg_state		<= ANEG_ABILITY_DETECT;
					tx_config_reg	<=
					{
						1'b0,			//Next page
						sgmii_mode,		//ACK in 1000base-X mode, constant 1 in SGMII mode
						2'b0,			//No remote fault
						3'b0,			//Reserved
						2'b0,			//No pause supported
						1'b0,			//No half duplex supported
						!sgmii_mode,	//Full duplex supported in 1000base-X mode, reserved in SGMII mode
						5'b0			//Reserved
					};
				end
			end	//end ANEG_RESTART

			ANEG_ABILITY_DETECT: begin
			end	//end ANEG_ABILITY_DETECT

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX autonegotiation transmit logic

	//Dummy state machine for now
	logic[2:0] tx_aneg_count = 0;
	always_ff @(posedge tx_clk) begin

		tx_aneg_count	<= tx_aneg_count + 1'h1;

		tx_aneg_data_is_ctl	<= 0;

		case(tx_aneg_count)

			//C1 ordered set: K28.5, D21.5, status
			0: begin
				tx_aneg_data		<= 8'hbc;
				tx_aneg_data_is_ctl	<= 1;
			end
			1: 	tx_aneg_data		<= 8'hb5;
			2:	tx_aneg_data		<= tx_config_reg[15:8];
			3:	tx_aneg_data		<= tx_config_reg[7:0];

			//C2 ordered set: K28.5, D2.2, status
			4: begin
				tx_aneg_data		<= 8'hbc;
				tx_aneg_data_is_ctl	<= 1;
			end
			5: 	tx_aneg_data		<= 8'h42;
			6:	tx_aneg_data		<= tx_config_reg[15:8];
			7:	tx_aneg_data		<= tx_config_reg[7:0];

		endcase

	end

endmodule
