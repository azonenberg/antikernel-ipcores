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
	output GmiiBus		gmii_rx_bus	= {$bits(GmiiBus){1'b0}},

	//TX GMII interface. 125 MHz but need not be clk_125mhz domain
	input wire			gmii_tx_clk,
	input GmiiBus		gmii_tx_bus,

	//TX SERDES interface. 125 MHz.
	output wire			tx_clk,
	output logic		tx_data_is_ctl				= 0,
	output logic[7:0]	tx_data						= 0,
	output logic		tx_force_disparity_negative	= 0,
	input wire			tx_disparity_negative
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Forward the transmit clock

	assign	tx_clk = clk_125mhz;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Shift the RX data into the 125 MHz clock domain

	//TODO: buffer 4 code groups at a time, drop an idle set if the fifo is too full

	wire		rx_fifo_empty;

	//Let the FIFO fill up a bit before we start reading
	wire[5:0]	rx_fifo_rsize;
	logic		rx_fifo_rd			= 0;
	logic		fifo_initial_fill	= 0;
	always_ff @(posedge clk_125mhz) begin
		if(rx_fifo_empty)
			fifo_initial_fill	<= 0;
		if(rx_fifo_rsize > 16)
			fifo_initial_fill	<= 1;
	end
	always_comb begin
		rx_fifo_rd	<= fifo_initial_fill && !rx_fifo_empty;
	end

	logic		rx_fifo_rd_valid	= 0;
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
		.rd_size(rx_fifo_rsize),
		.rd_empty(rx_fifo_empty),
		.rd_underflow()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit mux (select autonegotiation or link-up data)

	logic		tx_aneg_data_is_ctl	= 0;
	logic[7:0]	tx_aneg_data		= 0;
	logic		tx_aneg_force_disparity_negative	= 0;

	logic		tx_link_data_is_ctl					= 0;
	logic[7:0]	tx_link_data						= 0;

	always_ff @(posedge clk_125mhz) begin

		if(link_up) begin
			tx_data_is_ctl				<= tx_link_data_is_ctl;
			tx_data						<= tx_link_data;
			tx_force_disparity_negative	<= 0;
		end

		else begin
			tx_data_is_ctl				<= tx_aneg_data_is_ctl;
			tx_data						<= tx_aneg_data;
			tx_force_disparity_negative	<= tx_aneg_force_disparity_negative;
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

		if(rx_fifo_rd_valid) begin

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
					rx_aneg_cfg[7:0]		<= rx_fdata;
					rx_aneg_state			<= RX_ANEG_REG_1;

					if(rx_fdata_is_ctl)
						rx_aneg_state		<= RX_ANEG_WAIT_FOR_C;
				end	//end RX_ANEG_REG_0

				//Read second half of config register
				RX_ANEG_REG_1: begin
					rx_aneg_state			<= RX_ANEG_WAIT_FOR_C;

					if(!rx_fdata_is_ctl) begin
						rx_aneg_cfg[15:8]	<= rx_fdata;
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
		ANEG_IDLE_DETECT	= 3'h6,
		ANEG_LINK_OK		= 3'h7
	} aneg_state = ANEG_ENABLE;

	logic[31:0] link_timer	= 0;

	logic[15:0]	tx_config_reg	= 0;

	//10 ms in 1000base-X mode. 2 us in SGMII mode (default for DP83867)
	wire[31:0]	link_timer_target = sgmii_mode ? 249 : 1249999;
	wire		link_timer_done = (link_timer == link_timer_target);

	logic[15:0]	rx_aneg_cfg_ff		= 0;
	logic[15:0]	rx_aneg_cfg_ff2		= 0;
	logic		rx_aneg_cfg_match	= 0;

	logic		idle_match			= 0;
	logic[1:0]	idle_count			= 0;
	logic		last_was_k285		= 0;

	always_ff @(posedge clk_125mhz) begin

		if(link_timer != 0)
			link_timer	<= link_timer + 1'h1;
		if(link_timer_done)
			link_timer	<= 0;

		//1000base-X mode is always gigabit
		if(!sgmii_mode)
			link_speed	<= LINK_SPEED_1000M;

		//Detect config matches
		if(rx_aneg_cfg_valid) begin
			idle_count			<= 0;
			idle_match			<= 0;

			rx_aneg_cfg_ff		<= rx_aneg_cfg;
			rx_aneg_cfg_ff2		<= rx_aneg_cfg_ff;
			rx_aneg_cfg_match	<= (rx_aneg_cfg_ff == rx_aneg_cfg) && (rx_aneg_cfg_ff2 == rx_aneg_cfg);
		end

		//Detect idle matches
		if(rx_fifo_rd_valid) begin
			last_was_k285	<= 0;

			//Look for K28.5 D16.2 (idle-2)
			if( (rx_fdata == 8'hbc) && rx_fdata_is_ctl)
				last_was_k285	<= 1;
			else if( (rx_fdata == 8'h50) && !rx_fdata_is_ctl && last_was_k285) begin
				idle_count <= idle_count + 1'h1;
				if(idle_count == 3)
					idle_match	<= 1;
			end

			//Not idle-matching as soon as anything else shows up
			else
				idle_match	<= 0;

		end

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
						1'b0,			//No next page
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

				if(rx_aneg_cfg_match && (rx_aneg_cfg != 0) ) begin
					rx_aneg_cfg_match	<= 0;
					rx_aneg_cfg_ff2		<= 0;
					rx_aneg_cfg_ff		<= 0;

					aneg_state				<= ANEG_ACK_DETECT;

					//Jump back to enable state if we get invalid configuration
					//(half duplex)
					if(sgmii_mode && !rx_aneg_cfg[12])
						aneg_state			<= ANEG_ENABLE;
					if(!sgmii_mode && !rx_aneg_cfg[5])
						aneg_state			<= ANEG_ENABLE;

				end

				//If we start getting idles, the link is up - maybe doesn't have negotiation enabled
				if(idle_match) begin
					link_up				<= 1;
					link_timer			<= 1;
					aneg_state			<= ANEG_LINK_OK;
				end

			end	//end ANEG_ABILITY_DETECT

			ANEG_ACK_DETECT: begin

				//Set outbound ACK bit
				tx_config_reg[14]	<= 1;

				if(rx_aneg_cfg_match && rx_aneg_cfg[14]) begin

					//save link speed in SGMII mode
					if(sgmii_mode) begin
						case(rx_aneg_cfg[11:10])
							0: link_speed <= LINK_SPEED_10M;
							1: link_speed <= LINK_SPEED_100M;
							2: link_speed <= LINK_SPEED_1000M;
						endcase
					end

					link_timer			<= 1;
					aneg_state			<= ANEG_COMPLETE_ACK;
				end

				//Reset if the other end resets
				if(rx_aneg_cfg_match && (rx_aneg_cfg == 0) )
					aneg_state			<= ANEG_ENABLE;

				//If we start getting idles, the link is up - maybe doesn't have negotiation enabled
				if(link_timer_done && idle_match) begin
					link_up				<= 1;
					link_timer			<= 1;
					aneg_state			<= ANEG_LINK_OK;
				end

			end	//end ANEG_ACK_DETECT

			ANEG_COMPLETE_ACK: begin

				//Reset if the other end resets
				if(rx_aneg_cfg_match && (rx_aneg_cfg == 0) )
					aneg_state			<= ANEG_ENABLE;

				if(link_timer_done) begin

					//All good? Start sending idles
					if(rx_aneg_cfg_match && rx_aneg_cfg[14]) begin
						link_timer			<= 1;
						aneg_state			<= ANEG_IDLE_DETECT;
					end

					//Something changed, bail
					else
						aneg_state			<= ANEG_ENABLE;

				end
			end	//end ANEG_COMPLETE_ACK

			ANEG_IDLE_DETECT: begin

				//Reset if the other end resets
				if(rx_aneg_cfg_match && (rx_aneg_cfg == 0) )
					aneg_state			<= ANEG_ENABLE;

				if(link_timer_done && idle_match) begin
					link_up				<= 1;
					link_timer			<= 1;
					aneg_state			<= ANEG_LINK_OK;
				end

			end	//end ANEG_IDLE_DETECT

			//Link is up!
			//TODO: drop on excessive code errors or loss of 8b/10b sync
			ANEG_LINK_OK: begin

				//If we get an idle, reset the link timer
				if(idle_match)
					link_timer			<= 1;

				//If link timer wraps without any idles, the link must have dropped.
				//If we get a /C/ ordered set, drop the link immediately because the other end is down.
				if(link_timer_done || rx_aneg_cfg_valid) begin
					link_up				<= 0;
					aneg_state			<= ANEG_ENABLE;
				end

			end	//end ANEG_LINK_OK

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX idle / data transmits

	logic	tx_idle_count	= 0;

	logic	tx_en_ff		= 0;
	logic	first_idle		= 0;

	always_ff @(posedge tx_clk) begin
		tx_idle_count						<= !tx_idle_count;

		tx_link_data_is_ctl					<= 0;

		tx_en_ff							<= gmii_tx_bus.en;

		//Starting a frame? Replace the first preamble byte with a SOF
		if(gmii_tx_bus.en && !tx_en_ff) begin
			tx_link_data					<= 8'hfb;
			tx_link_data_is_ctl				<= 1;
		end

		//Sending data? Forward it on as-is
		else if(gmii_tx_bus.en)
			tx_link_data					<= gmii_tx_bus.data;

		//End of frame? Send end symbol then begin sending idle ordered sets
		else if(!gmii_tx_bus.en && tx_en_ff) begin

			//make sure we start at correct phase
			tx_idle_count					<= 1;

			tx_link_data					<= 8'hfd;
			tx_link_data_is_ctl				<= 1;

			first_idle						<= 1;
		end

		//In between frames? Send I2 ordered set: K28.5 D16.2
		else if(tx_idle_count) begin
			tx_link_data						<= 8'hbc;
			tx_link_data_is_ctl					<= 1;
		end

		else begin

			tx_link_data						<= 8'h50;

			//Adjust disparity between frames if needed (send /I1/, D5.6, instead of D16.2)
			if(first_idle && !tx_disparity_negative)
				tx_link_data					<= 8'hC5;

			first_idle							<= 0;

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX autonegotiation transmit logic

	logic[2:0] tx_aneg_count = 0;
	always_ff @(posedge tx_clk) begin

		tx_aneg_count						<= tx_aneg_count + 1'h1;

		tx_aneg_data_is_ctl					<= 0;
		tx_aneg_force_disparity_negative	<= 0;

		if(aneg_state == ANEG_IDLE_DETECT) begin

			//I2 ordered set: K28.5 D16.2
			if(tx_aneg_count[0]) begin
				tx_aneg_data						<= 8'hbc;
				tx_aneg_data_is_ctl					<= 1;
				tx_aneg_force_disparity_negative	<= 1;
			end
			else
				tx_aneg_data						<= 8'h50;

		end

		else begin

			case(tx_aneg_count)

				//C1 ordered set: K28.5, D21.5, status
				0: begin
					tx_aneg_data		<= 8'hbc;
					tx_aneg_data_is_ctl	<= 1;
				end
				1: 	tx_aneg_data		<= 8'hb5;
				2:	tx_aneg_data		<= tx_config_reg[7:0];
				3:	tx_aneg_data		<= tx_config_reg[15:8];

				//C2 ordered set: K28.5, D2.2, status
				4: begin
					tx_aneg_data		<= 8'hbc;
					tx_aneg_data_is_ctl	<= 1;
				end
				5: 	tx_aneg_data		<= 8'h42;
				6:	tx_aneg_data		<= tx_config_reg[7:0];
				7:	tx_aneg_data		<= tx_config_reg[15:8];

			endcase

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX frame decoding

	always_ff @(posedge clk_125mhz) begin

		gmii_rx_bus.dvalid	<= 1;	//TODO: handle 10/100 mode
		gmii_rx_bus.er		<= 0;

		//Wait for start-of-frame character K27.7
		if(rx_fdata_is_ctl && (rx_fdata == 8'hfb) ) begin
			gmii_rx_bus.en		<= 1;
			gmii_rx_bus.data	<= 8'h55;
		end

		//If we see an idle, the frame is over
		else if(idle_match) begin
			gmii_rx_bus.en	<= 0;
			gmii_rx_bus.er	<= 1;
		end

		//Forward frame data
		else if(gmii_rx_bus.en) begin

			//Look for end-of-frame character K29.7
			if(rx_fdata_is_ctl && (rx_fdata == 8'hfd) )
				gmii_rx_bus.en		<= 0;

			//Look for error propagation character K30.7
			else if(rx_fdata_is_ctl && (rx_fdata == 8'hfe) )
				gmii_rx_bus.er		<= 1;

			else if(!rx_fdata_is_ctl)
				gmii_rx_bus.data	<= rx_fdata;

			//TODO: handle other control characters including error propagation

		end

	end

endmodule
