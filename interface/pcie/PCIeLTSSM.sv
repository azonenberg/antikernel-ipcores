`timescale 1ns/1ps
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

module PCIeLTSSM(

	//Global clock and reset
	input wire			clk,
	input wire			rst_n,

	//RX data bus from PHY
	input wire[15:0]	rx_data,
	input wire[1:0]		rx_charisk,
	input wire[1:0]		rx_err,

	//Transmit data bus to output mux
	output wire[15:0]	tx_data,
	output wire[1:0]	tx_charisk,
	output wire			tx_skip_ack,
	input wire			tx_skip_req,
	input wire			tx_skip_done,

	//Status flags to rest of the stack
	output logic[4:0]	link_id	= 0,
	output logic[4:0]	lane_id	= 0,
	output logic		link_up
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Detect incoming training sets

	wire		rx_ts1_valid;
	wire		rx_ts2_valid;
	wire		rx_ts_link_valid;
	wire[4:0]	rx_ts_link;
	wire		rx_ts_lane_valid;
	wire[4:0]	rx_ts_lane;
	wire[7:0]	rx_ts_num_fts;
	wire		rx_ts_5g_supported;

	PCIeTrainingSetParser tsparser(
		.clk(clk),
		.rst_n(rst_n),

		.rx_data(rx_data),
		.rx_charisk(rx_charisk),
		.rx_err(rx_err),

		.rx_ts1_valid(rx_ts1_valid),
		.rx_ts2_valid(rx_ts2_valid),
		.rx_ts_link_valid(rx_ts_link_valid),
		.rx_ts_link(rx_ts_link),
		.rx_ts_lane_valid(rx_ts_lane_valid),
		.rx_ts_lane(rx_ts_lane),
		.rx_ts_num_fts(rx_ts_num_fts),
		.rx_ts_5g_supported(rx_ts_5g_supported)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Generate outgoing training sets

	logic		ts_type_is_ts2		= 0;
	logic		tx_ts_link_valid	= 0;
	logic[4:0]	tx_ts_link			= 0;
	logic		tx_ts_lane_valid	= 0;
	logic[4:0]	tx_ts_lane			= 0;

	wire		tx_ts_sent;

	PCIeTrainingSetGenerator tsgen(
		.clk(clk),
		.rst_n(rst_n),

		.tx_data(tx_data),
		.tx_charisk(tx_charisk),

		.tx_set_is_ts2(ts_type_is_ts2),
		.tx_ts_link_valid(tx_ts_link_valid),
		.tx_ts_link(tx_ts_link),
		.tx_ts_lane_valid(tx_ts_lane_valid),
		.tx_ts_lane(tx_ts_lane),
		.tx_ts_num_fts(8'd64),		//constant
		.tx_ts_5g_supported(1'b0),	//constant

		.tx_ts_sent(tx_ts_sent),
		.tx_skip_req(tx_skip_req),
		.tx_skip_ack(tx_skip_ack),
		.tx_skip_done(tx_skip_done)
	);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LTSSM states

	enum logic[3:0]
	{
		LTSSM_DETECT							= 0,
		LTSSM_POLLING_ACTIVE					= 1,
		LTSSM_POLLING_CONFIGURATION				= 2,
		LTSSM_CONFIGURATION_LINKWIDTH_START		= 3,
		LTSSM_CONFIGURATION_LINKWIDTH_ACCEPT	= 4,
		LTSSM_CONFIGURATION_LANENUM_WAIT		= 5,
		LTSSM_CONFIGURATION_LANENUM_ACCEPT		= 6,
		LTSSM_CONFIGURATION_COMPLETE			= 7,
		LTSSM_CONFIGURATION_IDLE				= 8,
		LTSSM_L0								= 9
	} ltssm_state	= LTSSM_DETECT;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The LTSSM

	logic[10:0]	tsSentCount	= 0;
	logic[3:0]	tsRecvCount = 0;

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			ltssm_state	<= LTSSM_DETECT;
			link_id		<= 0;
			lane_id		<= 0;
			tsSentCount	<= 0;
			tsRecvCount	<= 0;
			ts_type_is_ts2		<= 0;
			tx_ts_link_valid	<= 0;
			tx_ts_link			<= 0;
			tx_ts_lane_valid	<= 0;
			tx_ts_lane			<= 0;
		end

		else begin

			case (ltssm_state)

				//Jump immediately out of this state because we are targeting hard-wired applications.
				//A receiver will *always* be present.
				LTSSM_DETECT: begin
					ltssm_state			<= LTSSM_POLLING_ACTIVE;

					//Send TS1s with invalid lane/link numbers
					ts_type_is_ts2		<= 0;
					tx_ts_link_valid	<= 0;
					tx_ts_link			<= 0;
					tx_ts_lane_valid	<= 0;
					tx_ts_lane			<= 0;

					tsSentCount			<= 0;
					tsRecvCount			<= 0;
				end //LTSSM_DETECT

				//Advertise everything
				LTSSM_POLLING_ACTIVE: begin

					//We don't implement compliance receive or loopback so skip those conditions

					//Count how many valid TS2s we've seen, saturating at 8
					if(rx_ts2_valid && !rx_ts_link_valid && !rx_ts_lane_valid) begin
						if(!tsRecvCount[3])
							tsRecvCount	<= tsRecvCount + 4'h1;
					end

					//Count how many TS1s we've sent, saturating at 1024
					if(tx_ts_sent) begin
						if(!tsSentCount[10])
							tsSentCount	<= tsSentCount + 1;
					end

					//If we see eight TS2s with invalid lane/link number, and we've sent 1024 TS1s,
					//move to Polling.Configuration
					if(tsSentCount[10] && tsRecvCount[3]) begin

						//In this state, we want to send TS2s
						ts_type_is_ts2	<= 1;
						tsRecvCount		<= 0;
						tsSentCount		<= 0;
						ltssm_state		<= LTSSM_POLLING_CONFIGURATION;
					end

					//TODO: also move to Polling.Configuration after 24ms timeout?

				end	//end LTSSM_POLLING_ACTIVE

				//Same as before but sending TS2s
				LTSSM_POLLING_CONFIGURATION: begin

					//Count how many valid TS2s we've seen, saturating at 8
					if(rx_ts2_valid && !rx_ts_link_valid && !rx_ts_lane_valid) begin
						if(!tsRecvCount[3])
							tsRecvCount	<= tsRecvCount + 4'h1;
					end

					//Wait until we've seen at least one TS2, then send 16 more TS2s, then move to Configuration
					if(tx_ts_sent && (tsRecvCount != 0)) begin
						tsSentCount	<= tsSentCount + 1;

						if(tsSentCount[4]) begin
							ts_type_is_ts2	<= 0;
							tsRecvCount		<= 0;
							tsSentCount		<= 0;
							ltssm_state		<= LTSSM_CONFIGURATION_LINKWIDTH_START;
						end
					end

					//TODO: fall back to Detect after 48ms timeout

				end	//end LTSSM_POLLING_CONFIGURATION

				//Figure out how many lanes we want to use. This is easy, we only support one lol
				LTSSM_CONFIGURATION_LINKWIDTH_START: begin

					//If we get two consecutive TS1s with valid link numbers, save the link number
					if(rx_ts1_valid && rx_ts_link_valid && !rx_ts_lane_valid) begin

						//If this is the first one, save the link number
						if(tsRecvCount == 0) begin
							link_id		<= rx_ts_link;
							tsRecvCount	<= tsRecvCount + 4'h1;
						end

						//If this is the second, and they match, move to LINKWIDTH_ACCEPT
						else if(link_id == rx_ts_link) begin
							tx_ts_link			<= link_id;
							tx_ts_link_valid	<= 1;
							ltssm_state			<= LTSSM_CONFIGURATION_LINKWIDTH_ACCEPT;
						end

					end

				end	//end LTSSM_CONFIGURATION_LINKWIDTH_START

				LTSSM_CONFIGURATION_LINKWIDTH_ACCEPT: begin

					//If we get two consecutive TS1s with valid lane numbers, save the link number
					if(rx_ts1_valid && rx_ts_link_valid && rx_ts_lane_valid) begin

						//If this is the first one, save the lane number
						if(tsRecvCount == 0) begin
							lane_id		<= rx_ts_lane;
							tsRecvCount	<= tsRecvCount + 4'h1;
						end

						//If this is the second, and they match, move to LINKWIDTH_ACCEPT
						else if(lane_id == rx_ts_lane) begin
							tx_ts_lane			<= lane_id;
							tx_ts_lane_valid	<= 1;
							tsRecvCount			<= 0;
							ltssm_state			<= LTSSM_CONFIGURATION_LANENUM_WAIT;
						end

					end

				end	//end LTSSM_CONFIGURATION_LINKWIDTH_ACCEPT

				LTSSM_CONFIGURATION_LANENUM_WAIT: begin

					//If we get two consecutive TS2s move to Lanenum.Accept
					//TODO: validate lane numbers match us
					if(rx_ts2_valid && rx_ts_link_valid && rx_ts_lane_valid) begin
						tsRecvCount		<= tsRecvCount + 1;

						if(tsRecvCount == 2) begin
							tsRecvCount			<= 0;
							ltssm_state			<= LTSSM_CONFIGURATION_LANENUM_ACCEPT;
						end
					end

				end //end LTSSM_CONFIGURATION_LANENUM_WAIT

				LTSSM_CONFIGURATION_LANENUM_ACCEPT: begin

					//If we get two consecutive TS2s move to Lanenum.Accept
					//TODO: validate lane numbers match us
					if(rx_ts2_valid && rx_ts_link_valid && rx_ts_lane_valid) begin
						tsRecvCount		<= tsRecvCount + 1;

						if(tsRecvCount == 2) begin
							tsRecvCount			<= 0;
							ts_type_is_ts2		<= 1;
							ltssm_state			<= LTSSM_CONFIGURATION_COMPLETE;
						end
					end

				end //end LTSSM_CONFIGURATION_LANENUM_ACCEPT

				LTSSM_CONFIGURATION_COMPLETE: begin

					//If we get eight good TS2s move to Configuration.Idle
					//TODO: validate matching link/lane numbers
					if(rx_ts2_valid && rx_ts_link_valid && rx_ts_lane_valid) begin
						tsRecvCount		<= tsRecvCount + 1;

						if(tsRecvCount == 8) begin
							tsRecvCount			<= 0;
							ts_type_is_ts2		<= 1;
							ltssm_state			<= LTSSM_CONFIGURATION_IDLE;
						end
					end

				end	//end LTSSM_CONFIGURATION_COMPLETE

				LTSSM_CONFIGURATION_IDLE: begin

					//Cleanly switch to link-up state as we end a training set
					if(tx_ts_sent)
						link_up					<= 1;

					//TODO: search for incoming idle symbols
				end	//end LTSSM_CONFIGURATION_IDLE

				LTSSM_L0: begin
					//TODO
				end //end LTSSM_L0

			endcase

		end

	end

endmodule
