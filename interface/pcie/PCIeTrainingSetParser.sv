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

/**
	@brief Parse incoming training sets
 */
module PCIeTrainingSetParser(
	input wire			clk,
	input wire			rst_n,

	input wire[15:0]	rx_data,
	input wire[1:0]		rx_charisk,
	input wire[1:0]		rx_err,

	output logic		rx_ts1_valid		= 0,
	output logic		rx_ts2_valid		= 0,
	output logic		rx_ts_link_valid	= 0,
	output logic[4:0]	rx_ts_link			= 0,
	output logic		rx_ts_lane_valid	= 0,
	output logic[4:0]	rx_ts_lane			= 0,
	output logic[7:0]	rx_ts_num_fts		= 0,
	output logic		rx_ts_5g_supported	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Detect incoming training sets

	logic[1:0]	rx_ts_fillcount		= 0;

	enum logic[2:0]
	{
		TS_HEAD,
		TS_LANEFTS,
		TS_RATECTL,
		TS_FILLER,
		TS1_FILLER,
		TS2_FILLER
	} rx_ts_state = TS_HEAD;

	always_ff @(posedge clk or negedge rst_n) begin
		if(!rst_n) begin
			rx_ts_state		<= TS_HEAD;
		end

		else begin

			rx_ts1_valid	<= 0;
			rx_ts2_valid	<= 0;

			case(rx_ts_state)

				//Wait until we see the start of a training set
				//Should be K28.5 (0xbc) followed by either link number or K23.7 (0xf7)
				TS_HEAD: begin

					//First symbol is a comma
					if(rx_charisk[0] && (rx_data[7:0] == 8'hbc)) begin

						//K23.7 means no link number assigned yet
						if(rx_charisk[1] && (rx_data[15:8] == 8'hf7) ) begin
							rx_ts_link_valid	<= 0;
							rx_ts_link			<= 0;
							rx_ts_state			<= TS_LANEFTS;
						end

						//Otherwise should be a valid link number <= 31
						else if(!rx_charisk[1] && (rx_data[15:8] < 32) ) begin
							rx_ts_link_valid	<= 1;
							rx_ts_link			<= rx_data[12:8];
							rx_ts_state			<= TS_LANEFTS;
						end

						//anything else isn't a training set, ignore it

					end

				end	//TS_HEAD

				//Lane number and number of FTs
				TS_LANEFTS: begin

					//Number of FTs can be any data character
					rx_ts_num_fts			<= rx_data[15:8];

					//Lane number can be blank
					if(rx_charisk[0] && (rx_data[7:0] == 8'hf7) ) begin
						rx_ts_lane_valid	<= 0;
						rx_ts_lane			<= 0;
						rx_ts_state			<= TS_RATECTL;
					end

					//otherwise must be valid
					else if(!rx_charisk[0] && (rx_data[7:0] < 32) ) begin
						rx_ts_lane_valid	<= 1;
						rx_ts_lane			<= rx_data[4:0];
						rx_ts_state			<= TS_RATECTL;
					end

					//Bail if num FTs is a control character
					if(rx_charisk[1])
						rx_ts_state			<= TS_HEAD;

				end	//TS_LANEFTS

				//Data rate bitmask and training control
				TS_RATECTL: begin

					//Both should be D chars, bail if they're not
					if(rx_charisk)
						rx_ts_state			<= TS_HEAD;

					//check if 5 Gbps mode is advertised, ignore everything else
					else begin
						rx_ts_5g_supported	<= rx_data[2];
						rx_ts_state			<= TS_FILLER;
					end

				end //TS_RATECTL

				//First pair of filler characters
				TS_FILLER: begin

					rx_ts_fillcount			<= 0;

					if(rx_charisk)
						rx_ts_state			<= TS_HEAD;

					if(rx_data == 16'h4a4a)
						rx_ts_state			<= TS1_FILLER;
					else if(rx_data == 16'h4545)
						rx_ts_state			<= TS2_FILLER;
					else
						rx_ts_state			<= TS_HEAD;
				end //TS_FILLER

				//Additional filler characters (expect 4 pairs total in this state)
				TS1_FILLER: begin

					if( (rx_charisk == 0) && (rx_data == 16'h4a4a) ) begin
						rx_ts_fillcount		<= rx_ts_fillcount + 1;
						if(rx_ts_fillcount == 3) begin
							rx_ts1_valid	<= 1;
							rx_ts_state		<= TS_HEAD;
						end
					end

					//anything else, abort
					else
						rx_ts_state			<= TS_HEAD;

				end //TS1_FILLER

				TS2_FILLER: begin

					if( (rx_charisk == 0) && (rx_data == 16'h4545) ) begin
						rx_ts_fillcount		<= rx_ts_fillcount + 1;
						if(rx_ts_fillcount == 3) begin
							rx_ts2_valid	<= 1;
							rx_ts_state		<= TS_HEAD;
						end
					end

					//anything else, abort
					else
						rx_ts_state		<= TS_HEAD;

				end //TS2_FILLER

			endcase

			//If we have an error in either position abort the current training set
			if(rx_err) begin
				rx_ts1_valid	<= 0;
				rx_ts2_valid	<= 0;
				rx_ts_state		<= TS_HEAD;
			end

		end
	end


endmodule
