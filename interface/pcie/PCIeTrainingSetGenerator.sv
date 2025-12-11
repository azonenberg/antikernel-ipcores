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
	@brief Generate outbound training sets
 */
module PCIeTrainingSetGenerator(
	input wire			clk,
	input wire			rst_n,

	output logic[15:0]	tx_data			= 0,
	output logic[1:0]	tx_charisk		= 0,

	input wire			tx_set_is_ts2,
	input wire			tx_ts_link_valid,
	input wire[4:0]		tx_ts_link,
	input wire			tx_ts_lane_valid,
	input wire[4:0]		tx_ts_lane,
	input wire[7:0]		tx_ts_num_fts,
	input wire			tx_ts_5g_supported,

	output logic		tx_ts_sent		= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Generate training sets

	enum logic[2:0]
	{
		TS_HEAD	= 0,
		TS_LANEFTS,
		TS_RATECTL,
		TS_FILLER,

		TS_SKIP_0,
		TS_SKIP_1
	} state = TS_HEAD;

	logic[2:0] count = 0;
	logic[6:0]	skip_count	= 0;

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			tx_data		<= 0;
			tx_charisk	<= 0;
			count		<= 0;
			state		<= TS_HEAD;
			tx_ts_sent	<= 0;
		end

		else begin

			tx_ts_sent	<= 0;

			case(state)

				//K28.5, link number
				TS_HEAD: begin

					tx_data[7:0]	<= 8'hbc;
					tx_charisk[0]	<= 1;

					//Valid link number, send it
					if(tx_ts_link_valid) begin
						tx_data[15:8]	<= {3'b0, tx_ts_link};
						tx_charisk[1]	<= 0;
					end

					//No link number, send PAD
					else begin
						tx_data[15:8]	<= 8'hf7;
						tx_charisk[1]	<= 1;
					end

					state			<= TS_LANEFTS;

					//Keep track of how many sets we sent between skips
					skip_count		<= skip_count + 1;

				end	//end TS_HEAD

				//Lane number, number of FTs
				TS_LANEFTS: begin

					//Valid lane number, send it
					if(tx_ts_lane_valid) begin
						tx_data[7:0]	<= {3'b0, tx_ts_lane};
						tx_charisk[0]	<= 0;
					end

					//No lane number, send PAD
					else begin
						tx_data[7:0]	<= 8'hf7;
						tx_charisk[0]	<= 1;
					end

					//Number of FTs
					tx_data[15:8]	<= tx_ts_num_fts;
					tx_charisk[1]	<= 0;

					state			<= TS_RATECTL;

				end //end TS_LANEFTS

				//Data rate and training control
				TS_RATECTL: begin
					tx_charisk		<= 2'b00;
					tx_data[7:0]	<= { 5'h0, tx_ts_5g_supported, 2'b10 };	//always 2.5G, 5G optional, nothing faster
					tx_data[15:8]	<= 0;	//ignore training control

					state			<= TS_FILLER;
					count			<= 0;
				end //end TS_RATECTL

				TS_FILLER: begin
					tx_charisk		<= 2'b0;
					if(tx_set_is_ts2)
						tx_data	<= 16'h4545;
					else
						tx_data	<= 16'h4a4a;

					count			<= count + 1;
					if(count >= 4) begin
						state		<= TS_HEAD;

						tx_ts_sent	<= 1;

						//Send skips every 1180 to 1538 symbol times
						//Training sets are 16 symbols long so this is every 73.75 to 96 TSes
						//Send every 80 to be in the middle
						if(skip_count >= 79) begin
							skip_count	<= 0;
							state		<= TS_SKIP_0;
						end
					end
				end	//end TS_FILLER

				//TODO: refactoring, move TSes to higher level of the stack

				TS_SKIP_0: begin
					tx_charisk		<= 2'b11;
					tx_data			<= 16'h1cbc;
					state			<= TS_SKIP_1;
				end	//end TS_SKIP_0

				TS_SKIP_1: begin
					tx_charisk		<= 2'b11;
					tx_data			<= 16'h1c1c;
					state			<= TS_HEAD;
				end	//end TS_SKIP_0

			endcase

		end
	end

endmodule
