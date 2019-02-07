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

/**
	@file
	@author Andrew D. Zonenberg
	@brief 8B/10B line code receiver and comma aligner

	TODO: consider splitting decoder and comma aligner into separate blocks?
 */
module Decode8b10b(
	input wire			clk,

	input wire			codeword_valid,
	input wire[9:0]		codeword_in,

	output logic		data_valid	= 0,
	output logic[7:0]	data		= 0,
	output logic		data_is_ctl	= 0,

	output logic		disparity_err	= 0,
	output logic		symbol_err		= 0,
	output logic		locked			= 0,
	output logic		bitslip			= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Comma detector

	logic[9:0]	codeword_in_ff	= 0;
	wire[19:0]	comma_window	= {codeword_in_ff, codeword_in };

	logic		comma_found;
	logic[4:0]	comma_found_pos;
	logic		comma_found_polarity;

	integer	i;


	always_ff @(posedge clk) begin

		comma_found	<= 0;

		if(codeword_valid) begin
			codeword_in_ff	<= codeword_in;

			for(i=0; i<10; i=i+1) begin

				//Positive comma
				if(comma_window[i +: 5] == 5'b11111) begin
					comma_found				<= 1;
					comma_found_pos			<= i;
					comma_found_polarity	<= 1;
				end

				//Negative comma
				if(comma_window[i +: 5] == 5'b00000) begin
					comma_found				<= 1;
					comma_found_pos			<= i;
					comma_found_polarity	<= 0;
				end

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Comma aligner

	logic[7:0]	comma_count	= 0;
	logic[7:0]	bad_commas	= 0;

	logic		codeword_valid_ff	= 0;

	always_ff @(posedge clk) begin

		bitslip				<= 0;

		codeword_valid_ff	<= codeword_valid;

		if(codeword_valid_ff && comma_found) begin

			comma_count	<= comma_count + 1'h1;

			//All three legal 8b/10b comma characters have the comma at bits 7:3.
			//If we see one anywhere else, that's an indication we're out of sync.
			if(comma_found_pos != 3)
				bad_commas	<= bad_commas + 1'h1;

			//Every 256 commas, see what our alignment looks like.
			if(comma_count == 8'hff) begin
				bad_commas	<= 0;

				//If a lot of them are in the wrong place, realign
				if(bad_commas > 128) begin
					locked	<= 0;
					bitslip	<= 1;
				end

				//If we have no bad commas, we're properly aligned.
				else if(bad_commas == 0)
					locked	<= 1;

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX decoder

	//6-to-5 decoder
	logic signed[2:0]	rx_5b_disparity	= 0;
	logic[4:0]			rx_5b_value		= 0;
	logic				rx_5b_control	= 0;

endmodule
