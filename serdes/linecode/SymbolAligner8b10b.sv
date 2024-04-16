`default_nettype none
`timescale 1ns/1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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
	@brief 8B/10B line comma aligner
 */
module SymbolAligner8b10b(
	input wire			clk,

	input wire			codeword_valid,
	input wire[19:0]	comma_window,			//Sliding window big enough for any possible comma to be in

	output logic		locked			= 0,	//true if we've locked to a comma at this lane position
	output logic		no_commas		= 0,	//true if we're not seeing any commas
	output logic		bitslip			= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Comma detector

	logic		comma_found;
	logic[4:0]	comma_found_pos;

	integer	i;


	always_ff @(posedge clk) begin

		comma_found	<= 0;

		if(codeword_valid) begin

			for(i=0; i<10; i=i+1) begin

				//Positive comma
				if(comma_window[i +: 5] == 5'b11111) begin
					comma_found				<= 1;
					comma_found_pos			<= i;
				end

				//Negative comma
				if(comma_window[i +: 5] == 5'b00000) begin
					comma_found				<= 1;
					comma_found_pos			<= i;
				end

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Comma aligner

	logic[7:0]	comma_count		= 0;
	logic[7:0]	bad_commas		= 0;
	logic[15:0]	commaLastSeen	= 0;

	logic		codeword_valid_ff	= 0;

	always_ff @(posedge clk) begin

		bitslip				<= 0;

		codeword_valid_ff	<= codeword_valid;

		if(codeword_valid_ff) begin

			if(comma_found) begin
				commaLastSeen	<= 0;
				no_commas		<= 0;

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

			//Not a comma
			else begin
				if(commaLastSeen == 16'hffff)
					no_commas	<= 1;
				else
					commaLastSeen	<= commaLastSeen + 1;
			end

		end

	end

endmodule
