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
	// RX decoder stages

	//6-to-5 decoder
	wire[6:0]			rx_6b_code		= codeword_in[9:4];
	logic signed[2:0]	rx_5b_disparity	= 0;
	logic[4:0]			rx_5b_value		= 0;
	logic				rx_5b_control	= 0;
	logic				rx_5b_error		= 0;

	always_comb begin

		//Default to being a valid data character
		rx_5b_control	<= 0;
		rx_5b_error		<= 0;
		rx_5b_disparity	<= 0;
		rx_5b_value		<= 0;

		if(codeword_valid) begin

			case(rx_6b_code)

				6'b100111: begin
					rx_5b_value		<= 0;
					rx_5b_disparity	<= 2;
				end
				6'b011000: begin
					rx_5b_value		<= 0;
					rx_5b_disparity	<= -2;
				end

				6'b011101: begin
					rx_5b_value		<= 1;
					rx_5b_disparity	<= 2;
				end
				6'b100010: begin
					rx_5b_value		<= 1;
					rx_5b_disparity	<= -2;
				end

				6'b101101: begin
					rx_5b_value		<= 2;
					rx_5b_disparity	<= 2;
				end
				6'b010010: begin
					rx_5b_value		<= 2;
					rx_5b_disparity	<= -2;
				end

				6'b110001:
					rx_5b_value		<= 3;

				6'b110101: begin
					rx_5b_value		<= 4;
					rx_5b_disparity	<= 2;
				end
				6'b001010: begin
					rx_5b_value		<= 4;
					rx_5b_disparity	<= -2;
				end

				6'b101001:
					rx_5b_value		<= 5;

				6'b011001:
					rx_5b_value		<= 6;

				6'b111000: begin
					rx_5b_value		<= 7;
					rx_5b_disparity	<= 0;
				end
				6'b000111: begin
					rx_5b_value		<= 7;
					rx_5b_disparity	<= 0;
				end

				6'b111001: begin
					rx_5b_value		<= 8;
					rx_5b_disparity	<= 2;
				end
				6'b000110: begin
					rx_5b_value		<= 8;
					rx_5b_disparity	<= -2;
				end

				6'b100101:
					rx_5b_value		<= 9;

				6'b010101:
					rx_5b_value		<= 10;

				6'b110100:
					rx_5b_value		<= 11;

				6'b001101:
					rx_5b_value		<= 12;

				6'b101100:
					rx_5b_value		<= 13;

				6'b011100:
					rx_5b_value		<= 14;

				6'b010111: begin
					rx_5b_value		<= 15;
					rx_5b_disparity	<= 2;
				end
				6'b101000: begin
					rx_5b_value		<= 15;
					rx_5b_disparity	<= -2;
				end

				6'b011011: begin
					rx_5b_value		<= 16;
					rx_5b_disparity	<= 2;
				end
				6'b100100: begin
					rx_5b_value		<= 16;
					rx_5b_disparity	<= -2;
				end

				6'b100011:
					rx_5b_value		<= 17;

				6'b010011:
					rx_5b_value		<= 18;

				6'b110010:
					rx_5b_value		<= 19;

				6'b001011:
					rx_5b_value		<= 20;

				6'b101010:
					rx_5b_value		<= 21;

				6'b011010:
					rx_5b_value		<= 22;

				6'b111010: begin
					rx_5b_value		<= 23;
					rx_5b_disparity	<= -1;
				end
				6'b000101: begin
					rx_5b_value		<= 23;
					rx_5b_disparity	<= -2;
				end

				6'b110011: begin
					rx_5b_value		<= 24;
					rx_5b_disparity	<= 2;
				end
				6'b001100: begin
					rx_5b_value		<= 24;
					rx_5b_disparity	<= -2;
				end

				6'b100110:
					rx_5b_value		<= 25;

				6'b010110:
					rx_5b_value		<= 26;

				6'b110110: begin
					rx_5b_value		<= 27;
					rx_5b_disparity	<= 2;
				end
				6'b001001: begin
					rx_5b_value		<= 27;
					rx_5b_disparity	<= -2;
				end

				6'b001110:
					rx_5b_value		<= 28;

				6'b101110: begin
					rx_5b_value		<= 29;
					rx_5b_disparity	<= 2;
				end
				6'b010001: begin
					rx_5b_value		<= 29;
					rx_5b_disparity	<= -2;
				end

				6'b011110: begin
					rx_5b_value		<= 30;
					rx_5b_disparity	<= 2;
				end
				6'b100001: begin
					rx_5b_value		<= 30;
					rx_5b_disparity	<= -2;
				end

				6'b101011: begin
					rx_5b_value		<= 31;
					rx_5b_disparity	<= 2;
				end
				6'b010100: begin
					rx_5b_value		<= 31;
					rx_5b_disparity	<= -2;
				end

				6'b001111: begin
					rx_5b_value		<= 28;
					rx_5b_disparity	<= 2;
					rx_5b_control	<= 1;
				end
				6'b110000: begin
					rx_5b_value		<= 28;
					rx_5b_disparity	<= -2;
					rx_5b_control	<= 1;
				end

				//invalid code
				default: begin
					rx_5b_disparity	<= 0;
					rx_5b_error		<= 1;
				end

			endcase
		end
	end

	//4-to-3 decoder
	wire[3:0]			rx_4b_code		= codeword_in[3:0];
	logic signed[2:0]	rx_3b_disparity	= 0;
	logic[2:0]			rx_3b_value		= 0;
	logic				rx_3b_error		= 0;

	always_comb begin

		//Default to being a valid data character
		rx_3b_error		<= 0;
		rx_3b_disparity	<= 0;
		rx_3b_value		<= 0;

		if(codeword_valid) begin

			//Separate coding for K*.x and D*.x!
			if(rx_5b_control) begin
				case(rx_4b_code)

					4'b0010: begin
						rx_3b_disparity	<= -2;
						rx_3b_value		<= 4;
					end

					4'b0011:
						rx_3b_value		<= 3;

					4'b0100: begin
						rx_3b_disparity	<= -2;
						rx_3b_value		<= 0;
					end

					4'b0101: begin
						if(rx_5b_disparity == 2)
							rx_3b_value		<= 2;
						else
							rx_3b_value		<= 5;
					end

					4'b0110: begin
						if(rx_5b_disparity -2)
							rx_3b_value		<= 1;
						else
							rx_3b_value		<= 6;
					end

					4'b0111: begin
						rx_3b_disparity	<= 2;
						rx_3b_value		<= 7;
					end

					4'b1000: begin
						rx_3b_disparity	<= -2;
						rx_3b_value		<= 7;
					end

					4'b1001: begin
						if(rx_5b_disparity == 2)
							rx_3b_value		<= 1;
						else
							rx_3b_value		<= 6;
					end

					4'b1010: begin
						if(rx_5b_disparity == -2)
							rx_3b_value		<= 2;
						else
							rx_3b_value		<= 5;
					end

					4'b1011: begin
						rx_3b_disparity	<= 2;
						rx_3b_value		<= 0;
					end

					4'b1100:
						rx_3b_value		<= 3;

					4'b1101: begin
						rx_3b_disparity	<= 2;
						rx_3b_value		<= 4;
					end

					default:
						rx_3b_error		<= 1;

				endcase
			end

			else begin
				case(rx_4b_code)

					4'b0001: begin
						rx_3b_disparity	<= -2;
						rx_3b_value		<= 7;
					end

					4'b0010: begin
						rx_3b_disparity	<= -2;
						rx_3b_value		<= 4;
					end

					4'b0011:
						rx_3b_value		<= 3;

					4'b0100: begin
						rx_3b_disparity	<= -2;
						rx_3b_value		<= 0;
					end

					4'b0101:
						rx_3b_value		<= 2;

					4'b0110:
						rx_3b_value		<= 6;

					4'b0111: begin
						rx_3b_disparity	<= 2;
						rx_3b_value		<= 7;
					end

					4'b1000: begin
						rx_3b_disparity	<= -2;
						rx_3b_value		<= 7;
					end

					4'b1001:
						rx_3b_value		<= 1;

					4'b1010:
						rx_3b_value		<= 5;

					4'b1011: begin
						rx_3b_disparity	<= 2;
						rx_3b_value		<= 0;
					end

					4'b1100:
						rx_3b_value		<= 3;

					4'b1101: begin
						rx_3b_disparity	<= 2;
						rx_3b_value		<= 4;
					end

					4'b1110: begin
						rx_3b_disparity	<= 2;
						rx_3b_value		<= 7;
					end

					default:
						rx_3b_error		<= 1;

				endcase
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Merge RX decode results

	wire signed[3:0]	rx_total_disparity		= rx_3b_disparity + rx_5b_disparity;

	logic				rx_disparity_negative	= 1;

	always_ff @(posedge clk) begin
		symbol_err	<= 0;
		data_is_ctl	<= 0;
		data		<= 0;

		data_valid	<= codeword_valid;

		disparity_err			<= 0;

		//Exceptions to the normal coding rules for a few codes
		case(codeword_in)

			//K28.7
			10'b0011111000: begin
				data_is_ctl	<= 1;
				data		<= 8'hFC;
			end
			10'b1100000111: begin
				data_is_ctl	<= 1;
				data		<= 8'hFC;
			end

			//K23.7
			10'b1110101000: begin
				data_is_ctl	<= 1;
				data		<= 8'hf7;
			end
			10'b0001010111: begin
				data_is_ctl	<= 1;
				data		<= 8'hf7;
			end

			//K27.7
			10'b1101101000: begin
				data_is_ctl	<= 1;
				data		<= 8'hfb;
			end
			10'b0010010111: begin
				data_is_ctl	<= 1;
				data		<= 8'hfb;
			end

			//K29.7
			10'b1011101000: begin
				data_is_ctl	<= 1;
				data		<= 8'hfd;
			end
			10'b0100010111: begin
				data_is_ctl	<= 1;
				data		<= 8'hfd;
			end

			//K30.7
			10'b0111101000: begin
				data_is_ctl	<= 1;
				data		<= 8'hfe;
			end
			10'b1000010111: begin
				data_is_ctl	<= 1;
				data		<= 8'hfe;
			end

			//D17.7
			10'b1000110111:
				data		<= 8'hf1;

			//D18.7
			10'b0100110111:
				data		<= 8'hf2;

			//D20.7
			10'b0010110111:
				data		<= 8'hf4;

			//D11.7
			10'b1101001000:
				data		<= 8'heb;

			//D13.7
			10'b1011001000:
				data		<= 8'hed;

			//D14.7
			10'b0111001000:
				data		<= 8'hee;

			default: begin
				symbol_err	<= rx_3b_error || rx_5b_error;
				data_is_ctl	<= rx_5b_control;
				data		<= { rx_3b_value, rx_5b_value };
			end
		endcase

		//Disparity toggles based on the current value
		if(rx_disparity_negative && (rx_total_disparity == 2) )
			rx_disparity_negative	<= 0;

		else if(!rx_disparity_negative && (rx_total_disparity == -2) )
			rx_disparity_negative	<= 1;

		else if(rx_total_disparity != 0)
			disparity_err			<= 1;

	end

endmodule
