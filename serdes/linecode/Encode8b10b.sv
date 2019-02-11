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
	@author	Andrew D. Zonenberg
	@brief	8b/10b line coder
 */
module Encode8b10b(
	input wire			clk,

	input wire			data_is_ctl,
	input wire			force_disparity_negative,
	input wire[7:0]		data,

	output logic[9:0]	codeword	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//5B/6B encoder

	wire[4:0]	tx_5b_code = data[4:0];

	logic[5:0]	tx_6b_code_if_neg;
	logic[5:0]	tx_6b_code_if_pos;

	logic		tx_6b_disparity_flip;

	always_comb begin

		//Default to invalid code group
		tx_6b_code_if_neg		<= 6'b000000;
		tx_6b_code_if_pos		<= 6'b000000;

		//Default to flipping disparity
		tx_6b_disparity_flip	<= 1;

		//K/x
		if(data_is_ctl) begin

			case(tx_5b_code)

				23: begin
					tx_6b_code_if_neg	<= 6'b111010;
					tx_6b_code_if_pos	<= 6'b000101;
				end

				27: begin
					tx_6b_code_if_neg	<= 6'b110110;
					tx_6b_code_if_pos	<= 6'b001001;
				end

				28: begin
					tx_6b_code_if_neg	<= 6'b001111;
					tx_6b_code_if_pos	<= 6'b110000;
				end

				29: begin
					tx_6b_code_if_neg	<= 6'b101110;
					tx_6b_code_if_pos	<= 6'b010001;
				end

				30: begin
					tx_6b_code_if_neg	<= 6'b011110;
					tx_6b_code_if_pos	<= 6'b100001;
				end

			endcase

		end

		//D.x
		else begin

			case(tx_5b_code)

				0: begin
					tx_6b_code_if_neg	<= 6'b100111;
					tx_6b_code_if_pos	<= 6'b011000;
				end

				1: begin
					tx_6b_code_if_neg	<= 6'b011101;
					tx_6b_code_if_pos	<= 6'b100010;
				end

				2: begin
					tx_6b_code_if_neg	<= 6'b101101;
					tx_6b_code_if_pos	<= 6'b010010;
				end

				3: begin
					tx_6b_code_if_neg		<= 6'b110001;
					tx_6b_code_if_pos		<= 6'b110001;
					tx_6b_disparity_flip	<= 0;
				end

				4: begin
					tx_6b_code_if_neg	<= 6'b110101;
					tx_6b_code_if_pos	<= 6'b001010;
				end

				5: begin
					tx_6b_code_if_neg		<= 6'b101001;
					tx_6b_code_if_pos		<= 6'b101001;
					tx_6b_disparity_flip	<= 0;
				end

				6: begin
					tx_6b_code_if_neg		<= 6'b011001;
					tx_6b_code_if_pos		<= 6'b011001;
					tx_6b_disparity_flip	<= 0;
				end

				7: begin
					tx_6b_code_if_neg	<= 6'b111000;
					tx_6b_code_if_pos	<= 6'b000111;
				end

				8: begin
					tx_6b_code_if_neg	<= 6'b111001;
					tx_6b_code_if_pos	<= 6'b000110;
				end

				9: begin
					tx_6b_code_if_neg		<= 6'b100101;
					tx_6b_code_if_pos		<= 6'b100101;
					tx_6b_disparity_flip	<= 0;
				end

				10: begin
					tx_6b_code_if_neg		<= 6'b010101;
					tx_6b_code_if_pos		<= 6'b010101;
					tx_6b_disparity_flip	<= 0;
				end

				11: begin
					tx_6b_code_if_neg		<= 6'b110100;
					tx_6b_code_if_pos		<= 6'b110100;
					tx_6b_disparity_flip	<= 0;
				end

				12: begin
					tx_6b_code_if_neg		<= 6'b001101;
					tx_6b_code_if_pos		<= 6'b001101;
					tx_6b_disparity_flip	<= 0;
				end

				13: begin
					tx_6b_code_if_neg		<= 6'b101100;
					tx_6b_code_if_pos		<= 6'b101100;
					tx_6b_disparity_flip	<= 0;
				end

				14: begin
					tx_6b_code_if_neg		<= 6'b011100;
					tx_6b_code_if_pos		<= 6'b011100;
					tx_6b_disparity_flip	<= 0;
				end

				15: begin
					tx_6b_code_if_neg	<= 6'b010111;
					tx_6b_code_if_pos	<= 6'b101000;
				end

				16: begin
					tx_6b_code_if_neg	<= 6'b011011;
					tx_6b_code_if_pos	<= 6'b100100;
				end

				17: begin
					tx_6b_code_if_neg		<= 6'b100011;
					tx_6b_code_if_pos		<= 6'b100011;
					tx_6b_disparity_flip	<= 0;
				end

				18: begin
					tx_6b_code_if_neg		<= 6'b010011;
					tx_6b_code_if_pos		<= 6'b010011;
					tx_6b_disparity_flip	<= 0;
				end

				19: begin
					tx_6b_code_if_neg		<= 6'b110010;
					tx_6b_code_if_pos		<= 6'b110010;
					tx_6b_disparity_flip	<= 0;
				end

				20: begin
					tx_6b_code_if_neg		<= 6'b001011;
					tx_6b_code_if_pos		<= 6'b001011;
					tx_6b_disparity_flip	<= 0;
				end

				21: begin
					tx_6b_code_if_neg		<= 6'b101010;
					tx_6b_code_if_pos		<= 6'b101010;
					tx_6b_disparity_flip	<= 0;
				end

				22: begin
					tx_6b_code_if_neg		<= 6'b011010;
					tx_6b_code_if_pos		<= 6'b011010;
					tx_6b_disparity_flip	<= 0;
				end

				23: begin
					tx_6b_code_if_neg	<= 6'b111010;
					tx_6b_code_if_pos	<= 6'b000101;
				end

				24: begin
					tx_6b_code_if_neg	<= 6'b110011;
					tx_6b_code_if_pos	<= 6'b001100;
				end

				25: begin
					tx_6b_code_if_neg		<= 6'b100110;
					tx_6b_code_if_pos		<= 6'b100110;
					tx_6b_disparity_flip	<= 0;
				end

				26: begin
					tx_6b_code_if_neg		<= 6'b010110;
					tx_6b_code_if_pos		<= 6'b010110;
					tx_6b_disparity_flip	<= 0;
				end

				27: begin
					tx_6b_code_if_neg	<= 6'b110110;
					tx_6b_code_if_pos	<= 6'b001001;
				end

				28: begin
					tx_6b_code_if_neg		<= 6'b001110;
					tx_6b_code_if_pos		<= 6'b001110;
					tx_6b_disparity_flip	<= 0;
				end

				29: begin
					tx_6b_code_if_neg	<= 6'b101110;
					tx_6b_code_if_pos	<= 6'b010001;
				end

				30: begin
					tx_6b_code_if_neg	<= 6'b011110;
					tx_6b_code_if_pos	<= 6'b100001;
				end

				31: begin
					tx_6b_code_if_neg	<= 6'b101011;
					tx_6b_code_if_pos	<= 6'b010100;
				end

			endcase

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//3B/4B encoder

	wire[2:0]	tx_3b_code = data[7:5];

	logic[3:0]	tx_4b_code_if_neg;
	logic[3:0]	tx_4b_code_if_pos;
	logic		tx_4b_disparity_flip;

	always_comb begin
		tx_4b_code_if_pos		<= 0;
		tx_4b_code_if_neg		<= 0;
		tx_4b_disparity_flip	<= 1;

		if(data_is_ctl) begin
			case(tx_3b_code)

				0: begin
					tx_4b_code_if_neg		<= 4'b1011;
					tx_4b_code_if_pos		<= 4'b0100;
				end

				1: begin
					tx_4b_code_if_neg		<= 4'b0110;
					tx_4b_code_if_pos		<= 4'b1001;
					tx_4b_disparity_flip	<= 0;
				end

				2: begin
					tx_4b_code_if_neg		<= 4'b1010;
					tx_4b_code_if_pos		<= 4'b0101;
					tx_4b_disparity_flip	<= 0;
				end

				3: begin
					tx_4b_code_if_neg		<= 4'b1100;
					tx_4b_code_if_pos		<= 4'b0011;
					tx_4b_disparity_flip	<= 0;
				end

				4: begin
					tx_4b_code_if_neg		<= 4'b1101;
					tx_4b_code_if_pos		<= 4'b0010;
				end

				5: begin
					tx_4b_code_if_neg		<= 4'b0101;
					tx_4b_code_if_pos		<= 4'b1010;
					tx_4b_disparity_flip	<= 0;
				end

				6: begin
					tx_4b_code_if_neg		<= 4'b1001;
					tx_4b_code_if_pos		<= 4'b0110;
					tx_4b_disparity_flip	<= 0;
				end

				7: begin
					tx_4b_code_if_neg		<= 4'b0111;
					tx_4b_code_if_pos		<= 4'b1000;
				end

			endcase
		end

		else begin

			case(tx_3b_code)

				0: begin
					tx_4b_code_if_neg		<= 4'b1011;
					tx_4b_code_if_pos		<= 4'b0100;
				end

				1: begin
					tx_4b_code_if_neg		<= 4'b1001;
					tx_4b_code_if_pos		<= 4'b1001;
					tx_4b_disparity_flip	<= 0;
				end

				2: begin
					tx_4b_code_if_neg		<= 4'b0101;
					tx_4b_code_if_pos		<= 4'b0101;
					tx_4b_disparity_flip	<= 0;
				end

				3: begin
					tx_4b_code_if_neg		<= 4'b1100;
					tx_4b_code_if_pos		<= 4'b0011;
				end

				4: begin
					tx_4b_code_if_neg		<= 4'b1101;
					tx_4b_code_if_pos		<= 4'b0010;
				end

				5: begin
					tx_4b_code_if_neg		<= 4'b1010;
					tx_4b_code_if_pos		<= 4'b1010;
					tx_4b_disparity_flip	<= 0;
				end

				6: begin
					tx_4b_code_if_neg		<= 4'b0110;
					tx_4b_code_if_pos		<= 4'b0110;
					tx_4b_disparity_flip	<= 0;
				end

				7: begin

					//default to primary coding
					tx_4b_code_if_neg		<= 4'b1110;
					tx_4b_code_if_pos		<= 4'b0001;

					//alternate coding Dx.A7 to avoid unwanted commas
					if( (tx_5b_code == 17) || (tx_5b_code == 18) || (tx_5b_code == 20) )
						tx_4b_code_if_neg	<= 4'b0111;

					if( (tx_5b_code == 11) || (tx_5b_code == 13) || (tx_5b_code == 14) )
						tx_4b_code_if_pos	<= 4'b1000;

				end

			endcase

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Final coder

	wire	tx_disparity_flip 		= tx_4b_disparity_flip ^ tx_6b_disparity_flip;

	logic	tx_disparity_negative	= 1;

	always_ff @(posedge clk) begin

		if(tx_disparity_negative || force_disparity_negative) begin
			codeword[9:4]	<= tx_6b_code_if_neg;
			if(tx_6b_disparity_flip)
				codeword[3:0]	<= tx_4b_code_if_pos;
			else
				codeword[3:0]	<= tx_4b_code_if_neg;
		end

		else begin

			codeword[9:4]	<= tx_6b_code_if_pos;
			if(tx_6b_disparity_flip)
				codeword[3:0]	<= tx_4b_code_if_neg;
			else
				codeword[3:0]	<= tx_4b_code_if_pos;

		end

		if(tx_disparity_flip)
			tx_disparity_negative	<= !tx_disparity_negative;

		if(force_disparity_negative)
			tx_disparity_negative	<= !tx_disparity_flip;

	end

endmodule
