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
	@brief Mux between link training and data link layer to drive the 8b10b output. Also inserts skip sets.
 */
module PCIeOutputMux(
	input wire			clk,
	input wire			rst_n,

	//Outputs to SERDES
	output logic[15:0]	tx_data			= 0,
	output logic[1:0]	tx_charisk		= 0,

	//Inputs from link training
	input wire[15:0]	tx_train_data,
	input wire[1:0]		tx_train_charisk,
	input wire			tx_train_skip_ack,

	//Inputs from data link layer

	//Skip request
	output logic		tx_skip_req		= 0,
	output logic		tx_skip_done	= 0
);

	//Send skips every 1180 to 1538 symbol times per spec
	//Target 1400 symbols (700 clocks) interval
	logic[9:0]	count 			= 0;
	logic		tx_skip_done_ff	= 0;

	always_ff @(posedge clk or negedge rst_n) begin
		if(!rst_n) begin
			count			<= 0;
			tx_skip_req		<= 0;
			tx_skip_done	<= 0;
			tx_skip_done_ff	<= 0;
		end

		else begin

			//TODO: actual mux
			//for now, just forward data from link training
			tx_data		<= tx_train_data;
			tx_charisk	<= tx_train_charisk;

			//Clear single cycle flags
			tx_skip_done	<= 0;

			//Pipeline delays
			tx_skip_done_ff	<= tx_skip_done;

			//Request a skip set every 700 symbols
			if(!tx_skip_req) begin
				count		<= count + 1;

				if(count == 700) begin
					tx_skip_req	<= 1;
				end
			end

			//Send a skip when a request is acknowledged
			if(tx_train_skip_ack) begin
				tx_skip_req		<= 0;
				tx_skip_done	<= 1;
			end

			if(tx_skip_done) begin
				tx_charisk		<= 2'b11;
				tx_data			<= 16'h1cbc;
			end

			//Second half of skip set
			if(tx_skip_done_ff) begin
				tx_charisk		<= 2'b11;
				tx_data			<= 16'h1c1c;
			end

		end

	end

endmodule
