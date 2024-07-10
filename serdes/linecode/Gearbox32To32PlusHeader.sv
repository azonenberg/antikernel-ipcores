`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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
	@brief Gearboxes a 32-bit data stream to 66 bits,
	then outputs as a sequence of 32 bit data halves plus a 2-bit header
 */
module Gearbox32To32PlusHeader(

	//SERDES RX data bus
	input wire			clk,
	input wire[31:0]	rx_serdes_data,

	//Bus to MAC
	input wire			rx_bitslip,
	output logic		rx_header_valid = 0,
	output logic[1:0]	rx_header = 0,
	output logic[31:0]	rx_data = 0,
	output logic		rx_data_valid = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First stage: 32/33 gearbox

	wire[32:0]	rx_gearbox_data_out;
	wire		rx_gearbox_data_valid;

	//Gearbox the 32-bit data to 33
	Gearbox32ToN #(
		.IN_WIDTH(32),
		.OUT_WIDTH(33)
	) rx_gearbox (
		.clk(clk),
		.data_in(rx_serdes_data),
		.valid_in(1'b1),
		.bitslip(rx_bitslip),
		.data_out(rx_gearbox_data_out),
		.valid_out(rx_gearbox_data_valid)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second stage: 33/66 gearbox

	logic		rx_66b_valid = 0;
	logic		rx_66b_phase = 0;
	logic[65:0]	rx_66b_data = 0;
	always_ff @(posedge clk) begin
		rx_66b_valid	<= 0;

		if(rx_gearbox_data_valid) begin

			rx_66b_phase <= !rx_66b_phase;

			if(rx_66b_phase) begin
				rx_66b_data[32:0]	<= rx_gearbox_data_out;
				rx_66b_valid		<= 1;
			end
			else
				rx_66b_data[65:33]	<= rx_gearbox_data_out;

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Third stage: Reserialize to header + 2x 32b data

	//Convert to a format the MAC wants
	logic			rx_66b_valid_ff	= 0;
	logic[31:0]		rx_66b_data_ff = 0;
	always_ff @(posedge clk) begin

		rx_data_valid		<= 0;
		rx_header_valid		<= 0;
		rx_66b_valid_ff		<= 0;

		//First half
		if(rx_66b_valid) begin
			rx_data			<= rx_66b_data[63:32];
			rx_66b_data_ff	<= rx_66b_data[31:0];

			rx_data_valid	<= 1;
			rx_header_valid	<= 1;
			rx_header		<= rx_66b_data[65:64];
			rx_66b_valid_ff	<= 1;
		end

		//Second half
		else if(rx_66b_valid_ff) begin
			rx_data			<= rx_66b_data_ff;
			rx_data_valid	<= 1;
		end

	end

endmodule
