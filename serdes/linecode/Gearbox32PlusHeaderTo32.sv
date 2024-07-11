`timescale 1ns / 1ps
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
	@brief Gearbox wrapper for interfacing an XGEthernetPCS to a SERDES without using any hard IP gearboxing
 */
module Gearbox32PlusHeaderTo32(

	//Common clock
	input wire			clk,

	//PCS interface
	input wire			tx_header_valid,
	input wire[1:0]		tx_header,
	input wire			tx_data_valid,
	input wire[31:0]	tx_data,

	//SERDES interface
	output wire[31:0]	tx_serdes_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 1: Rearrange the TX data stream from {header, data} to 33 data bits plus a valid flag

	logic		tx_33b_data_valid	= 0;
	logic[32:0]	tx_33b_data			= 0;
	logic		tx_33b_lsb			= 0;
	always_ff @(posedge clk) begin

		tx_33b_data_valid	<= 0;

		if(tx_header_valid && tx_data_valid) begin
			tx_33b_data_valid	<= 1;
			tx_33b_data			<= { tx_header, tx_data[31:1] };
			tx_33b_lsb			<= tx_data[0];
		end
		else if(tx_data_valid) begin
			tx_33b_data_valid	<= 1;
			tx_33b_data			<= { tx_33b_lsb, tx_data[31:0] };
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Step 2: Gearbox the 33-bit data to 32

	Gearbox33To32 tx_gearbox(
		.clk(clk),
		.data_in(tx_33b_data),
		.valid_in(tx_33b_data_valid),
		.data_out(tx_serdes_data)
	);

endmodule
