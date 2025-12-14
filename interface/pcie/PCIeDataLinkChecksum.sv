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

module PCIeDataLinkChecksum(
	input wire			clk,
	input wire			rst_n,

	input wire			first_phase,
	input wire			second_phase,
	input wire[15:0]	data_in_first,
	input wire[15:0]	data_in_second,

	output logic[15:0]	crc_out	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Combinatorial CRC engine

	logic[15:0]	crc_din;
	logic[15:0]	crc_comb;

	always_comb begin

		if(first_phase) begin
			crc_din		= data_in_first;
			crc_comb	= 16'hffff;
		end

		else begin
			crc_din		= data_in_second;
			crc_comb	= crc_out;
		end

		for(integer i=0; i<2; i++) begin
			for(integer j=0; j<8; j++) begin
				if(crc_comb[0] ^ crc_din[8*i + j])
					crc_comb = crc_comb[15:1] ^ 16'hd008;
				else
					crc_comb = crc_comb[15:1];
			end
		end

		//Output inversion and byte swapping
		if(second_phase)
			crc_comb = {~crc_comb[7:0], ~crc_comb[15:8] };

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register the output

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			crc_out	<= 0;
		end

		else begin
			if(first_phase || second_phase)
				crc_out	<= crc_comb;
		end

	end

endmodule
