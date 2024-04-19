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
	@brief Gearbox synchronization block for 10Gbase-R 64/66b line code
 */
module SymbolAligner64b66b(
	input wire		clk,

	input wire		header_valid,
	input wire[1:0]	header,

	output logic	bitslip 		= 0,
	output logic	block_sync_good	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX 64/66b block alignment

	logic[3:0]	block_align_errors	= 0;
	logic[10:0]	bitslip_window		= 0;
	logic[7:0]	bitslip_cooldown	= 0;

	always_ff @(posedge clk) begin

		bitslip							<= 0;

		if(header_valid) begin

			//Wait 256 clocks between bitslips to make sure we don't miss the sync
			if(bitslip_cooldown)
				bitslip_cooldown		<= bitslip_cooldown + 1'h1;

			else begin

				bitslip_window			<= bitslip_window + 1'h1;

				//Count bad headers
				if(header[0] == header[1])
					block_align_errors	<= block_align_errors + 1'h1;

				//More than 15 block align errors in 2048 blocks (arbitrary cutoff for now) triggers re-alignment
				if(block_align_errors == 15) begin
					bitslip_window		<= 0;
					block_align_errors	<= 0;
					bitslip				<= 1;
					bitslip_cooldown	<= 1;
					block_sync_good		<= 0;
				end

				if(bitslip_window == 0)
					block_align_errors	<= 0;

				//Declare block sync good after 128 blocks in a row with valid headers
				if( (bitslip_window == 128) && (block_align_errors == 0) )
					block_sync_good		<= 1;

			end

		end

	end

endmodule
