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
	@brief	10-to-8 gearbox for line coding
 */
module Gearbox10To8(
	input wire			clk,

	input wire			fifo_ready,
	output logic		fifo_rd_en	= 0,
	input wire[9:0]		fifo_rd_data,

	output logic[7:0]	dout		= 0
);

	logic[7:0]	temp_buf	= 0;

	logic[2:0]	rd_count	= 0;

	always_ff @(posedge clk) begin

		rd_count		<= rd_count + 1'h1;

		case(rd_count)

			//TX word 3 (temp_buf has 6 bits)
			0: begin

				fifo_rd_en	<= 1;

				//Hold after reset until FIFO fills
				if(!fifo_ready) begin
					rd_count	<= 0;
					fifo_rd_en	<= 0;
				end

				dout		<= { temp_buf[5:0], fifo_rd_data[9:8] };
				temp_buf	<= fifo_rd_data[7:0];

			end

			//TX word 4 (temp_buf has all 8 bits)
			1: begin
				fifo_rd_en	<= 1;

				dout		<= temp_buf;
				temp_buf	<= 0;
			end

			//TX word 0 (temp_buf is empty)
			2: begin
				fifo_rd_en	<= 1;

				dout		<= fifo_rd_data[9:2];
				temp_buf	<= fifo_rd_data[1:0];
			end

			//TX word 1 (temp_buf has 2 bits)
			3: begin
				fifo_rd_en	<= 1;

				dout		<= { temp_buf[1:0], fifo_rd_data[9:4] };
				temp_buf	<= fifo_rd_data[3:0];
			end

			//TX word 2 (temp_buf has 4 bits)
			4: begin
				fifo_rd_en	<= 0;
				rd_count	<= 0;

				dout		<= { temp_buf[3:0], fifo_rd_data[9:6] };
				temp_buf	<= fifo_rd_data[5:0];
			end

		endcase

	end

endmodule
