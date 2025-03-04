`timescale 1ns / 1ps
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
	@file
	@author Andrew D. Zonenberg
	@brief QSPI host-mode transceiver

	Does not manage chip select signals, the parent code is responsible for doing this as needed.
 */
module QSPIHostInterface(

	//Clocking
	input wire clk,

	//Clock divider is zero based with a minimum value of 1 (divide by 2)
	//LSB is ignored (all division factors are multiples of 2).
	input wire[15:0]	clkdiv,

	//QSPI interface
	output logic		qspi_sck = 0,

	output logic[3:0]	qspi_dq_out = 4'b0000,
	input wire[3:0]		qspi_dq_in,
	output logic[3:0]	qspi_dq_tris = 4'b1110,

	//Control interface
	input wire			shift_en,
	input wire			quad_shift_en,
	output logic		shift_done = 0,
	input wire[7:0]		tx_data,
	output reg[7:0]		rx_data = 0,
	input wire			auto_restart	//if set, immediately start another shift operation after the previous finishes
    );

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	logic	toggle;
	logic[14:0]	clkcount	= 0;

	logic		active			= 0;
	logic		quad_active		= 0;

	logic[3:0]	count			= 0;
	logic[6:0]	tx_shreg		= 0;

	always_ff @(posedge clk) begin

		//Bump clock counter
		if(active)
			clkcount 	= clkcount + 15'h1;

		//Figure out if clock is toggline
		if(shift_en || quad_shift_en)
			toggle	= 0;
		else
			toggle	= (clkcount >= clkdiv[15:1]) && active;

		//Toggle the clock
		if(toggle)
			qspi_sck 	= !qspi_sck;

		//Handle completion of a shift
		if(shift_done) begin

			//End of the burst? Force clock low
			shift_done		= 0;

			//If restarting, nothing special needed
			if(auto_restart) begin
			end

			//Nope, burst is done
			else begin
				qspi_sck		= 0;
				count			= 0;
				active			= 0;
				quad_active		= 0;
			end
		end

		//Wait for a start request
		if(shift_en || quad_shift_en) begin
			active			= 1;
			clkcount		= 0;
			quad_active		= quad_shift_en;

			//For now, only support quad for reads
			if(quad_shift_en) begin
				qspi_dq_tris	= 4'b1111;
				qspi_dq_out		= 4'b0000;
				tx_shreg		= 0;
			end
			else begin
				qspi_dq_tris	= 4'b1110;
				qspi_dq_out[0]	= tx_data[7];
				tx_shreg		= tx_data[6:0];
			end

			count			= 0;
			qspi_sck 		= 0;

		end

		//Toggle processing
		if(toggle && (!shift_done || auto_restart) ) begin

			//Reset the counter and toggle the clock every toggle by default
			clkcount	= 0;

			//Stop if not auto restarting
			if(shift_done && !auto_restart)
				qspi_sck	= 0;

			//ACTIVE EDGE
			else if(!qspi_sck) begin
				qspi_dq_out[0] = tx_shreg[6];

				tx_shreg = {tx_shreg[5:0], 1'b0};

				if(quad_active)
					rx_data = {rx_data[3:0], qspi_dq_in[3:0] };
				else
					rx_data = {rx_data[6:0], qspi_dq_in[1] };

			end

			//INACTIVE EDGE
			else begin

				//Stop on the end of the last clock
				if( (count == 'd8) ) begin
					shift_done		= 1;

					if(auto_restart) begin

						//Start the next word
						if(quad_active) begin
							qspi_dq_out		= 4'b0000;
							tx_shreg		= 0;
						end
						else begin
							qspi_dq_out[0]	= tx_data[7];
							tx_shreg		= tx_data[6:0];
						end

					end

					qspi_sck		= 0;
					count			= 0;

				end

				else if(quad_active)
					count = count + 4'h4;
				else
					count = count + 4'h1;

			end

		end

	end

endmodule
