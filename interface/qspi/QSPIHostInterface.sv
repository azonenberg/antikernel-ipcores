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
module QSPIHostInterface #(

	//Indicates which edge of SCK the remote end samples data on.
	parameter SAMPLE_EDGE 		= "RISING",

	//Indicates which edge of SCK the local end samples data on
	//NORMAL = same as remote
	//INVERTED = opposite
	parameter LOCAL_EDGE 		= "NORMAL",

	//Set true to gate transitions on rx_data during a shift operation
	//and only update when shift_done goes high. Adds one cycle of latency
	parameter CHANGE_ON_DONE	= 0,

	//If PIPE_DIV is true, clkdiv has an additional cycle of latency but timing is improved
	parameter PIPE_DIV			= 0
) (

	//Clocking
	input wire clk,

	//Clock divider is zero based with a minimum value of 1 (divide by 2)
	//LSB is ignored (all division factors are multiples of 2).
	input wire[15:0]	clkdiv,

	//SPI interface
	output logic		qspi_sck = 0,
	output logic		qspi_mosi = 0,
	input wire			qspi_miso,

	//Control interface
	input wire			shift_en,
	output logic		shift_done = 0,
	input wire[7:0]		tx_data,
	output reg[7:0]		rx_data = 0

	`ifdef FORMAL
	, output logic active	= 0
	`endif
    );

 	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock divider edge finding

	logic	toggle;
	logic[14:0]	clkcount	= 0;

	if(PIPE_DIV) begin
		always_ff @(posedge clk) begin
			if(shift_en)
				toggle	<= 0;
			else
				toggle	<= (clkcount >= clkdiv[15:1]) && active;
		end
	end

	else begin
		always_comb begin
			if(shift_en)
				toggle	= 0;
			else
				toggle	= (clkcount >= clkdiv[15:1]) && active;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	`ifndef FORMAL
	logic		active		= 0;
	`endif

	logic[3:0]	count		= 0;

	logic[6:0]	tx_shreg		= 0;

	logic[7:0]	rx_shreg		= 0;
	logic		shift_done_adv	= 0;

	//Optionally: Only update output at end of a shift operation
	generate
		if(CHANGE_ON_DONE) begin
			always_ff @(posedge clk) begin
				shift_done		<= 0;
				if(shift_done_adv) begin
					rx_data		<= rx_shreg;
					shift_done	<= 1;
				end
			end
		end

		else begin
			always_comb begin
				rx_data		= rx_shreg;
				shift_done	= shift_done_adv;
			end
		end
	endgenerate

	initial begin
		if( (SAMPLE_EDGE != "RISING") && (SAMPLE_EDGE != "FALLING") ) begin
			$fatal("ERROR: Invalid sample edge in SPIHostInterface");
		end
	end

	logic almost_done	= 0;

	always_ff @(posedge clk) begin
		shift_done_adv	<= 0;

		//Prevent starting in illegal states during formal verification
		`ifdef FORMAL
			assert(clkcount <= clkdiv[15:1]);
			assert(count <= 9);

			if(!active) begin
				assert(!almost_done);
			end
		`endif

		//Wait for a start request
		if(shift_en) begin
			active		<= 1;
			clkcount	<= 0;

			if(SAMPLE_EDGE == "FALLING") begin
				count	<= 1;
				qspi_sck <= 1;
			end
			else begin
				count	<= 0;
				qspi_sck <= 0;
			end

			qspi_mosi <= tx_data[7];
			tx_shreg <= tx_data[6:0];
		end

		//Toggle processing
		else if(active) begin
			clkcount 	<= clkcount + 15'h1;

			if(toggle) begin

				//Reset the counter and toggle the clock
				clkcount	<= 0;
				qspi_sck 	<= !qspi_sck;

				//Make the done flag wait half a bit period if necessary
				if(almost_done) begin
					qspi_sck			<= 0;
					shift_done_adv	<= 1;
					active			<= 0;
					almost_done		<= 0;
				end

				//ACTIVE EDGE
				else if( (qspi_sck && (SAMPLE_EDGE == "RISING")) || (!qspi_sck && (SAMPLE_EDGE == "FALLING")) ) begin
					qspi_mosi <= tx_shreg[6];

					tx_shreg <= {tx_shreg[5:0], 1'b0};

					if(LOCAL_EDGE == "INVERTED")
						rx_shreg <= {rx_shreg[6:0], qspi_miso};

				end

				//INACTIVE EDGE
				else begin
					count <= count + 4'h1;

					//Stop on the end of the last clock
					if( (count == 'd8) ) begin
						qspi_sck		<= 0;
						almost_done	<= 1;
					end

					//Sample just before the clock rises
					else if(LOCAL_EDGE == "NORMAL")
						rx_shreg <= {rx_shreg[6:0], qspi_miso};

				end

			end
		end

	end

endmodule
