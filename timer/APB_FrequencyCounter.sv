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

`include "../../../antikernel-ipcores/amba/apb/APBTypes.sv"

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	APB controlled frequency counter

	Counts how many cycles of an unknown input frequency we see in a given number of cycles of a known reference.
 */
module APB_FrequencyCounter(

	//The APB bus
	APB.completer 	apb,

	//Test signal input
	input wire		clkin
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32 bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[7:0]
	{
		REG_CTRL		= 'h00,		//[0] start a measurement
		REG_STATUS		= 'h04,		//[0] measurement in progress
		REG_TESTLEN		= 'h08,		//Number of cycles to measure the input for
		REG_COUNT		= 'h0c		//Count value
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize start/done flags

	logic	start = 0;
	wire	start_sync;

	PulseSynchronizer sync_start(
		.clk_a(apb.pclk),
		.pulse_a(start),

		.clk_b(clkin),
		.pulse_b(start_sync)
	);

	logic	stop = 0;
	wire	stop_sync;

	PulseSynchronizer sync_stop(
		.clk_a(apb.pclk),
		.pulse_a(stop),

		.clk_b(clkin),
		.pulse_b(stop_sync)
	);

	logic[31:0] count		= 0;
	wire[31:0]	count_sync;
	wire		done;

	RegisterSynchronizer #(.WIDTH(32)) sync_result(
		.clk_a(clkin),
		.en_a(stop_sync),
		.ack_a(),
		.reg_a(count),

		.clk_b(apb.pclk),
		.updated_b(done),
		.reset_b(!apb.preset_n),
		.reg_b(count_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Test domain counter

	logic	running = 1;

	always_ff @(posedge clkin) begin

		if(start_sync) begin
			count	<= 0;
			running	<= 1;
		end
		else if(stop_sync)
			running	<= 0;
		else
			count	<= count + 1;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic and ref clock counter

	logic[31:0] testlen		= 0;
	logic[31:0] testcount	= 0;
	logic		busy		= 0;

	always_comb begin

		//Combinatorially assert PREADY when selected
		apb.pready		= apb.psel && apb.penable;

		//Default to no errors and no read data
		apb.prdata		= 0;
		apb.pslverr		= 0;

		if(apb.pready) begin

			if(apb.pwrite) begin

				case(apb.paddr)

					REG_CTRL: begin
					end
					REG_TESTLEN: begin
					end

					//unmapped or non-writable address
					default:	apb.pslverr		= 1;

				endcase

			end

			else begin

				case(apb.paddr)

					REG_CTRL:		apb.prdata = 0;
					REG_STATUS:		apb.prdata = { 31'h0, busy };
					REG_TESTLEN:	apb.prdata = testlen;
					REG_COUNT:		apb.prdata = count_sync;

					//unmapped address
					default:		apb.pslverr		= 1;

				endcase

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			testlen		<= 0;
			testcount	<= 0;
			busy		<= 0;
			start		<= 0;
			stop		<= 0;
		end

		//Normal path
		else begin

			start	<= 0;
			stop	<= 0;

			if(busy) begin
				testcount	<= testcount + 1;
				if(testcount == testlen)
					stop	<= 1;
			end

			if(done)
				busy	<= 0;

			if(apb.pready) begin

				//Writes
				if(apb.pwrite) begin

					case(apb.paddr)
						REG_CTRL: begin
							if(apb.pwdata[0]) begin
								start 		<= 1;
								busy		<= 1;
								testcount	<= 0;
							end
						end
						REG_TESTLEN:	testlen	<= apb.pwdata;

						default: begin
						end
					endcase

				end

			end
		end

	end

endmodule
