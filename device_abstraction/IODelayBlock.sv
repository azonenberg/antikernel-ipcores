`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2021 Andrew D. Zonenberg                                                                          *
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
	@brief An I/O delay line

	For now, we only support synthesis-time fixed delay values (no runtime tuning).
 */
module IODelayBlock #(
	parameter WIDTH = 16,

	parameter CAL_FREQ		= 200,		//IDELAYCTRL reference frequency (MHz)
										//For now, only support 200/400 MHz, not 300

	parameter INPUT_DELAY	= 100,		//picoseconds
	parameter OUTPUT_DELAY	= 100,		//picoseconds
	parameter DIRECTION		= "IN",		//IN or OUT only support for now (no IO mode yet)
	parameter IS_CLOCK		= 0
) (
	input wire[WIDTH-1 : 0]		i_pad,				//input from pad to rx datapath
	output wire[WIDTH-1 : 0]	i_fabric,			//output from rx datapath to fabric

	output wire[WIDTH-1 : 0]	o_pad,				//output from tx datapath to pad
	input wire[WIDTH-1 : 0]		o_fabric,			//input from fabric or serdes to tx datapath

	input wire		input_en						//high = input, low = output
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual delay block

	for(genvar i=0; i<WIDTH; i++) begin

		//PTV-calibrated delays for 7 series
		//For now, we only support fixed delays
		//300/400 MHz refclk only supported in -2 and -3 speed grades for 7 series!
		//Delays for artix7 and kintex7 are the same
		`ifdef XILINX_7SERIES

			localparam tap_size				= (CAL_FREQ == 400) ? 39 : 78;	//39.0625 ps per tap at 400 MHz
																			//78.125 ps per tap at 200 MHz
			localparam input_delay_taps 	= INPUT_DELAY / tap_size;
			localparam output_delay_taps	= INPUT_DELAY / tap_size;

			//Sanity check, max number of taps is 31
			initial begin
				if(input_delay_taps > 31) begin
					$fatal(1, "ERROR: IODelayBlock computed >31 taps (%d) for input delay value %d ps",
						input_delay_taps, INPUT_DELAY);
				end
				if(output_delay_taps > 31) begin
					$fatal(1, "ERROR: IODelayBlock computed >31 taps (%d) for input delay value %d ps",
						output_delay_taps, OUTPUT_DELAY);
				end
			end

			//Create the input delay
			if(DIRECTION == "IN") begin

				//Create the IDELAY block
				IDELAYE2 #(
					.IDELAY_TYPE("FIXED"),
					.DELAY_SRC("IDATAIN"),
					.IDELAY_VALUE(input_delay_taps),
					.HIGH_PERFORMANCE_MODE("FALSE"),		//TODO: decide when to enable
					.SIGNAL_PATTERN(IS_CLOCK ? "CLOCK" : "DATA"),
					.REFCLK_FREQUENCY(CAL_FREQ),
					.CINVCTRL_SEL("FALSE"),
					.PIPE_SEL("FALSE")
				) idelayblock (
					.C(),
					.REGRST(1'b0),
					.LD(1'b0),
					.CE(1'b0),
					.INC(1'b0),
					.CINVCTRL(1'b0),
					.CNTVALUEIN(5'b0),
					.IDATAIN(i_pad[i]),
					.DATAIN(1'b0),
					.LDPIPEEN(1'b0),
					.DATAOUT(i_fabric[i]),
					.CNTVALUEOUT()
				);

				assign o_pad[i]				= 0;
			end

			else if(DIRECTION == "OUT") begin
				//ODELAY not implemented for 7 series yet
				/*
				initial begin
					$display("7-series ODELAY not implemented yet in IODelayBlock\n");
					$finish;
				end
				*/
				assign o_pad[i]				= o_fabric[i];
			end

			else begin
				initial begin
					$fatal(1, "IODelayBlock DIRECTION must be IN or OUT");
				end
			end

			//Print stats
			initial begin
				if(i == 0) begin
					if(DIRECTION != "OUT") begin
						$info("Target input delay for IODelayBlock %m is %d ps, actual is %d",
							INPUT_DELAY, input_delay_taps * tap_size);
					end
					if(DIRECTION != "IN") begin
						$info("Target output delay for IODelayBlock %m is %d ps, actual is %d",
							OUTPUT_DELAY, input_delay_taps * tap_size);
					end
				end
			end

		`else
			$fatal(1, "IODelayBlock: unrecognized device family (did you forget to define XILINX_7SERIES?)");
		`endif

	end

endmodule

