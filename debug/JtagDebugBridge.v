`default_nettype none
`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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

module JtagDebugBridge(
	output reg[3:0] led
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The TAP interface for discovery (DEBUG_IDCODE register)

	//See https://github.com/azonenberg/jtaghal/wiki/FPGA-debug
	localparam IDCODE_VID = 24'h42445a;	//"ADZ"
	localparam IDCODE_PID = 8'h00;		//Antikernel NoC interface

	reg[31:0]	idcode_shreg = 0;

	wire		idcode_active;
	wire		idcode_shift;
	wire		idcode_clear;
	wire		idcode_tck_raw;
	wire		idcode_tck_bufh;

	//The TAP itself
	JtagTAP #(
		.USER_INSTRUCTION(1)
	) idcode_tap (
		.instruction_active(idcode_active),
		.state_capture_dr(idcode_clear),
		.state_reset(),
		.state_runtest(),
		.state_shift_dr(idcode_shift),
		.state_update_dr(),
		.tck(idcode_tck_raw),
		.tck_gated(),
		.tms(),
		.tdi(),
		.tdo(idcode_shreg[0])
	);

	//Buffer the clock b/c ISE is derpy and often won't instantiate a buffer (woo skew!)
	//TODO: according to comments in older code BUFHs here sometimes won't work in spartan6?
	ClockBuffer #(
		.TYPE("LOCAL"),
		.CE("NO")
	) idcode_tck_clkbuf (
		.clkin(idcode_tck_raw),
		.clkout(idcode_tck_bufh),
		.ce(1'b1)
	);

	//The actual shift register
	always @(posedge idcode_tck_bufh) begin

		if(!idcode_active) begin
		end

		else if(idcode_clear)
			idcode_shreg	<= {IDCODE_VID, IDCODE_PID};

		else if(idcode_shift)
			idcode_shreg	<= {1'b1, idcode_shreg[31:1]};

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug glue

	always @(*) begin
		led[0]	<= idcode_active;
		led[1]	<= idcode_clear;
		led[2]	<= idcode_shift;
		led[3]	<= 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The TAP interface for Antikernel debug
	/*
	wire	tap_active;
	wire	tap_capture;

	wire	tck_raw;
	wire	tms;
	wire	tdi;
	wire	tdo;

	JtagTAP #(
		);
	*/

endmodule
