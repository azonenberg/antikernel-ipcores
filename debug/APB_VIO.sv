`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg                                                                          *
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
	@brief A multi port virtual I/O controller, readable/writable over APB
 */
module APB_VIO #(
	parameter 			OUT0_WIDTH	= 1,
	parameter[247:0]	OUT0_NAME	= 248'h0,
	parameter			OUT0_INIT	= 0,

	parameter			OUT1_WIDTH	= 1,
	parameter[247:0]	OUT1_NAME	= 248'h0,
	parameter			OUT1_INIT	= 0,

	parameter			OUT2_WIDTH	= 1,
	parameter[247:0]	OUT2_NAME	= 248'h0,
	parameter 			OUT2_INIT	= 0,

	parameter			OUT3_WIDTH	= 1,
	parameter[247:0]	OUT3_NAME	= 248'h0,
	parameter			OUT3_INIT	= 0,

	parameter			OUT4_WIDTH	= 1,
	parameter[247:0]	OUT4_NAME	= 248'h0,
	parameter			OUT4_INIT	= 0,

	parameter			OUT5_WIDTH	= 1,
	parameter[247:0]	OUT5_NAME	= 248'h0,
	parameter			OUT5_INIT	= 0,

	parameter			OUT6_WIDTH	= 1,
	parameter[247:0]	OUT6_NAME	= 248'h0,
	parameter			OUT6_INIT	= 0,

	parameter			OUT7_WIDTH	= 1,
	parameter[247:0]	OUT7_NAME	= 248'h0,
	parameter			OUT7_INIT	= 0,

	//

	parameter 			IN0_WIDTH	= 1,
	parameter[247:0]	IN0_NAME	= 248'h0,
	parameter			IN0_INIT	= 0,

	parameter			IN1_WIDTH	= 1,
	parameter[247:0]	IN1_NAME	= 248'h0,
	parameter			IN1_INIT	= 0,

	parameter			IN2_WIDTH	= 1,
	parameter[247:0]	IN2_NAME	= 248'h0,
	parameter 			IN2_INIT	= 0,

	parameter			IN3_WIDTH	= 1,
	parameter[247:0]	IN3_NAME	= 248'h0,
	parameter			IN3_INIT	= 0,

	parameter			IN4_WIDTH	= 1,
	parameter[247:0]	IN4_NAME	= 248'h0,
	parameter			IN4_INIT	= 0,

	parameter			IN5_WIDTH	= 1,
	parameter[247:0]	IN5_NAME	= 248'h0,
	parameter			IN5_INIT	= 0,

	parameter			IN6_WIDTH	= 1,
	parameter[247:0]	IN6_NAME	= 248'h0,
	parameter			IN6_INIT	= 0,

	parameter			IN7_WIDTH	= 1,
	parameter[247:0]	IN7_NAME	= 248'h0,
	parameter			IN7_INIT	= 0
) (
	//The APB bus
	APB.completer 					apb,

	//Output ports
	output wire[OUT0_WIDTH-1:0]		probe_out0,
	output wire[OUT1_WIDTH-1:0]		probe_out1,
	output wire[OUT2_WIDTH-1:0]		probe_out2,
	output wire[OUT3_WIDTH-1:0]		probe_out3,
	output wire[OUT4_WIDTH-1:0]		probe_out4,
	output wire[OUT5_WIDTH-1:0]		probe_out5,
	output wire[OUT6_WIDTH-1:0]		probe_out6,
	output wire[OUT7_WIDTH-1:0]		probe_out7,

	//Input ports
	input wire[IN0_WIDTH-1:0]		probe_in0,
	input wire[IN1_WIDTH-1:0]		probe_in1,
	input wire[IN2_WIDTH-1:0]		probe_in2,
	input wire[IN3_WIDTH-1:0]		probe_in3,
	input wire[IN4_WIDTH-1:0]		probe_in4,
	input wire[IN5_WIDTH-1:0]		probe_in5,
	input wire[IN6_WIDTH-1:0]		probe_in6,
	input wire[IN7_WIDTH-1:0]		probe_in7
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Concatenate bus width and port name

	localparam OUT0_ID = { OUT0_WIDTH[7:0], OUT0_NAME[247:0] };
	localparam OUT1_ID = { OUT1_WIDTH[7:0], OUT1_NAME[247:0] };
	localparam OUT2_ID = { OUT2_WIDTH[7:0], OUT2_NAME[247:0] };
	localparam OUT3_ID = { OUT3_WIDTH[7:0], OUT3_NAME[247:0] };
	localparam OUT4_ID = { OUT4_WIDTH[7:0], OUT4_NAME[247:0] };
	localparam OUT5_ID = { OUT5_WIDTH[7:0], OUT5_NAME[247:0] };
	localparam OUT6_ID = { OUT6_WIDTH[7:0], OUT6_NAME[247:0] };
	localparam OUT7_ID = { OUT7_WIDTH[7:0], OUT7_NAME[247:0] };

	localparam IN0_ID = { IN0_WIDTH[7:0], IN0_NAME[247:0] };
	localparam IN1_ID = { IN1_WIDTH[7:0], IN1_NAME[247:0] };
	localparam IN2_ID = { IN2_WIDTH[7:0], IN2_NAME[247:0] };
	localparam IN3_ID = { IN3_WIDTH[7:0], IN3_NAME[247:0] };
	localparam IN4_ID = { IN4_WIDTH[7:0], IN4_NAME[247:0] };
	localparam IN5_ID = { IN5_WIDTH[7:0], IN5_NAME[247:0] };
	localparam IN6_ID = { IN6_WIDTH[7:0], IN6_NAME[247:0] };
	localparam IN7_ID = { IN7_WIDTH[7:0], IN7_NAME[247:0] };

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define output logics as internal logic + wire for Efinix toolchain compatibility

	logic[OUT0_WIDTH-1:0]  probe_out0_int = OUT0_INIT;
	logic[OUT1_WIDTH-1:0]  probe_out1_int = OUT1_INIT;
	logic[OUT2_WIDTH-1:0]  probe_out2_int = OUT2_INIT;
	logic[OUT3_WIDTH-1:0]  probe_out3_int = OUT3_INIT;
	logic[OUT4_WIDTH-1:0]  probe_out4_int = OUT4_INIT;
	logic[OUT5_WIDTH-1:0]  probe_out5_int = OUT5_INIT;
	logic[OUT6_WIDTH-1:0]  probe_out6_int = OUT6_INIT;
	logic[OUT7_WIDTH-1:0]  probe_out7_int = OUT7_INIT;

	assign probe_out0 = probe_out0_int;
	assign probe_out1 = probe_out1_int;
	assign probe_out2 = probe_out2_int;
	assign probe_out3 = probe_out3_int;
	assign probe_out4 = probe_out4_int;
	assign probe_out5 = probe_out5_int;
	assign probe_out6 = probe_out6_int;
	assign probe_out7 = probe_out7_int;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter validation

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	//For now, limit to 64 bits per channel since scopehal doesn't have support for wider scalars
	//but eventually we'll want to allow going up to 256 since the address space is allocated
	if(OUT0_WIDTH > 64)
		port_width_is_invalid();
	if(OUT1_WIDTH > 64)
		port_width_is_invalid();
	if(OUT2_WIDTH > 64)
		port_width_is_invalid();
	if(OUT3_WIDTH > 64)
		port_width_is_invalid();
	if(OUT4_WIDTH > 64)
		port_width_is_invalid();
	if(OUT5_WIDTH > 64)
		port_width_is_invalid();
	if(OUT6_WIDTH > 64)
		port_width_is_invalid();
	if(OUT7_WIDTH > 64)
		port_width_is_invalid();

	if(IN0_WIDTH > 64)
		port_width_is_invalid();
	if(IN1_WIDTH > 64)
		port_width_is_invalid();
	if(IN2_WIDTH > 64)
		port_width_is_invalid();
	if(IN3_WIDTH > 64)
		port_width_is_invalid();
	if(IN4_WIDTH > 64)
		port_width_is_invalid();
	if(IN5_WIDTH > 64)
		port_width_is_invalid();
	if(IN6_WIDTH > 64)
		port_width_is_invalid();
	if(IN7_WIDTH > 64)
		port_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memory map

	/*
		64 bytes per port
			31 byte name
			1 byte width
			32 byte value

		8 input and 8 output ports allowed, can chain multiple VIOs to get more lanes
		This lets us fit in a 1024 byte chunk of address space (i.e. in standard peripheral space)

		Output ports are the low half, input the high half
	 */
	localparam PORT_BLOCK_SIZE = 64;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Combinatorial register read/status reporting

	logic		paddr_is_output;
	logic[2:0]	paddr_port_idx;
	logic		paddr_is_name;
	logic[2:0]	paddr_word_idx;

	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		//Decode the address
		paddr_is_output	= !apb.paddr[9];
		paddr_port_idx	= apb.paddr[8:6];
		paddr_is_name	= !apb.paddr[5];
		paddr_word_idx	= apb.paddr[4:2];

		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin

				//Read output ports
				if(paddr_is_output) begin

					//Output port name and width
					if(paddr_is_name) begin
						case(paddr_port_idx)
							0: apb.prdata = OUT0_ID[paddr_word_idx*32 +: 32];
							1: apb.prdata = OUT1_ID[paddr_word_idx*32 +: 32];
							2: apb.prdata = OUT2_ID[paddr_word_idx*32 +: 32];
							3: apb.prdata = OUT3_ID[paddr_word_idx*32 +: 32];
							4: apb.prdata = OUT4_ID[paddr_word_idx*32 +: 32];
							5: apb.prdata = OUT5_ID[paddr_word_idx*32 +: 32];
							6: apb.prdata = OUT6_ID[paddr_word_idx*32 +: 32];
							7: apb.prdata = OUT7_ID[paddr_word_idx*32 +: 32];
						endcase
					end

					//Output port value
					else begin
						case(paddr_port_idx)
							0: apb.prdata = probe_out0_int[paddr_word_idx*32 +: 32];
							1: apb.prdata = probe_out1_int[paddr_word_idx*32 +: 32];
							2: apb.prdata = probe_out2_int[paddr_word_idx*32 +: 32];
							3: apb.prdata = probe_out3_int[paddr_word_idx*32 +: 32];
							4: apb.prdata = probe_out4_int[paddr_word_idx*32 +: 32];
							5: apb.prdata = probe_out5_int[paddr_word_idx*32 +: 32];
							6: apb.prdata = probe_out6_int[paddr_word_idx*32 +: 32];
							7: apb.prdata = probe_out7_int[paddr_word_idx*32 +: 32];
						endcase
					end

				end

				//Read input ports
				else begin

					//Input port name and width
					if(paddr_is_name) begin
						case(paddr_port_idx)
							0: apb.prdata = IN0_ID[paddr_word_idx*32 +: 32];
							1: apb.prdata = IN1_ID[paddr_word_idx*32 +: 32];
							2: apb.prdata = IN2_ID[paddr_word_idx*32 +: 32];
							3: apb.prdata = IN3_ID[paddr_word_idx*32 +: 32];
							4: apb.prdata = IN4_ID[paddr_word_idx*32 +: 32];
							5: apb.prdata = IN5_ID[paddr_word_idx*32 +: 32];
							6: apb.prdata = IN6_ID[paddr_word_idx*32 +: 32];
							7: apb.prdata = IN7_ID[paddr_word_idx*32 +: 32];
						endcase
					end

					//Input port value
					else begin
						case(paddr_port_idx)
							0: apb.prdata = probe_in0[paddr_word_idx*32 +: 32];
							1: apb.prdata = probe_in1[paddr_word_idx*32 +: 32];
							2: apb.prdata = probe_in2[paddr_word_idx*32 +: 32];
							3: apb.prdata = probe_in3[paddr_word_idx*32 +: 32];
							4: apb.prdata = probe_in4[paddr_word_idx*32 +: 32];
							5: apb.prdata = probe_in5[paddr_word_idx*32 +: 32];
							6: apb.prdata = probe_in6[paddr_word_idx*32 +: 32];
							7: apb.prdata = probe_in7[paddr_word_idx*32 +: 32];
						endcase
					end

				end

			end

			//write error reporting: cannot write to names, or input port values
			else begin
				if(paddr_is_name || !paddr_is_output)
					apb.pslverr	= 1;
			end

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sequential register write logic

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			probe_out0_int <= OUT0_INIT;
			probe_out1_int <= OUT1_INIT;
			probe_out2_int <= OUT2_INIT;
			probe_out3_int <= OUT3_INIT;
			probe_out4_int <= OUT4_INIT;
			probe_out5_int <= OUT5_INIT;
			probe_out6_int <= OUT6_INIT;
			probe_out7_int <= OUT7_INIT;
		end

		//Normal path
		else begin

			//Can only write to output ports (duh)
			if(apb.pready && apb.pwrite && paddr_is_output && !paddr_is_name) begin
				case(paddr_port_idx)
					0: probe_out0_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
					1: probe_out1_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
					2: probe_out2_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
					3: probe_out3_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
					4: probe_out4_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
					5: probe_out5_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
					6: probe_out6_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
					7: probe_out7_int[paddr_word_idx*32 +: 32] <= apb.pwdata;
				endcase
			end

		end

	end

endmodule
