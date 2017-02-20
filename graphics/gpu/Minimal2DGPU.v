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

/**
	@brief Minimalistic 2D graphics processor

	Pixels are an opaque datatype, we don't care what the bits in them mean.
 */
module Minimal2DGPU(
	clk,

	framebuffer_mem_en,
	framebuffer_mem_wr,
	framebuffer_mem_addr,
	framebuffer_mem_wdata,
	framebuffer_mem_rdata,

	fg_color,
	bg_color,

	cmd_en,
	cmd,
	cmd_done
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameters

	//Can't use v2001 syntax b/c functions -> localparams -> port widths

	`include "../../synth_helpers/clog2.vh"

	parameter FRAMEBUFFER_WIDTH				= 128;	//Pixels per scanline
	parameter FRAMEBUFFER_HEIGHT			= 32;	//Rows in the image
	parameter PIXEL_DEPTH					= 1;	//Number of bits in a pixel

	localparam FRAMEBUFFER_PIXELS			= FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT;
	localparam FRAMEBUFFER_BITS				= FRAMEBUFFER_PIXELS * PIXEL_DEPTH;
	localparam FRAMEBUFFER_BYTES			= FRAMEBUFFER_BITS / 8;
	localparam PIXELS_PER_BYTE				= 8 / PIXEL_DEPTH;
	localparam FRAMEBUFFER_ADDR_BITS		= clog2(FRAMEBUFFER_BYTES);

	localparam MAX_ADDR						= FRAMEBUFFER_BYTES - 1'h1;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// System-wide stuff

	input wire		clk;							//Core and interface clock

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Framebuffer SRAM interface

	output reg								framebuffer_mem_en		= 0;
	output reg								framebuffer_mem_wr		= 0;
	output reg[FRAMEBUFFER_ADDR_BITS-1:0]	framebuffer_mem_addr	= 0;
	output reg[7:0] 						framebuffer_mem_wdata	= 0;
	input wire[7:0]							framebuffer_mem_rdata;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Drawing command interface

	input wire[PIXEL_DEPTH-1:0] fg_color;			//Foreground draw color
	input wire[PIXEL_DEPTH-1:0] bg_color;			//Background draw color

	input wire					cmd_en;				//Start processing something
	input wire[3:0]				cmd;				//The operation to execute
	output reg					cmd_done = 0;		//Set high for one clock when the command finishes

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sanity checks: bit depth must be 1, 2, 4, 8 for now (multi-byte pixels or weird alignments not supported)

	initial begin

		if( (PIXEL_DEPTH == 1) || (PIXEL_DEPTH == 2) || (PIXEL_DEPTH == 4) || (PIXEL_DEPTH == 8) ) begin
			//all OK
		end

		else begin
			$display("ERROR: Minimal2DGPU: PIXEL_DEPTH %d is unsupported", PIXEL_DEPTH);
			$finish;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Helper values

	wire[7:0] solid_bg						= {PIXELS_PER_BYTE{bg_color}};
	wire[7:0] solid_fg						= {PIXELS_PER_BYTE{fg_color}};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	`include "Minimal2DGPU_opcodes_localparam.vh"

	reg[FRAMEBUFFER_ADDR_BITS-1 : 0]		count = 0;


	//State values
	localparam STATE_IDLE			= 8'h00;
	localparam STATE_CLEAR_0		= 8'h01;

	reg[7:0] state	= STATE_IDLE;

	always @(posedge clk) begin

		cmd_done				<= 0;
		framebuffer_mem_en		<= 0;
		framebuffer_mem_wr		<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE: Wait for a command request to come in

			STATE_IDLE: begin

				if(cmd_en) begin

					case(cmd)

						//do nothing!
						GPU_OP_NOP: begin
							cmd_done	<= 1;
						end	//end GPU_OP_NOP

						//Clear the screen
						GPU_OP_CLEAR: begin
							count		<= 0;
							state		<= STATE_CLEAR_0;
						end	//end GPU_OP_CLEAR

						//Silently ignore all other commands (TODO report error)
						default: begin
							cmd_done	<= 1;
						end

					endcase

				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Clear the screen to the background color

			STATE_CLEAR_0: begin
				framebuffer_mem_en		<= 1;
				framebuffer_mem_wr		<= 1;
				framebuffer_mem_addr	<= count;
				framebuffer_mem_wdata	<= solid_bg;

				count					<= count + 1'h1;

				//Done if we're writing the last pixel
				if(count == MAX_ADDR) begin
					cmd_done			<= 1;
					state				<= STATE_IDLE;
				end

			end	//end STATE_CLEAR_0

		endcase
	end

endmodule
