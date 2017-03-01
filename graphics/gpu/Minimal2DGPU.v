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

	left,
	right,
	top,
	bottom,

	cmd_en,
	cmd,
	cmd_char,
	cmd_char_width,
	cmd_char_height,
	cmd_done,
	cmd_fail
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

	localparam X_BITS						= clog2(FRAMEBUFFER_WIDTH);
	localparam Y_BITS						= clog2(FRAMEBUFFER_HEIGHT);

	parameter FONT_HEIGHT					= 16;	//max number of lines per character cell
	parameter FONT_WIDTH					= 16;	//max number of columns per character cell
	localparam FONT_WBITS					= clog2(FONT_WIDTH);

	//A font should contain FONT_HEIGHT lines for each character.
	//Fonts should contain all printable characters from ' ' (0x20) to '~' (0x7e)
	//This is 0x5f (95) rows.

	localparam FONT_ROW_BITS				= clog2(FONT_HEIGHT);

	/*
		For now, we don't know how high the font is (all we care is that all chars fit in a 16 x16 cell)

		Arial 12 point:		16
		Arial 6 point:		8
		Courier 7 point:	8
	 */
	assign cmd_char_height					= FONT_HEIGHT;

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

	input wire[PIXEL_DEPTH-1:0] fg_color;				//Foreground draw color
	input wire[PIXEL_DEPTH-1:0] bg_color;				//Background draw color

	input wire[X_BITS-1:0]		left;					//Corners for operations
	input wire[Y_BITS-1:0]		top;
	input wire[X_BITS-1:0]		right;
	input wire[Y_BITS-1:0]		bottom;

	input wire					cmd_en;					//Start processing something
	input wire[3:0]				cmd;					//The operation to execute
	input wire[7:0]				cmd_char;				//Character to draw
	output reg[3:0]				cmd_char_width;			//Width of the character we just drew
	output wire[4:0]			cmd_char_height;		//Height of a character cell
	output reg					cmd_done = 0;			//Set high for one clock when the command finishes
	output reg					cmd_fail = 0;			//Set high for one clock when the command finishes with an error

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sanity checks: bit depth must be 1, 2, 4, 8 for now (multi-byte pixels or weird alignments not supported)

	initial begin

		if( (PIXEL_DEPTH == 1) /*|| (PIXEL_DEPTH == 2) || (PIXEL_DEPTH == 4) || (PIXEL_DEPTH == 8)*/ ) begin
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
	// Font ROM

	//TODO: MemoryMacro once we get dep scanning figured out?
	//TODO: make this parameterizable?
	reg[FONT_WIDTH-1:0] font_rom[2047:0];
	initial begin
		$readmemh("../fonts/arial-12pt.hex", font_rom);
	end

	//we only need 6 bits, but pad out for now to avoid warnings
	localparam CINDEX_BITS = 11 - FONT_ROW_BITS;

	reg[CINDEX_BITS-1:0]	font_cindex = 0;		//Index within the font ROM (anything below ' ' is non-printable)
	reg[FONT_ROW_BITS-1:0]	font_line	= 0;		//Line within the current character
	reg[FONT_WBITS:0]		font_col	= 0;		//Column within the current character (need extra bit for = case)

	reg						font_rom_rd	= 0;
	reg[FONT_WIDTH-1:0]		font_rom_out = 0;
	always @(posedge clk) begin
		if(font_rom_rd)
			font_rom_out	<= font_rom[ {font_cindex, font_line} ];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Font width ROM (for proportional fonts)

	reg[FONT_WBITS-1 : 0]	font_width_rom[127:0];
	initial begin
		$readmemh("../fonts/arial-12pt-width.hex", font_width_rom);
	end

	reg[3:0]		font_rom_cwidth = 0;
	always @(posedge clk) begin
		if(font_rom_rd)
			font_rom_cwidth	<= font_width_rom[font_cindex];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	`include "Minimal2DGPU_opcodes_localparam.vh"

	reg[FRAMEBUFFER_ADDR_BITS-1 : 0]		count = 0;

	//State values
	localparam STATE_IDLE			= 8'h00;
	localparam STATE_CLEAR_0		= 8'h01;
	localparam STATE_RECT_0			= 8'h02;
	localparam STATE_RECT_1			= 8'h03;
	localparam STATE_RECT_2			= 8'h04;
	localparam STATE_RECT_3			= 8'h05;

	localparam STATE_PIXEL_0		= 8'h10;
	localparam STATE_PIXEL_1		= 8'h11;

	localparam STATE_HLINE_0		= 8'h20;
	localparam STATE_HLINE_1		= 8'h21;
	localparam STATE_HLINE_2		= 8'h22;
	localparam STATE_HLINE_3		= 8'h23;
	localparam STATE_HLINE_4		= 8'h24;

	localparam STATE_CHAR_0			= 8'h30;
	localparam STATE_CHAR_1			= 8'h31;
	localparam STATE_CHAR_2			= 8'h32;
	localparam STATE_CHAR_3			= 8'h33;

	reg[X_BITS-1:0]	pix_x			= 0;						//Random X/Y helper coordinates
	reg[Y_BITS-1:0]	pix_y			= 0;

	wire[X_BITS-4:0] pix_x_byte		= pix_x[X_BITS-1 : 3];		//Byte position of the current pixel
																//TODO: Update for multi-bit stuff?
	wire[2:0] pix_x_col				= pix_x[2:0];
	wire[X_BITS-4:0] pix_x_byte_inc	= pix_x_byte + 1'h1;		//Byte index of next pixel byte in the line
	wire[X_BITS-1:0] pix_x_inc		= {pix_x_byte_inc, 3'h0};	//Starting X coordinate of next pixel byte in the line

	wire[X_BITS-4:0] left_byte		= left[X_BITS-1 : 3];		//Byte position of the left pixel being written
	wire[2:0] left_col				= left[2:0];

	wire[X_BITS-4:0] right_byte		= right[X_BITS-1 : 3];		//Byte position of the right pixel being written
	wire[2:0] right_col				= right[2:0];

	reg[7:0] state					= STATE_IDLE;
	reg[7:0] state_ret				= STATE_IDLE;

	reg framebuffer_mem_en_ff		= 0;

	integer i;

	always @(posedge clk) begin

		cmd_done				<= 0;
		framebuffer_mem_en		<= 0;
		framebuffer_mem_wr		<= 0;

		font_rom_rd				<= 0;

		framebuffer_mem_en_ff	<= framebuffer_mem_en;

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

						//Draw an outlined rectangle between specified X and Y coordinates
						GPU_OP_RECT: begin
							state		<= STATE_RECT_0;
						end	//end GPU_OP_RECT

						//Draw a character with top left corner at (left, top)
						//TODO: report delta X to calling code?
						//If we ask for a non-printable char, complain
						GPU_OP_CHAR: begin
							if( (cmd_char < " ") || (cmd_char > "~") ) begin
								cmd_done	<= 1;
								cmd_fail	<= 1;
							end
							else
								state		<= STATE_CHAR_0;
						end	//end GPU_OP_CHAR

						//Silently ignore all other commands
						default: begin
							cmd_fail	<= 1;
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

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Draw an outline-only rectangle very inefficiently (one rmw per pixel o_O)

			//Draw top side
			STATE_RECT_0: begin
				pix_y					<= top;
				state_ret				<= STATE_RECT_1;
				state					<= STATE_HLINE_0;
			end	//end STATE_RECT_0

			//Draw bottom side
			STATE_RECT_1: begin
				pix_y					<= bottom;
				state_ret				<= STATE_RECT_2;
				state					<= STATE_HLINE_0;
			end	//end STATE_RECT_1

			//Prepare to do the vertical stuff
			STATE_RECT_2: begin
				pix_x					<= left;
				pix_y					<= top;
				state					<= STATE_RECT_3;
			end	//end STATE_RECT_2

			STATE_RECT_3: begin

				//If we just drew the last pixel, done
				if(pix_y == bottom) begin

					//If we're drawing the left side, start the right
					if(pix_x == left) begin
						pix_x			<= right;
						pix_y			<= top;
						state			<= STATE_PIXEL_0;
						state_ret		<= STATE_RECT_3;
					end

					//Done drawing
					else begin
						cmd_done		<= 1;
						state			<= STATE_IDLE;
					end

				end

				//Nope, draw the next pixel
				else begin
					pix_y				<= pix_y + 1'h1;
					state				<= STATE_PIXEL_0;
					state_ret			<= STATE_RECT_3;
				end

			end	//end STATE_RECT_3

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Draw a single character at (left, top)

			//Set up the char ROM
			STATE_CHAR_0: begin
				font_cindex				<= cmd_char[6:0] - 7'h20;
				font_line				<= 0;
				font_col				<= 0;
				font_rom_rd				<= 1;
				state					<= STATE_CHAR_1;
			end	//end STATE_CHAR_0

			//Wait for rom read
			STATE_CHAR_1: begin
				state					<= STATE_CHAR_2;
			end	//end STATE_CHAR_1

			//Write the pixels one at a time (TODO optimize)
			STATE_CHAR_2: begin

				state_ret				<= STATE_CHAR_3;

				//Draw at the proper offset within the character cell
				pix_x					<= left + font_col;
				pix_y					<= top + font_line;

				//Draw if char rom bit is set
				if(font_rom_out[FONT_WIDTH - 1 - font_col])
					state				<= STATE_PIXEL_0;

				else
					state				<= STATE_CHAR_3;

				//Bump our column
				font_col				<= font_col + 1'h1;

			end	//end STATE_CHAR_2

			//Move to next location
			STATE_CHAR_3: begin

				//If we just did the last pixel in the row, go to the next row
				//(also stop early if we hit the right side of the display)
				if( (font_col == FONT_WIDTH) || (pix_x == FRAMEBUFFER_WIDTH - 1) ) begin

					//End of last row? We're finished
					if(font_line == (FONT_HEIGHT - 1'h1) ) begin
						cmd_done		<= 1;
						cmd_char_width	<= font_rom_cwidth;
						state			<= STATE_IDLE;
					end

					//Nope, read the next row from the char rom and keep going
					else begin
						font_line		<= font_line + 1'h1;
						font_col		<= 0;
						font_rom_rd		<= 1;
						state			<= STATE_CHAR_1;
					end

				end

				//Nope, keep going in the current row
				else begin
					state				<= STATE_CHAR_1;
				end

			end	//end STATE_CHAR_3

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Helper: Set a single pixel at (pix_x, pix_y) via read-modify-write

			//Dispatch read
			STATE_PIXEL_0: begin
				framebuffer_mem_en		<= 1;
				framebuffer_mem_addr	<= {pix_y, pix_x_byte};
				state					<= STATE_PIXEL_1;
			end	//end STATE_PIXEL_0

			//Mask in the new bit and write it back
			STATE_PIXEL_1: begin
				if(framebuffer_mem_en_ff) begin
					framebuffer_mem_en					<= 1;
					framebuffer_mem_wr					<= 1;
					framebuffer_mem_wdata				<= framebuffer_mem_rdata;
					framebuffer_mem_wdata[pix_x_col]	<= fg_color;	//TODO: multi-bit handling
					state								<= state_ret;
				end
			end	//end STATE_PIXEL_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Helper: Draw a horizontal line from (left, pix_y) to (right, pix_y) efficiently (one rmw per byte)

			//Kick off the first read
			STATE_HLINE_0: begin
				pix_x					<= left;

				framebuffer_mem_en		<= 1;
				framebuffer_mem_addr	<= {pix_y, left_byte};
				state					<= STATE_HLINE_1;
			end	//end STATE_HLINE_0

			//Fill in the pixels from the left to the end of the first byte.
			STATE_HLINE_1: begin

				if(framebuffer_mem_en_ff) begin
					framebuffer_mem_en					<= 1;
					framebuffer_mem_wr					<= 1;
					framebuffer_mem_wdata				<= framebuffer_mem_rdata;

					//If the line is entirely within this byte, we're done right now
					if(left_byte == right_byte) begin
						for(i=0; i<8; i=i+1) begin
							if(i >= left_col && i <= right_col)
								framebuffer_mem_wdata[i]	<= fg_color;	//TODO: multi-bit handling
						end

						state								<= state_ret;
					end

					//Nope, fill to end of byte
					else begin
						for(i=0; i<8; i=i+1) begin
							if(i >= left_col)
								framebuffer_mem_wdata[i]	<= fg_color;	//TODO: multi-bit handling
						end

						//Start the byte-wide fill
						pix_x								<= pix_x_inc;
						state								<= STATE_HLINE_2;

					end

				end

			end	//end STATE_HLINE_1

			//Fill in the pixels until the end of the full bytes (no read needed)
			STATE_HLINE_2: begin

				//If we're done with full-byte writes, then move to the last block
				if(pix_x_byte >= right_byte)
					state				<= STATE_HLINE_3;

				//Nope, fill this block
				else begin
					framebuffer_mem_en		<= 1;
					framebuffer_mem_addr	<= {pix_y, pix_x_byte};
					framebuffer_mem_wr		<= 1;
					framebuffer_mem_wdata	<= solid_fg;

					pix_x					<= pix_x_inc;
				end

			end	//end STATE_HLINE_2

			//Read the last byte
			STATE_HLINE_3: begin
				framebuffer_mem_en		<= 1;
				framebuffer_mem_addr	<= {pix_y, pix_x_byte};
				state					<= STATE_HLINE_4;
			end	//end STATE_HLINE_3

			//Write the last byte
			STATE_HLINE_4: begin
				if(!framebuffer_mem_en && framebuffer_mem_en_ff) begin

					framebuffer_mem_en		<= 1;
					framebuffer_mem_wr		<= 1;

					framebuffer_mem_wdata	<= framebuffer_mem_rdata;
					for(i=0; i<8; i=i+1) begin
						if(i <= right_col)
							framebuffer_mem_wdata[i]	<= fg_color;	//TODO: multi-bit handling
					end

					//Done
					state				<= state_ret;
				end
			end	//end STATE_HLINE_4

		endcase
	end

endmodule
