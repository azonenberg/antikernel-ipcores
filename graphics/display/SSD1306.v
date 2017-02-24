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
	@brief Driver for a Solomon SSD1306 OLED controller
 */
module SSD1306 #(
	parameter INTERFACE = "SPI"		//this is the only supported interface for now
) (

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// System-wide stuff

	input wire clk,					//Core and interface clock
	input wire[15:0] clkdiv,		//SPI clock divisor

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// To display

	//System control signals
	output reg	rst_out_n = 0,		//Reset output to display
	output reg	vbat_en_n = 1,		//Power rail enables
	output reg	vdd_en_n = 1,

	//SPI
	output wire spi_sck,			//4-wire SPI bus to display (MISO not used by this core)
	output wire spi_mosi,
	output reg	spi_cs_n = 1,

	//Misc data lines
	output reg	cmd_n = 0,			//SPI command/data flag

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// To GPU

	//Command inputs
	input wire		powerup,					//Request to turn the display on
	input wire		powerdown,					//Request to turn the display off
	input wire		refresh,					//Request to refresh the display from the GPU framebuffer

	//Status outputs
	output wire		ready,						//1 = ready for new commands, 0 = busy
												//All commands except "power down" are ignored when not ready.
												//Power down is queued if needed.

	output reg		framebuffer_rd_en	= 0,	//Framebuffer SRAM read bus (expects single cycle latency)
	output reg[8:0]	framebuffer_rd_addr	= 0,
	input wire[7:0]	framebuffer_rd_data,

	output reg		power_state = 0				//1=on, 0=off
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sanity check

    initial begin
		if(INTERFACE != "SPI") begin
			$display("ERROR: SSD1306 only supports INTERFACE=SPI for now");
			$finish;
		end
    end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SPI interface

	reg			spi_shift_en	= 0;
	wire		spi_shift_done;
	reg[7:0]	spi_tx_data		= 0;

	SPITransceiver #(
		.SAMPLE_EDGE("RISING"),
		.LOCAL_EDGE("NORMAL")
    ) spi_tx (

		.clk(clk),
		.clkdiv(clkdiv),

		.spi_sck(spi_sck),
		.spi_mosi(spi_mosi),
		.spi_miso(1'b0),			//read not hooked up

		.shift_en(spi_shift_en),
		.shift_done(spi_shift_done),
		.tx_data(spi_tx_data),
		.rx_data()
    );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SPI chip select control wrapper

    reg			spi_byte_en		= 0;
    reg[2:0]	spi_byte_state	= 0;
    reg[2:0]	spi_byte_count	= 0;
    reg			spi_byte_done	= 0;

    //SPI state machine
    always @(posedge clk) begin

		spi_shift_en		<= 0;
		spi_byte_done		<= 0;

		case(spi_byte_state)

			//Wait for command request, then assert CS
			0: begin
				if(spi_byte_en) begin
					spi_cs_n		<= 0;
					spi_byte_state	<= 1;
					spi_byte_count	<= 0;
				end
			end

			//Wait 3 clocks of setup time, then initiate the transfer
			1: begin
				spi_byte_count		<= spi_byte_count + 1'd1;
				if(spi_byte_count == 2) begin
					spi_shift_en	<= 1;
					spi_byte_state	<= 2;
				end
			end

			//Wait for transfer to finish
			2: begin
				if(spi_shift_done) begin
					spi_byte_count	<= 0;
					spi_byte_state	<= 3;
				end
			end

			//Wait 3 clocks of hold time, then deassert CS
			3: begin
				spi_byte_count		<= spi_byte_count + 1'd1;
				if(spi_byte_count == 2) begin
					spi_cs_n		<= 1;
					spi_byte_state	<= 4;
					spi_byte_count	<= 0;
				end
			end

			//Wait 3 clocks of inter-frame gap, then return
			4: begin
				spi_byte_count		<= spi_byte_count + 1'd1;
				if(spi_byte_count == 2) begin
					spi_byte_done	<= 1;
					spi_byte_state	<= 0;
				end
			end

		endcase

    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Framebuffer rotation

    /*
		Our framebuffer is stored in scanline order (as is common for pretty much all standard image file formats)
		The display wants us to output data in 8-pixel vertical slices so we have to rotate the bit ordering
		within each 8x8 block.

		To read a new block of data, assert block_read and wait for block_ready to go high
     */
    reg			block_read		= 0;
    reg			block_ready		= 0;
    reg[3:0]	block_col		= 0;
    reg[2:0]	block_row		= 0;

	//pixel_scanline[0] is scanline 0 of the block
	//just a raw array of FFs for transposing
    reg[7:0]	pixel_scanline0 = 8'h80;
    reg[7:0]	pixel_scanline1 = 8'h00;
    reg[7:0]	pixel_scanline2 = 8'h00;
    reg[7:0]	pixel_scanline3 = 8'h00;
    reg[7:0]	pixel_scanline4 = 8'h00;
    reg[7:0]	pixel_scanline5 = 8'h00;
    reg[7:0]	pixel_scanline6 = 8'h00;
    reg[7:0]	pixel_scanline7 = 8'h00;

	reg			framebuffer_rd_en_ff	= 0;
	reg[2:0]	block_scanline			= 0;
	wire[2:0]	next_scanline			= block_scanline + 1'h1;
	wire		more_scanlines 			= (block_scanline != 7);

	/*
		Address map: blocks go L-R then raster scan (16 blocks wide x 4 high, or 8 for a 64-pixel display)
		Each block is 1 byte wide x 8 scanlines high

		addr[8:7] = row (9:7 for 64-line displays)
		addr[6:4] = scanline
		addr[3:0] = col
	 */
    always @(posedge clk) begin

		//clear flags
		framebuffer_rd_en		<= 0;
		block_ready				<= 0;

		//one cycle after we dispatched a read, data is available
		framebuffer_rd_en_ff	<= framebuffer_rd_en;

		//Start reading the first scanline of a new block
		if(block_read)
			block_scanline			<= 0;

		//Reading a new scanline if we're starting a block, or not done with current one
		framebuffer_rd_addr[8:7]	<= block_row[1:0];
		framebuffer_rd_addr[3:0]	<= block_col + 4'h1;	//what is this offset from?
		if(block_read) begin
			framebuffer_rd_addr[6:4]	<= 0;
			framebuffer_rd_en			<= 1;
		end
		if(framebuffer_rd_en_ff && more_scanlines) begin
			framebuffer_rd_addr[6:4]	<= next_scanline;
			framebuffer_rd_en			<= 1;
		end

		//Bump row pointer if there's more stuff to read
		if(framebuffer_rd_en_ff && more_scanlines)
			block_scanline			<= next_scanline;

		//Save completed scanlines in the buffer
		//0x80 should be a vertical column of pixels, 1 in 8 illuminated
		if(framebuffer_rd_en_ff) begin
			case(block_scanline)
				0:	pixel_scanline0		<= framebuffer_rd_data;
				1:	pixel_scanline1		<= framebuffer_rd_data;
				2:	pixel_scanline2		<= framebuffer_rd_data;
				3:	pixel_scanline3		<= framebuffer_rd_data;
				4:	pixel_scanline4		<= framebuffer_rd_data;
				5:	pixel_scanline5		<= framebuffer_rd_data;
				6:	pixel_scanline6		<= framebuffer_rd_data;
				7:	pixel_scanline7		<= framebuffer_rd_data;
			endcase
		end

		//Done with the block? Let the display controller know
		if(framebuffer_rd_en_ff && !more_scanlines)
			block_ready						<= 1;

    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main state machine

	localparam STATE_OFF			= 8'h00;
	localparam STATE_BOOT_0			= 8'h01;
	localparam STATE_BOOT_1			= 8'h02;
	localparam STATE_BOOT_2			= 8'h03;
	localparam STATE_BOOT_3			= 8'h04;
	localparam STATE_BOOT_4			= 8'h05;
	localparam STATE_BOOT_5			= 8'h06;
	localparam STATE_WAIT_IDLE		= 8'h07;
	localparam STATE_IDLE			= 8'h08;
	localparam STATE_SHUTDOWN_0		= 8'h09;
	localparam STATE_SHUTDOWN_1		= 8'h0a;
	localparam STATE_SHUTDOWN_2		= 8'h0b;
	localparam STATE_REFRESH_0		= 8'h0c;
	localparam STATE_REFRESH_1		= 8'h0d;
	localparam STATE_REFRESH_2		= 8'h0e;
	localparam STATE_REFRESH_3		= 8'h0f;
	localparam STATE_REFRESH_4		= 8'h10;

	reg[7:0]	state			= 0;
	reg[23:0]	count			= 0;

	reg[2:0]	block_nbit		= 0;

	reg powerdown_pending		= 0;

	assign ready = (state == STATE_IDLE) || (state == STATE_OFF);

	//Microcode table of init commands
	//TODO: some of this is panel specific, have a parameter to specify various configs?
	reg[3:0]	init_rom_addr	= 0;
	reg[7:0]	init_rom[15:0];
	initial begin

		init_rom[0]		<= 8'h8d;		//Set up charge pump for internal DC-DC
		init_rom[1]		<= 8'h14;
		init_rom[2]		<= 8'hd9;		//Set pre-charge period for internal DC-DCs
		init_rom[3]		<= 8'hf1;

		init_rom[4]		<= 8'ha1;		//Segment re-mapping
		init_rom[5]		<= 8'hc8;		//COM scan direction
		init_rom[6]		<= 8'hda;		//COM hardware config. Note that panel docs say 0x02 which is wrong!
		init_rom[7]		<= 8'h20;
		init_rom[8]		<= 8'h2e;		//Disable scrolling

		init_rom[9]		<= 8'he3;		//nop padding for future init commands
		init_rom[10]	<= 8'he3;
		init_rom[11]	<= 8'he3;
		init_rom[12]	<= 8'he3;
		init_rom[13]	<= 8'he3;
		init_rom[14]	<= 8'he3;

		init_rom[15]	<= 8'haf;		//Turn on display

	end
	wire[7:0]	init_rom_cmd	= init_rom[init_rom_addr];

    always @(posedge clk) begin

		spi_byte_en				<= 0;
		block_read				<= 0;

		if(powerdown)
			powerdown_pending	<= 1;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// OFF

			STATE_OFF: begin

				power_state		<= 0;

				if(powerup) begin
					vdd_en_n	<= 0;

					count		<= 0;
					state		<= STATE_BOOT_0;

				end
			end	//end STATE_OFF

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BOOT: power etc initialization

			//Give power rails ~1 ms to stabilize, then turn the display off
			STATE_BOOT_0: begin

				//Get ready to read the first ROM command
				init_rom_addr	<= 0;

				count			<= count + 1'h1;
				if(count == 24'h01ffff) begin
					spi_tx_data		<= 8'hae;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= STATE_BOOT_1;
				end
			end	//end STATE_BOOT_0

			//Wait for command to finish, then strobe reset for ~1 ms
			STATE_BOOT_1: begin
				if(spi_byte_done) begin
					rst_out_n		<= 0;
					count			<= 0;
					state			<= STATE_BOOT_2;
				end
			end	//end STATE_BOOT_1

			//When reset finishes, send the first init command
			STATE_BOOT_2: begin
				count			<= count + 1'h1;
				if(count == 24'h01ffff) begin
					rst_out_n		<= 1;

					init_rom_addr	<= init_rom_addr + 1'h1;

					spi_tx_data		<= init_rom_cmd;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= STATE_BOOT_3;
				end
			end	//end STATE_BOOT_2

			//Send remaining init commands
			STATE_BOOT_3: begin
				if(spi_byte_done) begin

					init_rom_addr	<= init_rom_addr + 1'h1;

					spi_tx_data		<= init_rom_cmd;
					spi_byte_en		<= 1;
					cmd_n			<= 0;

					//If we have more commands, stay here.
					//If we just sent the last command, move on.
					if(init_rom_addr == 'hf)
						state			<= STATE_BOOT_4;
				end
			end	//end STATE_BOOT_3

			//When the last send finishes, turn on Vbat
			STATE_BOOT_4: begin
				if(spi_byte_done) begin
					vbat_en_n		<= 0;
					count			<= 0;
					state			<= STATE_BOOT_5;
				end
			end	//end STATE_BOOT_4

			//Wait 100 ms then go to idle
			STATE_BOOT_5: begin
				count				<= count + 1'h1;
				if(count == 24'hbfffff)
					state			<= STATE_IDLE;
			end	//end STATE_BOOT_5

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// WAIT: go to idle after current txn finishes

			STATE_WAIT_IDLE: begin
				if(spi_byte_done)
					state			<= STATE_IDLE;
			end	//end STATE_WAIT_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE: Wait for something to happen

			STATE_IDLE: begin

				power_state					<= 1;

				//If we were asked to shut down, do that
				if(powerdown_pending) begin
					powerdown_pending		<= 0;
					state					<= STATE_SHUTDOWN_0;
				end

				//If asked to refresh the display, do that
				else if(refresh) begin
					block_row				<= 0;
					block_col				<= 0;

					//Send a nop b/c REFRESH_0 expects to wait for a tx
					spi_tx_data				<= 8'hE3;
					spi_byte_en				<= 1;
					cmd_n					<= 0;

					state					<= STATE_REFRESH_0;
				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// REFRESH: Update the display

			//Block-based raster scan: 8 pixels high left to right, then next block
			//Each byte is one pixel wide and 8 high.

			//Send row pointer
			STATE_REFRESH_0: begin
				if(spi_byte_done) begin
					spi_tx_data		<= {4'hB, 1'b0, block_row};
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= STATE_REFRESH_1;
				end
			end	//end STATE_REFRESH_0

			//Col addr low = 0
			STATE_REFRESH_1: begin
				if(spi_byte_done) begin
					spi_tx_data		<= {4'h0, block_col[0], 3'h0};
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= STATE_REFRESH_2;
				end
			end	//end STATE_REFRESH_1

			//Col addr high = 0
			STATE_REFRESH_2: begin
				if(spi_byte_done) begin
					spi_tx_data		<= {5'h1, block_col[3:1]};
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= STATE_REFRESH_3;
					count			<= 0;

					block_col		<= 0;
				end
			end	//end STATE_REFRESH_2

			//Fetch the next block of data
			STATE_REFRESH_3: begin
				if(spi_byte_done) begin
					block_read		<= 1;
					block_nbit		<= 0;
					state			<= STATE_REFRESH_4;
				end
			end	//end STATE_REFRESH_3

			//When the block comes back, start sending it out
			STATE_REFRESH_4: begin

				if(block_ready || spi_byte_done) begin

					//Send the byte
					spi_tx_data		<=
					{
						pixel_scanline7[block_nbit],
						pixel_scanline6[block_nbit],
						pixel_scanline5[block_nbit],
						pixel_scanline4[block_nbit],
						pixel_scanline3[block_nbit],
						pixel_scanline2[block_nbit],
						pixel_scanline1[block_nbit],
						pixel_scanline0[block_nbit]
					};
					spi_byte_en		<= 1;
					cmd_n			<= 1;

					//Go to next bitplane in the block
					block_nbit		<= block_nbit + 1'h1;

					//If done with this block, move to the next one in the row
					if(block_nbit == 7) begin

						//Default to fetching the next block from the current row
						block_nbit			<= 0;
						state				<= STATE_REFRESH_3;

						//If done with this column, move to next row
						if(block_col == 15) begin

							block_col		<= 0;
							block_row		<= block_row + 1'h1;
							state			<= STATE_REFRESH_0;

							//If done with the last row, finish
							if(block_row == 3)
								state		<= STATE_WAIT_IDLE;

						end

						//Nope, just go to next block position
						else
							block_col	<= block_col + 1'h1;

					end

				end
			end	//end STATE_REFRESH_4

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// SHUTDOWN: turn the display off

			//Send "display off" command
			STATE_SHUTDOWN_0: begin
				spi_tx_data		<= 8'hae;
				spi_byte_en		<= 1;
				cmd_n			<= 0;
				state			<= STATE_SHUTDOWN_1;
			end	//end STATE_SHUTDOWN_0

			//When send finishes, turn off Vbat
			STATE_SHUTDOWN_1: begin
				if(spi_byte_done) begin
					vbat_en_n	<= 1;
					count		<= 0;
					state		<= STATE_SHUTDOWN_2;
				end
			end	//end STATE_SHUTDOWN_1

			//Wait 100ms then turn off Vdd and reset
			STATE_SHUTDOWN_2: begin
				count			<= count + 1'h1;
				if(count == 24'hbfffff) begin
					vdd_en_n	<= 1;
					state		<= STATE_OFF;
				end
			end	//end STATE_SHUTDOWN_2

		endcase

    end

endmodule
