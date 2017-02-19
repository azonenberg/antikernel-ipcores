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

	output reg rst_out_n = 0,		//Reset output to display

	output wire spi_sck,			//4-wire SPI bus to display (MISO not used by this core)
	output wire spi_mosi,
	output reg spi_cs_n = 1,

	output reg cmd_n = 0,			//SPI command/data flag

	output reg vbat_en_n = 1,		//Power rail enables
	output reg vdd_en_n = 1,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// To GPU

	input wire powerup,				//Request to turn the display on
	input wire powerdown			//Request to turn the display off
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sanity check

    initial begin
		if(INTERFACE != "SPI") begin
			$display("SSD1306 only supports INTERFACE=SPI for now");
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
    // Board bring-up

	reg[7:0] state = 0;
	reg[23:0] count = 0;

    always @(posedge clk) begin

		spi_byte_en				<= 0;

		case(state)

			//Init
			0: begin

				if(powerup) begin
					vdd_en_n	<= 0;

					count		<= 0;
					state		<= 1;

				end
			end

			//Give power rails ~1 ms to stabilize, then turn the display off
			1: begin
				count			<= count + 1'h1;
				if(count == 24'h01ffff) begin
					spi_tx_data		<= 8'hae;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= 2;
				end
			end

			//Wait for command to finish, then strobe reset for ~1 ms
			2: begin
				if(spi_byte_done) begin
					rst_out_n		<= 0;
					count			<= 0;
					state			<= 3;
				end
			end

			//When reset finishes, set the charge pump and pre-charge period
			3: begin
				count			<= count + 1'h1;
				if(count == 24'h01ffff) begin
					rst_out_n		<= 1;

					spi_tx_data		<= 8'h8d;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= 4;
				end
			end
			4: begin
				if(spi_byte_done) begin
					spi_tx_data		<= 8'h14;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= 5;
				end
			end
			5: begin
				if(spi_byte_done) begin
					spi_tx_data		<= 8'hd9;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= 6;
				end
			end
			6: begin
				if(spi_byte_done) begin
					spi_tx_data		<= 8'hf1;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= 7;
				end
			end

			//When the last send finishes, turn on Vcc and wait ~100 ms
			7: begin
				if(spi_byte_done) begin
					vbat_en_n		<= 0;
					count			<= 0;
					state			<= 8;
				end
			end

			//After the wait is over, turn the display to solid white regardless of the actual RAM contents
			8: begin
				count				<= count + 1'h1;
				if(count == 24'hbfffff) begin
					spi_tx_data		<= 8'ha5;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= 9;
				end
			end

			//Turn the actual display on
			9: begin
				if(spi_byte_done) begin
					spi_tx_data		<= 8'hAF;
					spi_byte_en		<= 1;
					cmd_n			<= 0;
					state			<= 10;
				end
			end

			//Done, wait for something to happen
			10: begin
				if(powerdown)
					state			<= 50;
			end

			/////////////////////

			//SHUTDOWN: Send "display off" command
			50: begin
				spi_tx_data		<= 8'hae;
				spi_byte_en		<= 1;
				cmd_n			<= 0;
				state			<= 51;
			end

			//When send finishes, turn off Vbat
			51: begin
				if(spi_byte_done) begin
					vbat_en_n	<= 1;
					count		<= 0;
					state		<= 52;
				end
			end

			//Wait 100ms then turn off Vdd and reset
			52: begin
				count			<= count + 1'h1;
				if(count == 24'hbfffff) begin
					vdd_en_n	<= 1;
					state		<= 0;
				end
			end

		endcase

    end

endmodule
