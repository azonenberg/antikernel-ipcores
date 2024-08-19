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
	@brief	APB register access to a SPI bus controller

	Includes a control for a single chip select pin. Additional chip selects, if needed, must be provided by
	a separate GPIO block.
 */
module APB_SPIHostInterface(

	//The APB bus
	APB.completer 					apb,

	//SPI interface
	output wire						spi_sck,
	output wire						spi_mosi,
	input wire						spi_miso,
	output logic					spi_cs_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16 or 32 bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH > 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	//Align all writable registers to 0x20 boundaries to work around STM32H7 OCTOSPI bugs
	typedef enum logic[7:0]
	{
		REG_CLK_DIV		= 'h00,		//clock divider from PCLK to SCK
		REG_DATA		= 'h20,		//[7:0] data to send/receive
		REG_CS_N		= 'h40,		//[0] = chip select output value
		REG_STATUS		= 'h60,		//[0] = busy flag
		REG_STATUS_2	= 'h80		//duplicate of REG_STATUS
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The SPI controller

	logic[15:0]	clkdiv	= 100;

	logic		shift_en;
	logic[7:0]	shift_data;
	wire		shift_done;
	wire[7:0]	rx_data;

	SPIHostInterface spi(
		.clk(apb.pclk),
		.clkdiv(clkdiv),

		.spi_sck(spi_sck),
		.spi_mosi(spi_mosi),
		.spi_miso(spi_miso),

		.shift_en(shift_en),
		.shift_done(shift_done),
		.tx_data(shift_data),
		.rx_data(rx_data));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	logic	shift_busy = 0;

	always_comb begin

		//Combinatorially assert PREADY when selected
		apb.pready		= apb.psel && apb.penable;

		//Default to no errors and no read data
		apb.prdata		= 0;
		apb.pslverr		= 0;

		//Clear control signals
		shift_en		= 0;
		shift_data		= 0;

		if(apb.pready) begin

			if(apb.pwrite) begin

				case(apb.paddr)

					//Start a read or write
					REG_DATA: begin
						shift_en	= 1;
						shift_data	= apb.pwdata[7:0];
					end

					//write is sequential, just need to not trigger default case
					REG_CLK_DIV: begin
					end
					REG_CS_N: begin
					end

					//unmapped address
					default:	apb.pslverr		= 1;

				endcase

			end

			else begin

				case(apb.paddr)

					REG_CLK_DIV:	apb.prdata	= clkdiv;
					REG_DATA:		apb.prdata	= {8'h0, rx_data};
					REG_CS_N:		apb.prdata	= {15'h0, spi_cs_n};
					REG_STATUS:		apb.prdata	= {15'h0, shift_busy};
					REG_STATUS_2:	apb.prdata	= {15'h0, shift_busy};

					//unmapped address
					default:		apb.pslverr		= 1;

				endcase

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			spi_cs_n	<= 0;
			clkdiv		<= 100;
			shift_busy	<= 0;
		end

		//Normal path
		else begin

			if(shift_en)
				shift_busy	<= 1;
			if(shift_done)
				shift_busy	<= 0;

			if(apb.pready) begin

				//Writes
				if(apb.pwrite) begin

					case(apb.paddr)
						REG_CS_N:		spi_cs_n	<= apb.pwdata[0];
						REG_CLK_DIV:	clkdiv		<= apb.pwdata[15:0];

						default: begin
						end
					endcase

				end

			end

		end

	end

endmodule
