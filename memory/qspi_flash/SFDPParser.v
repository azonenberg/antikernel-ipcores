`timescale 1ns / 1ps
`default_nettype none
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
	@file
	@author Andrew D. Zonenberg
	@brief Parser for Serial Flash Discoverable Parameters table

	Bring scan_start high for one cycle to start scanning the memory.
	This is brought out externally so that power-on reset delays can be added if needed by the end application.

	When scan_done goes high, the parsed fields are stable and ready to use.
 */
module SFDPParser(

	//Main system clock
	input wire		clk,

	//Scan interface
	//input wire		scan_start,
	output reg		scan_done = 0,

	//SPI interface
	output reg		shift_en	= 0,
	input wire		shift_done,
	output reg[7:0]	spi_tx_data	= 0,
	input wire[7:0]	spi_rx_data,
	output reg		spi_cs_n	= 1,

	//TODO: parsed fields with config info

	//DEBUG LA
	input wire		uart_rxd,
    output wire		uart_txd
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration

	localparam XILINX_7SERIES_CCLK_WORKAROUND	= 1;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Main internal state machine

	//List of JEDEC standard SPI flash instructions
	localparam INST_READ_SFDP		= 8'h5a;
	localparam INST_READ_JEDEC_ID	= 8'h9f;

	localparam STATE_BOOT_WAIT			= 0;
	localparam STATE_BOOT_DUMMY			= 1;
	localparam STATE_BOOT_HEADER_SEL	= 2;
	localparam STATE_BOOT_HEADER_READ_0	= 3;
	localparam STATE_BOOT_HEADER_READ_1	= 4;
	localparam STATE_BOOT_HEADER_PARSE	= 5;

	localparam STATE_BOOT_READ_BASIC	= 6;

	localparam STATE_BOOT_HANG			= 255;

	reg[7:0]	state		= STATE_BOOT_WAIT;
	reg[8:0]	count		= 0;

	reg			sfdp_avail	= 0;
	reg[8:0]	sfdp_addr	= 0;
	reg[7:0]	sfdp_data	= 0;

	reg			phdrs_done	= 0;

	wire		la_ready;

	//Most recent revision number and offset for the JEDEC Basic SPI Flash Parameters table
	reg[15:0]	basic_params_rev		= 0;
	reg[23:0]	basic_params_offset		= 0;
	reg[7:0]	basic_params_len		= 0;

	//Reasons why we failed to parse the descriptor table
	reg			sfdp_bad			= 0;
	reg[7:0]	sfdp_fail_reason	= 0;

	localparam SFDP_REASON_BAD_HEADER			= 8'h1;
	localparam SFDP_REASON_BAD_MAJOR_VERSION	= 8'h2;

    always @(posedge clk) begin
		shift_en	<= 0;
		sfdp_avail	<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BOOT - discover flash parameters

			//Send a dummy byte with CS high to correctly initialize the CCLK output buffer.
			//This is required for 7 series and is harmless in other chips so we do it no matter what.
			//See https://www.xilinx.com/support/answers/52626.html
			STATE_BOOT_WAIT: begin
				if(la_ready) begin
					if(XILINX_7SERIES_CCLK_WORKAROUND) begin
						shift_en		<= 1;
						state			<= STATE_BOOT_DUMMY;
					end
					else
						state			<= STATE_BOOT_HEADER_SEL;
				end
			end	//end STATE_BOOT_WAIT

			//Wait for the dummy send to finish
			STATE_BOOT_DUMMY: begin
				if(shift_done)
					state				<= STATE_BOOT_HEADER_SEL;
			end	//end STATE_BOOT_DUMMY

			//TODO

			//Select the flash
			STATE_BOOT_HEADER_SEL: begin
				count				<= count + 1'h1;
				if(count == 15)
					spi_cs_n		<= 0;
				if(count == 31) begin
					count			<= 0;
					state			<= STATE_BOOT_HEADER_READ_0;
				end
			end	//end STATE_BOOT_HEADER_SEL

			//Send the read command
			STATE_BOOT_HEADER_READ_0: begin
				shift_en		<= 1;
				spi_tx_data		<= INST_READ_SFDP;

				state			<= STATE_BOOT_HEADER_READ_1;
			end	//end STATE_BOOT_HEADER_READ_0

			//When the command finishes, send a dummy word so we can read the response
			STATE_BOOT_HEADER_READ_1: begin
				if(shift_done) begin
					spi_tx_data	<= 0;
					shift_en	<= 1;
					state		<= STATE_BOOT_HEADER_PARSE;
				end
			end	//end STATE_BOOT_HEADER_READ_1

			//Done
			STATE_BOOT_HEADER_PARSE: begin
				if(shift_done) begin

					//Send this word on to get analyzed
					if(count >= 4) begin
						sfdp_addr	<= count - 8'd4;			//remove the four dummy bytes
						sfdp_data	<= spi_rx_data;
						sfdp_avail	<= 1;
					end

					//Go on to the next word
					count		<= count + 1'h1;

					//If we overran the available PHDR space, or got a bad PHDR, die
					if( (sfdp_addr == 8'hff) || sfdp_bad ) begin
						spi_cs_n	<= 1;
						state		<= STATE_BOOT_HANG;
					end

					//If we're done reading the parameter headers, move on
					else if(phdrs_done) begin
						state		<= STATE_BOOT_READ_BASIC;
					end

					//Nope, read another byte of stuff
					else begin
						spi_tx_data	<= 0;
						shift_en	<= 1;
					end
				end
			end	//end STATE_BOOT_HEADER_PARSE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read basic parameters

			STATE_BOOT_READ_BASIC: begin
			end	//end STATE_BOOT_READ_BASIC

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// HANG - bad SFDP info, we can't do anything

			STATE_BOOT_HANG: begin
			end	//end STATE_BOOT_HANG

		endcase
    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Crunch the SFDP data as it comes in

	//SFDP header data. Probably not useful outside this state machine
	reg[7:0]	sfdp_minor_rev			= 0;
	reg[7:0]	sfdp_num_phdrs			= 0;

	//List of parameter IDs
	localparam	PARAM_ID_JEDEC_BASIC			= 16'hff00;	//JEDEC basic SPI flash parameters

	//The SFDP parameter table currently being parsed
	reg			sfdp_param_valid		= 0;
	reg[15:0]	sfdp_current_param		= 0;
	reg[15:0]	sfdp_param_rev			= 0;
	reg[7:0]	sfdp_param_len			= 0;
	reg[23:0]	sfdp_param_offset		= 0;

	wire[5:0]	phdr_num			= sfdp_addr[8:3] - 6'd1;
	wire[2:0]	phdr_boff			= sfdp_addr[2:0];

	localparam SFDP_STATE_READ_HEADER			= 0;
	localparam SFDP_STATE_READ_PHDR				= 1;
	localparam SFDP_STATE_READ_JEDEC_DESC		= 2;

	reg[2:0]	sfdp_state			= SFDP_STATE_READ_HEADER;

    always @(posedge clk) begin

		phdrs_done			<= 0;
		sfdp_param_valid	<= 0;

		//As parameters become valid, see what they are.
		if(sfdp_param_valid) begin

			case(sfdp_current_param)

				//It's JEDEC Basic Flash Parameters
				PARAM_ID_JEDEC_BASIC: begin

					//Major rev must be 1, we only support this.
					//Ignore anything higher.
					if(sfdp_param_rev[15:8] == 8'h1) begin

						//If rev is higher than what we had, this is better.
						if(sfdp_param_rev > basic_params_rev) begin
							basic_params_len	<= sfdp_param_len;
							basic_params_offset	<= sfdp_param_offset;
							basic_params_rev	<= sfdp_param_rev;
						end

					end

				end	//end PARAM_ID_JEDEC_BASIC

			endcase

		end

		if(sfdp_avail) begin

			case(sfdp_state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Master header for the entire SFDP image
				SFDP_STATE_READ_HEADER: begin

					case(sfdp_addr)

						//Signature: "SFDP"
						0: begin
							if(sfdp_data != "S") begin
								sfdp_bad			<= 1;
								sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
							end
						end

						1: begin
							if(sfdp_data != "F") begin
								sfdp_bad			<= 1;
								sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
							end
						end

						2: begin
							if(sfdp_data != "D") begin
								sfdp_bad			<= 1;
								sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
							end
						end

						3: begin
							if(sfdp_data != "P") begin
								sfdp_bad			<= 1;
								sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
							end
						end

						//SFDP spec minor revision
						4: begin
							sfdp_minor_rev			<= sfdp_data;
						end

						//SFDP spec major revision
						5: begin
							if(sfdp_data != 8'h1) begin
								sfdp_bad			<= 1;
								sfdp_fail_reason	<= SFDP_REASON_BAD_MAJOR_VERSION;
							end
						end

						//Number of parameter headers
						6: begin
							sfdp_num_phdrs		<= sfdp_data;
						end

						//Reserved field, ignore (but move on to the parameter headers)
						7: begin
							sfdp_state			<= SFDP_STATE_READ_PHDR;
						end

					endcase

				end	//end SFDP_STATE_READ_HEADER

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Read and parse the parameter headers

				SFDP_STATE_READ_PHDR: begin

					//Crunch parameter headers

					case(phdr_boff)

						//Byte 0: PHDR ID (LSB)
						0:	sfdp_current_param[7:0]		<= sfdp_data;

						//Byte 1/2: PHDR minor/major rev
						1:	sfdp_param_rev[7:0]			<= sfdp_data;
						2:	sfdp_param_rev[15:8]		<= sfdp_data;

						//Byte 3: Length of this parameter table (in DWORDs)
						3:	sfdp_param_len				<= sfdp_data;

						//Byte 4/5/6: parameter offset
						4:	sfdp_param_offset[7:0]		<= sfdp_data;
						5:	sfdp_param_offset[15:8]		<= sfdp_data;
						6:	sfdp_param_offset[23:16]	<= sfdp_data;

						//Byte 7: PHDR ID (MSB)
						7: begin
							sfdp_param_valid			<= 1;
							sfdp_current_param[15:8]	<= sfdp_data;
						end

					endcase

					//If we just read the last byte of the last parameter header,
					//get ready to read the actual PHDR data.
					//Note, sfdp_num_phdrs is 0-based per JESD216 6.2.2!
					if( (phdr_num == sfdp_num_phdrs ) && (phdr_boff == 7) ) begin
						sfdp_state			<= SFDP_STATE_READ_JEDEC_DESC;
					end

				end	//end SFDP_STATE_READ_PHDR

				SFDP_STATE_READ_JEDEC_DESC: begin
				end	//end SFDP_STATE_READ_JEDEC_DESC

			endcase

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The LA

	//intclk is 66.5 MHz for current prototype at last measurement
	RedTinUartWrapper #(
		.WIDTH(256),
		.DEPTH(2048),
		//.UART_CLKDIV(16'd868),	//115200 @ 100 MHz
		.UART_CLKDIV(16'd577),		//115200 @ 66.5 MHz
		.SYMBOL_ROM(
			{
				16384'h0,
				"DEBUGROM", 				8'h0, 8'h01, 8'h00,
				32'd15037,			//period of internal clock, in ps
				32'd2048,			//Capture depth (TODO auto-patch this?)
				32'd256,			//Capture width (TODO auto-patch this?)
				{ "sfdp_avail",				8'h0, 8'h1,  8'h0 },
				{ "sfdp_data",				8'h0, 8'h8,  8'h0 },
				{ "state",					8'h0, 8'h8,  8'h0 },
				{ "sfdp_bad",				8'h0, 8'h1,  8'h0 },
				{ "sfdp_fail_reason",		8'h0, 8'h8,  8'h0 },
				{ "sfdp_minor_rev",			8'h0, 8'h8,  8'h0 },
				{ "sfdp_num_phdrs",			8'h0, 8'h8,  8'h0 },
				{ "phdr_num",				8'h0, 8'h6,  8'h0 },
				{ "phdr_boff",				8'h0, 8'h3,  8'h0 },
				{ "sfdp_state",				8'h0, 8'h3,  8'h0 },
				{ "sfdp_param_valid",		8'h0, 8'h1,  8'h0 },
				{ "sfdp_current_param",		8'h0, 8'h10,  8'h0 },
				{ "sfdp_param_rev",			8'h0, 8'h10,  8'h0 },
				{ "sfdp_param_len",			8'h0, 8'h8,  8'h0 },
				{ "sfdp_param_offset",		8'h0, 8'h18,  8'h0 },
				{ "basic_params_rev",		8'h0, 8'h10,  8'h0 },
				{ "basic_params_len",		8'h0, 8'h8,  8'h0 },
				{ "basic_params_offset",	8'h0, 8'h18,  8'h0 }
			}
		)
	) analyzer (
		.clk(clk),
		.capture_clk(clk),
		.din({
				sfdp_avail,				//1
				sfdp_data,				//8
				state,					//8
				sfdp_bad,				//1
				sfdp_fail_reason,		//8
				sfdp_minor_rev,			//8
				sfdp_num_phdrs,			//8
				phdr_num,				//6
				phdr_boff,				//3
				sfdp_state,				//3
				sfdp_param_valid,		//1
				sfdp_current_param,		//16
				sfdp_param_rev,			//16
				sfdp_param_len,			//8
				sfdp_param_offset,		//24
				basic_params_rev,		//16
				basic_params_len,		//8
				basic_params_offset,	//24

				89'h0					//padding
			}),
		.uart_rx(uart_rxd),
		.uart_tx(uart_txd),
		.la_ready(la_ready)
	);

endmodule
