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
	@brief Controller for quad SPI (or single SPI) NOR Flash memory

	TODO: notes on CS# setup/hold
 */
module QuadSPIFlashController(

	//The main system clock
	input wire				clk,

	//The SPI bus (connected to top-level ports)
	//TODO: have separate tristate enables for each pin?
	output wire				spi_sck,
	inout wire[3:0] 		spi_dq,
	output reg				spi_cs_n 		= 1,

	//Divider for the SPI bus
	input wire[15:0]		clkdiv,

	//Control signals
	input wire				cmd_en,
	input wire[3:0]			cmd_id,
	input wire[15:0]		cmd_len,
	input wire[31:0]		cmd_addr,
	output reg[31:0]		read_data		= 0,
	output reg				read_valid		= 0,
	output wire				busy,

	//DEBUG
	input wire				uart_rxd,
	output wire				uart_txd,
	input wire				start
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration

	//This should be default, probably wont need to get changed unless the memory is weird
	parameter SAMPLE_EDGE = "RISING";
	parameter LOCAL_EDGE = "INVERTED";

	//Set to 0 (default) if the quad pins are connected to the flash on the PCB.
	//If this value is 1, we won't attempt to use quad mode even if the SFDP says it's supported.
	parameter QUAD_DISABLE = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The SPI controller

	assign		spi_dq[1] = 1'bz;	//tri-state MISO

	assign		spi_dq[2] = 1'b1;	//drive high (inactive) for now
	assign		spi_dq[3] = 1'b1;	//drive high (inactive) for now

	reg			shift_en		= 0;
	wire		shift_done;

	reg[7:0]	spi_tx_data		= 0;
	wire[7:0]	spi_rx_data;

	SPITransceiver #(
		.SAMPLE_EDGE("RISING"),
		.LOCAL_EDGE("INVERTED"),
		.CHANGE_ON_DONE(1)
	) txvr (
		.clk(clk),
		.clkdiv(clkdiv),

		.spi_sck(spi_sck),
		.spi_mosi(spi_dq[0]),
		.spi_miso(spi_dq[1]),

		.shift_en(shift_en),
		.shift_done(shift_done),
		.tx_data(spi_tx_data),
		.rx_data(spi_rx_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The SFDP parser

    wire		sfdp_shift_en;
    wire[7:0]	sfdp_tx_data;
    wire		sfdp_cs_n;

    wire		sfdp_scan_done;

    wire		has_3byte_addr;
    wire		has_4byte_addr;

    wire		enter_4b_b7;
    wire		enter_4b_we_b7;
    wire		enter_4b_nvcr;
    wire		enter_4b_dedicated;

	//The parser
    SFDPParser parser(
		.clk(clk),

		.scan_start(la_ready),
		.scan_done(sfdp_scan_done),

		.shift_en(sfdp_shift_en),
		.shift_done(shift_done),
		.spi_tx_data(sfdp_tx_data),
		.spi_rx_data(spi_rx_data),
		.spi_cs_n(sfdp_cs_n),

		.has_3byte_addr(has_3byte_addr),
		.has_4byte_addr(has_4byte_addr),

		.enter_4b_b7(enter_4b_b7),
		.enter_4b_we_b7(enter_4b_we_b7),
		.enter_4b_nvcr(enter_4b_nvcr),
		.enter_4b_dedicated(enter_4b_dedicated),

		.uart_rxd(/*uart_rxd*/),
		.uart_txd(/*uart_txd*/)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Table of standard flash opcodes (not chip dependent)

    localparam		OP_FAST_READ		= 8'h0b;		//Fast read with dummy byte before data

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The actual flash controller state machine

    localparam		STATE_BOOT_INIT		= 4'h0;

    localparam		STATE_IDLE			= 4'h8;			//must be lowest number after BOOT_*
    localparam		STATE_ADDRESS		= 4'h9;
    localparam		STATE_READ_WAIT		= 4'hc;
    localparam		STATE_READ_DUMMY	= 4'hd;
    localparam		STATE_READ			= 4'he;
    localparam		STATE_WAIT			= 4'hf;

    reg[3:0]		state			= STATE_BOOT_INIT;
    reg[3:0]		state_ret		= STATE_IDLE;
    reg[15:0]		count			= 0;

	//Runtime configuration
	reg				use_4byte_addr	= 0;

    wire			booting			= (state < STATE_IDLE);
    assign			busy			= (state != STATE_IDLE);
    wire			waiting			= (state == STATE_WAIT);

    wire			la_ready;

    always @(posedge clk) begin

		shift_en	<= 0;
		read_valid	<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BOOT - wait for SFDP parser to run

			STATE_BOOT_INIT: begin

				//Send a shift request if the SFDP core wants it
				spi_cs_n					<= sfdp_cs_n;
				if(sfdp_shift_en) begin
					shift_en				<= 1;
					spi_tx_data				<= sfdp_tx_data;
				end

				if(sfdp_scan_done && la_ready) begin

					//Default to being done
					state				<= STATE_IDLE;

					//If we have only 3- or 4-byte addresses, that makes it easy - use that mode
					if(has_3byte_addr && !has_4byte_addr)
						use_4byte_addr	<= 0;
					else if(!has_3byte_addr && has_4byte_addr)
						use_4byte_addr	<= 1;

					//If we have both, always use 4-byte addresses.
					//But there's more than one way to do this!
					//TODO: prefer use of dedicated instruction set if available for faster boot?
					//TODO: once we've enabled the right addressing mode, enter quad mode if supported?
					else if(enter_4b_b7)begin
						use_4byte_addr	<= 1;

						spi_cs_n		<= 0;
						spi_tx_data		<= 8'hb7;
						shift_en		<= 1;

						state			<= STATE_WAIT;
						state_ret		<= STATE_IDLE;
					end

				end

			end	//end STATE_BOOT_INIT

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for stuff to happen

			STATE_IDLE: begin
				spi_cs_n				<= 1;
				count					<= 0;

				if(cmd_en) begin

					//Assume this is a read (TODO opcode table)
					if(cmd_id == 0) begin

						spi_cs_n			<= 0;

						//Send the opcode
						spi_tx_data			<= OP_FAST_READ;

						//Skip the MSB of the address in 24-bit mode
						if(use_4byte_addr)
							count			<= 0;
						else
							count			<= 1;

						shift_en			<= 1;
						state				<= STATE_ADDRESS;

					end

				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// ADDRESS - send remaining address bytes

			STATE_ADDRESS: begin

				if(shift_done) begin

					count				<= count + 1'h1;
					shift_en			<= 1;

					case(count)
						0:	spi_tx_data <= cmd_addr[31:24];
						1:	spi_tx_data <= cmd_addr[23:16];
						2:	spi_tx_data <= cmd_addr[15:8];
						3:	spi_tx_data <= cmd_addr[7:0];
					endcase

					if(count == 3)
						state			<= STATE_READ_WAIT;

				end

			end	//end STATE_ADDRESS

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// READ - read data bytes

			//Wait for the last address byte to be sent
			STATE_READ_WAIT: begin

				if(shift_done) begin
					count				<= 0;
					spi_tx_data			<= 0;
					shift_en			<= 1;
					state				<= STATE_READ_DUMMY;
				end

			end	//end STATE_READ_WAIT

			//Send the dummy byte
			STATE_READ_DUMMY: begin
				if(shift_done) begin
					spi_tx_data			<= 0;
					shift_en			<= 1;
					state				<= STATE_READ;
				end
			end	//end STATE_READ_DUMMY

			//Do the actual read
			STATE_READ: begin
				if(shift_done) begin
					count				<= count + 1'h1;

					//Report results if we're done with a byte (vs the dummy)
					if(count != 0) begin
						read_valid		<= 1;
						read_data		<= spi_rx_data;
					end

					//Done
					if(count == cmd_len) begin
						state			<= STATE_IDLE;
					end

					//Not done, read more
					else begin
						spi_tx_data		<= 0;
						shift_en		<= 1;
					end

				end
			end	//end STATE_READ

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// WAIT - wait for the current SPI transaction to finish, then bring CS high and return

			STATE_WAIT: begin
				if(shift_done) begin
					spi_cs_n			<= 1;
					state				<= state_ret;
				end
			end	//end STATE_WAIT

		endcase

    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Our logic analyzer

    //intclk is 66.5 MHz for current prototype at last measurement
	RedTinUartWrapper #(
		.WIDTH(128),
		.DEPTH(2048),
		.UART_CLKDIV(16'd577),		//115200 @ 66.5 MHz
		.USE_EXT_TRIG(1),
		.SYMBOL_ROM(
			{
				16384'h0,
				"DEBUGROM", 				8'h0, 8'h01, 8'h00,
				32'd15037,			//period of internal clock, in ps
				32'd2048,			//Capture depth (TODO auto-patch this?)
				32'd128,			//Capture width (TODO auto-patch this?)
				{ "state",					8'h0, 8'h4,  8'h0 },
				{ "booting",				8'h0, 8'h1,  8'h0 },
				{ "waiting",				8'h0, 8'h1,  8'h0 },
				{ "busy",					8'h0, 8'h1,  8'h0 },
				{ "sfdp_scan_done",			8'h0, 8'h1,  8'h0 },
				{ "has_3byte_addr",			8'h0, 8'h1,  8'h0 },
				{ "has_4byte_addr",			8'h0, 8'h1,  8'h0 },
				{ "use_4byte_addr",			8'h0, 8'h1,  8'h0 },

				{ "enter_4b_b7",			8'h0, 8'h1,  8'h0 },
				{ "enter_4b_we_b7",			8'h0, 8'h1,  8'h0 },
				{ "enter_4b_nvcr",			8'h0, 8'h1,  8'h0 },
				{ "enter_4b_dedicated",		8'h0, 8'h1,  8'h0 },

				{ "cmd_en",					8'h0, 8'h1,  8'h0 },
				{ "read_valid",				8'h0, 8'h1,  8'h0 },
				{ "cmd_id",					8'h0, 8'h4,  8'h0 },
				{ "cmd_len",				8'h0, 8'h10,  8'h0 },
				{ "cmd_addr",				8'h0, 8'h20,  8'h0 },
				{ "read_data",				8'h0, 8'h20,  8'h0 },

				{ "spi_cs_n",				8'h0, 8'h1,  8'h0 },
				{ "shift_en",				8'h0, 8'h1,  8'h0 },
				{ "spi_tx_data",			8'h0, 8'h8,  8'h0 },
				{ "spi_rx_data",			8'h0, 8'h8,  8'h0 },
				{ "shift_done",				8'h0, 8'h1,  8'h0 }
			}
		)
	) analyzer (
		.clk(clk),
		.capture_clk(clk),
		.ext_trig(sfdp_scan_done),
		.din({
				state,					//4
				booting,				//1
				waiting,				//1
				busy,					//1
				sfdp_scan_done,			//1
				has_3byte_addr,			//1
				has_4byte_addr,			//1
				use_4byte_addr,			//1

				enter_4b_b7,			//1
				enter_4b_we_b7,			//1
				enter_4b_nvcr,			//1
				enter_4b_dedicated,		//1

				cmd_en,					//1
				read_valid,				//1
				cmd_id,					//4
				cmd_len,				//16
				cmd_addr,				//32
				read_data,				//32

				spi_cs_n,				//1
				shift_en,				//1
				spi_tx_data,			//8
				spi_rx_data,			//8
				shift_done,				//1

				8'h0					//padding
			}),
		.uart_rx(uart_rxd),
		.uart_tx(uart_txd),
		.la_ready(la_ready)
	);

	//assign la_ready = start;

endmodule
