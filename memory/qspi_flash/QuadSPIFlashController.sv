`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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

`include "QuadSPIFlashController_opcodes.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief Controller for quad SPI (or single SPI) NOR Flash memory

	TODO: notes on CS# setup/hold

	To send a command:
		Set cmd_id/len/addr
		Assert cmd_en
		Wait for busy to go low. Do not change cmd_* during this time.

	Read data:
		When each byte of data comes in, read_valid will go high for 1 clock with read_data set appropriately

	Write data
		When the controller is ready for another byte of data, write_ready will go high for 1 clock.
		The host must set write_data and assert write_valid for 1 clock to proceed.
		Do not change write_data until write_valid goes high again.
 */
module QuadSPIFlashController(

	//The main system clock
	input wire					clk,

	//The SPI bus (connected to top-level ports)
	//TODO: have separate tristate enables for each pin?
	output wire					spi_sck,
	inout wire[3:0] 			spi_dq,
	output logic				spi_cs_n 		= 1,

	//Divider for the SPI bus
	input wire[15:0]			clkdiv,

	//Control signals
	input wire					cmd_en,
	input wire qspi_opcode_t	cmd_id,
	input wire[15:0]			cmd_len,
	input wire[31:0]			cmd_addr,
	output logic[7:0]			read_data		= 0,
	output logic				read_valid		= 0,
	input wire[7:0]				write_data,
	input wire					write_valid,
	output logic				write_ready,
	output wire					busy,

	//Useful info
	output wire[15:0]			capacity_mbits,

	//Debug status
	output wire					sfdp_bad
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

	//TODO: full quad support

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

    wire[7:0]	erase_type2_insn;
    wire[15:0]	erase_type2_kbits;

	//The parser
	wire		trig_out = 1;	//debug logic analyzer trigger, tie to 1 if not using ILA to observe boot

    SFDPParser parser(
		.clk(clk),

		.scan_start(1'b1),
		.scan_done(sfdp_scan_done),
		.sfdp_bad(sfdp_bad),

		.shift_en(sfdp_shift_en),
		.shift_done(shift_done),
		.spi_tx_data(sfdp_tx_data),
		.spi_rx_data(spi_rx_data),
		.spi_cs_n(sfdp_cs_n),

		.capacity_mbits(capacity_mbits),

		.has_3byte_addr(has_3byte_addr),
		.has_4byte_addr(has_4byte_addr),

		.enter_4b_b7(enter_4b_b7),
		.enter_4b_we_b7(enter_4b_we_b7),
		.enter_4b_nvcr(enter_4b_nvcr),
		.enter_4b_dedicated(enter_4b_dedicated),

		.erase_type2_insn(erase_type2_insn),
		.erase_type2_kbits(erase_type2_kbits),

		.la_ready(trig_out)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Table of standard flash opcodes (not chip dependent)

    typedef enum logic[7:0]
    {
		OP_PAGE_PROGRAM		= 8'h02,	//Write up to one 256-byte page of data
										//TODO: allow >256 byte writes, chip dependent?
										//Some have 512 byte etc page size
		OP_READ_SR1			= 8'h05,	//Read status register 1
		OP_WRITE_ENABLE		= 8'h06,	//Enable write operations
		OP_FAST_READ		= 8'h0b,	//Fast read with dummy byte before data
		OP_WRITE_SR3		= 8'h11,	//Write status register 3
		OP_BULK_ERASE		= 8'h60,	//Full-chip erase
		OP_SECTOR_ERASE		= 8'hd8		//Sector erase (size variable)

    } flash_opcode_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The actual flash controller state machine

    typedef enum logic[4:0]
    {
		STATE_BOOT_INIT		= 5'h0,
		STATE_BOOT_SRWE		= 5'h1,
		STATE_BOOT_SRWRITE	= 5'h2,
		STATE_BOOT_SRDATA	= 5'h3,

		STATE_IDLE			= 5'h4,			//must be lowest number after BOOT_*

		STATE_ERASE_WAIT	= 5'h5,
		STATE_POLL_0		= 5'h6,
		STATE_POLL_1		= 5'h7,
		STATE_POLL_2		= 5'h8,

		STATE_WRITE			= 5'h9,
		STATE_WRITE_ENABLE	= 5'ha,

		STATE_READ_WAIT		= 5'h1b,
		STATE_READ			= 5'h1c,

		STATE_ADDRESS		= 5'h1d,
		STATE_DONE			= 5'h1e,
		STATE_WAIT			= 5'h1f
    } flash_state_t;

	flash_state_t	state			= STATE_BOOT_INIT;
	flash_state_t	state_ret		= STATE_IDLE;
	logic[15:0]		count			= 0;

	//Runtime configuration
	logic			use_4byte_addr	= 0;

	wire			booting			= (state < STATE_IDLE) || ( (state == STATE_WAIT) && (state_ret < STATE_IDLE) );
	assign			busy			= (state != STATE_IDLE);
	wire			waiting			= (state == STATE_WAIT);

	logic			write_buf_valid	= 0;

	//Check if the SPI transceiver is currently busy
	logic			shift_active_ff	= 0;
	wire			shift_active	= (shift_active_ff && !shift_done) || shift_en;

    always_ff @(posedge clk) begin

		shift_en	<= 0;
		read_valid	<= 0;
		write_ready	<= 0;

		if(write_valid)
			write_buf_valid		<= 1;
		if(shift_en)
			shift_active_ff		<= 1;
		if(shift_done)
			shift_active_ff		<= 0;

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

				if(sfdp_scan_done && trig_out) begin

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
					//TODO: discover erase block size more properly
					else if(enter_4b_b7)begin
						use_4byte_addr	<= 1;

						spi_cs_n		<= 0;
						spi_tx_data		<= 8'hb7;
						shift_en		<= 1;

						//After the write finishes, enable uniform size sector erase
						//TODO: this is chip specific, read the sector map!!!
						state_ret		<= STATE_BOOT_SRWE;
						state			<= STATE_WAIT;
					end

				end

			end	//end STATE_BOOT_INIT

			STATE_BOOT_SRWE: begin
				spi_cs_n				<= 0;
				spi_tx_data				<= OP_WRITE_ENABLE;
				shift_en				<= 1;

				state_ret				<= STATE_BOOT_SRWRITE;
				state					<= STATE_WAIT;

			end	//end STATE_BOOT_SRWE

			STATE_BOOT_SRWRITE: begin
				spi_cs_n				<= 0;
				spi_tx_data				<= OP_WRITE_SR3;
				shift_en				<= 1;

				state					<= STATE_BOOT_SRDATA;
			end	//end STATE_BOOT_SRWRITE

			STATE_BOOT_SRDATA: begin
				if(shift_done) begin
					spi_tx_data			<= 8'h20;
					shift_en			<= 1;

					state				<= STATE_WAIT;
					state_ret			<= STATE_IDLE;
				end
			end	//end STATE_BOOT_SRDATA

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for stuff to happen

			STATE_IDLE: begin
				spi_cs_n				<= 1;

				//Prepare the address counter in case we're starting a command this cycle.
				//Skip the MSB of the address in 24-bit mode
				if(use_4byte_addr)
					count			<= 0;
				else
					count			<= 1;

				if(cmd_en) begin

					//See what we're doing
					case(cmd_id)

						FLASH_OP_READ: begin

							spi_cs_n			<= 0;

							//Send the opcode
							spi_tx_data			<= OP_FAST_READ;
							shift_en			<= 1;

							state				<= STATE_ADDRESS;
							state_ret			<= STATE_READ_WAIT;

						end	//end FLASH_OP_READ

						FLASH_OP_ERASE: begin

							//Send the write-enable command
							spi_cs_n			<= 0;
							spi_tx_data			<= OP_WRITE_ENABLE;
							shift_en			<= 1;
							state				<= STATE_WAIT;
							state_ret			<= STATE_WRITE_ENABLE;

						end	//end FLASH_OP_ERASE

						FLASH_OP_PROGRAM: begin

							//Send the write-enable command
							spi_cs_n			<= 0;
							spi_tx_data			<= OP_WRITE_ENABLE;
							shift_en			<= 1;
							state				<= STATE_WAIT;
							state_ret			<= STATE_WRITE_ENABLE;

						end	//end FLASH_OP_PROGRAM

						FLASH_OP_WIPE: begin

							//Send the write-enable command
							spi_cs_n			<= 0;
							spi_tx_data			<= OP_WRITE_ENABLE;
							shift_en			<= 1;
							state				<= STATE_WAIT;
							state_ret			<= STATE_WRITE_ENABLE;

						end	//end FLASH_OP_WIPE

					endcase

				end

			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// WRITE ENABLE - wait for write enable command then dispatch the actual write

			STATE_WRITE_ENABLE: begin

				spi_cs_n			<= 0;

				//Set up the address counter. Skip the MSB in 24-bit mode.
				if(use_4byte_addr)
					count			<= 0;
				else
					count			<= 1;

				case(cmd_id)

					FLASH_OP_ERASE: begin

						//TODO: send the opcode based on cmd_len?
						if(erase_type2_insn == 8'hff)
							spi_tx_data		<= OP_SECTOR_ERASE;
						else
							spi_tx_data		<= erase_type2_insn;

						shift_en			<= 1;

						state				<= STATE_ADDRESS;
						state_ret			<= STATE_ERASE_WAIT;

					end	//end FLASH_OP_ERASE

					FLASH_OP_WIPE: begin

						spi_tx_data			<= OP_BULK_ERASE;
						shift_en			<= 1;

						state				<= STATE_ERASE_WAIT;

					end	//end FLASH_OP_WIPE

					FLASH_OP_PROGRAM: begin

						spi_cs_n			<= 0;

						//Send the opcode
						spi_tx_data			<= OP_PAGE_PROGRAM;
						shift_en			<= 1;

						state				<= STATE_ADDRESS;
						state_ret			<= STATE_WRITE;

						//We need a data byte to send
						write_ready			<= 1;

					end	//end FLASH_OP_PROGRAM

					//should never get here
					default: begin
						state					<= STATE_IDLE;
					end

				endcase

			end	//end STATE_WRITE_ENABLE

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

					if(count == 3) begin
						count			<= 0;
						state			<= state_ret;
					end

				end

			end	//end STATE_ADDRESS

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// WRITE - write data to previously erased memory cells

			STATE_WRITE: begin

				if(shift_done) begin

					//If we just sent the last byte, we're done
					if(count == cmd_len) begin
						count			<= 0;
						state_ret		<= STATE_POLL_0;
						state			<= STATE_DONE;
						write_buf_valid	<= 0;
					end

					//Send the next byte when we finish the previous send.
					//TODO: handle delayed arrival of write data?
					else if(write_buf_valid) begin
						spi_tx_data		<= write_data;
						shift_en		<= 1;
						count			<= count + 1'h1;

						//Buffer is empty and we're ready for more data
						write_buf_valid	<= 0;
						write_ready 	<= 1;
					end

				end

			end	//end STATE_WRITE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// READ - read data bytes

			//Wait for the last address byte to be sent, then send the dummy byte
			STATE_READ_WAIT: begin

				if(shift_done) begin
					count				<= 0;
					spi_tx_data			<= 0;
					shift_en			<= 1;
					state				<= STATE_READ;
				end

			end	//end STATE_READ_WAIT

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
						state_ret		<= STATE_IDLE;
						state			<= STATE_DONE;
					end

					//Not done, read more
					else begin
						spi_tx_data		<= 0;
						shift_en		<= 1;
					end

				end

			end	//end STATE_READ

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// ERASE - erase one sector (or the whole chip)

			//Wait for the last byte of the address to get sent, then deselect
			STATE_ERASE_WAIT: begin
				if(shift_done) begin
					spi_cs_n			<= 1;
					count				<= 0;
					state				<= STATE_POLL_0;
				end
			end	//end STATE_ERASE_WAIT

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// POLL - wait for "busy" bit to be cleared

			//Wait 4 clocks, then re-select and poll the status register
			STATE_POLL_0: begin
				count					<= count + 1'h1;
				if(count == 8'h3) begin
					spi_cs_n			<= 0;
					shift_en			<= 1;
					spi_tx_data			<= OP_READ_SR1;
					state				<= STATE_POLL_1;
				end
			end	//end STATE_POLL_0

			//Send an 0x00 byte to read the status register
			STATE_POLL_1: begin
				if(shift_done) begin
					shift_en			<= 1;
					spi_tx_data			<= 8'h00;
					state				<= STATE_POLL_2;
				end
			end	//end STATE_POLL_1

			//If the busy bit (LSB of SR1) is still set, go back and poll again.
			//If not, erase is complete
			STATE_POLL_2: begin
				if(shift_done) begin

					//Still busy, poll again.
					//We can just keep sending dummy bytes to poll the same status register over and over.
					if(spi_rx_data[0]) begin
						spi_tx_data			<= 8'h00;
						shift_en			<= 1;
					end

					//Not busy, we're finished
					else begin
						spi_cs_n		<= 1;
						state_ret		<= STATE_IDLE;
						state			<= STATE_DONE;
					end

				end
			end	//end STATE_POLL_2

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// DONE - bring CS high and wait 4 clocks, then finish

			STATE_DONE: begin
				if(spi_cs_n) begin
					count				<= count + 1'h1;
					if(count == 3) begin
						count			<= 0;
						state			<= state_ret;
					end
				end
				else begin
					count				<= 0;
					spi_cs_n			<= 1;
				end
			end	//end STATE_DONE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// WAIT - wait for the current SPI transaction to finish, then bring CS high and return after 4 clocks

			STATE_WAIT: begin
				if(shift_done) begin
					count				<= 0;
					spi_cs_n			<= 1;
				end
				else if(spi_cs_n) begin
					count				<= count + 1'h1;
					if(count == 3) begin
						count			<= 0;
						state			<= state_ret;
					end
				end
			end	//end STATE_WAIT

		endcase

    end

endmodule
