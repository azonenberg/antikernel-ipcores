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
    // Flash read-request logic

    //List of JEDEC standard SPI flash instructions
	localparam INST_READ_SFDP		= 8'h5a;

	//Indicates we're busy and not able to accept commands
    reg			read_busy				= 1;

	//Request a read from the given address
    reg			read_request			= 0;
    reg[23:0]	read_addr				= 0;

	//Request termination of the read
	//(it's done when read_busy goes low)
    reg			read_finish_request		= 0;
    reg			read_finish_pending		= 0;

	//Read state machine
    localparam	READ_STATE_INIT_0		= 0;
    localparam	READ_STATE_INIT_1		= 1;
    localparam	READ_STATE_IDLE			= 2;
    localparam	READ_STATE_COMMAND		= 3;
    localparam	READ_STATE_ADDR			= 4;
    localparam	READ_STATE_DUMMY		= 5;
    localparam	READ_STATE_DATA			= 6;

	reg[8:0]	count					= 0;
	reg			read_done				= 0;
	reg[7:0]	read_data				= 0;
    reg[2:0]	read_state				= READ_STATE_INIT_0;

    always @(posedge clk) begin
		shift_en		<= 0;
		read_done		<= 0;

		if(read_finish_request)
			read_finish_pending		<= 1;

		case(read_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// INIT - get ready to do stuff (FPGA-specific bug workarounds)

			READ_STATE_INIT_0: begin

				//Send a dummy byte with CS high to correctly initialize the CCLK output buffer.
				//This is required for 7 series and is harmless in other chips so we do it no matter what.
				//See https://www.xilinx.com/support/answers/52626.html
				if(XILINX_7SERIES_CCLK_WORKAROUND) begin
					shift_en		<= 1;
					read_state		<= READ_STATE_INIT_1;
				end

				//Never going to target 7 series? Skip those states and make the logic a tad bit simpler
				else begin
					read_state		<= READ_STATE_IDLE;
					read_busy		<= 0;
				end

			end	//end READ_STATE_INIT_0

			READ_STATE_INIT_1: begin
				if(shift_done) begin
					read_busy		<= 0;
					read_state		<= READ_STATE_IDLE;
				end
			end	//end READ_STATE_INIT_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - sit around and wait for commands

			READ_STATE_IDLE: begin

				if(read_request) begin
					spi_cs_n		<= 0;
					read_busy		<= 1;
					read_state		<= READ_STATE_COMMAND;
				end

			end	//end READ_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// COMMAND / ADDR / DUMMY - send the "read SFDP" instruction followed by the address and a dummy word

			READ_STATE_COMMAND: begin

				shift_en			<= 1;
				spi_tx_data			<= INST_READ_SFDP;
				read_state			<= READ_STATE_ADDR;
				count				<= 0;

			end	//end READ_STATE_COMMAND

			READ_STATE_ADDR: begin
				if(shift_done) begin

					count			<= count + 1'h1;

					shift_en		<= 1;

					case(count)
						0:	spi_tx_data		<= read_addr[23:16];
						1:	spi_tx_data		<= read_addr[15:8];
						2:	spi_tx_data		<= read_addr[7:0];
						3:	spi_tx_data		<= 0;	//dummy word
					endcase

					if(count == 3)
						read_state	<= READ_STATE_DUMMY;

				end
			end	//end READ_STATE_ADDR

			READ_STATE_DUMMY: begin
				if(shift_done) begin
					shift_en		<= 1;
					spi_tx_data		<= 0;
					read_state		<= READ_STATE_DATA;
				end
			end	//end READ_STATE_DUMMY

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// DATA - push out zeroes while waiting for data to come back

			READ_STATE_DATA: begin

				if(shift_done) begin

					read_done			<= 1;
					read_data			<= spi_rx_data;

					if(read_finish_pending) begin
						spi_cs_n			<= 1;
						read_busy			<= 0;
						read_finish_pending	<= 0;
						read_state			<= READ_STATE_IDLE;
					end

					else begin
						shift_en		<= 1;
						spi_tx_data		<= 0;
					end
				end

			end	//end READ_STATE_DATA

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Master state machine: read header then each block we care about

	localparam STATE_BOOT_WAIT		= 0;
	localparam STATE_HEADER_PARSE	= 1;
	localparam STATE_READ_BASIC		= 2;
	localparam STATE_PARSE_BASIC	= 3;

	localparam STATE_HANG			= 255;

	reg[7:0]	state		= STATE_BOOT_WAIT;

	reg			phdrs_done	= 0;

	wire		la_ready;

	//Most recent revision number and offset for the JEDEC Basic SPI Flash Parameters table
	reg[15:0]	basic_params_rev		= 0;
	reg[23:0]	basic_params_offset		= 0;
	reg[7:0]	basic_params_len		= 0;

	reg[8:0]	sfdp_addr			= 0;
	wire[5:0]	phdr_num			= sfdp_addr[8:3] - 6'd1;
	wire[2:0]	phdr_boff			= sfdp_addr[2:0];

	//A few helpers from the top-level state machine
	wire		parsing_master_header	= (state == STATE_HEADER_PARSE);
	wire		parsing_basic_params	= (state == STATE_PARSE_BASIC);

	//Reasons why we failed to parse the descriptor table
	reg			sfdp_bad			= 0;
	reg[7:0]	sfdp_fail_reason	= 0;

	localparam SFDP_REASON_BAD_HEADER			= 8'h1;
	localparam SFDP_REASON_BAD_MAJOR_VERSION	= 8'h2;

    always @(posedge clk) begin
		read_request		<= 0;
		read_finish_request	<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BOOT - discover flash parameters

			//Send a dummy byte with CS high to correctly initialize the CCLK output buffer.
			//This is required for 7 series and is harmless in other chips so we do it no matter what.
			//See https://www.xilinx.com/support/answers/52626.html
			STATE_BOOT_WAIT: begin
				if(la_ready && !read_busy) begin
					read_request		<= 1;
					read_addr			<= 0;
					sfdp_addr			<= 0;		//max, will overflow to zero
					state				<= STATE_HEADER_PARSE;
				end
			end	//end STATE_BOOT_WAIT

			STATE_HEADER_PARSE: begin

				if(read_done) begin
					sfdp_addr				<= sfdp_addr + 1'h1;

					//If we got a bad PHDR, die
					if(sfdp_bad) begin
						state				<= STATE_HANG;
						read_finish_request	<= 1;
					end

				end

				//If we're done reading the parameter headers, move on
				if( (phdr_num == sfdp_num_phdrs ) && (phdr_boff == 7) ) begin
					read_finish_request	<= 1;
					state				<= STATE_READ_BASIC;
				end

			end	//end STATE_BOOT_HEADER_PARSE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read basic parameters

			STATE_READ_BASIC: begin
				if(!read_busy) begin
					read_request		<= 1;
					read_addr			<= basic_params_offset;
					state				<= STATE_PARSE_BASIC;

					sfdp_addr			<= 9'h0;
				end
			end	//end STATE_READ_BASIC

			STATE_PARSE_BASIC: begin
				if(read_done) begin

					sfdp_addr			<= sfdp_addr + 1'h1;

					//If we just read the last byte of the descriptor, we're done.
					//Note that basic_params_len is a count of DWORDs so we have to multiply by 4!
					if( (basic_params_len == sfdp_addr[8:2]) && (sfdp_addr[1:0] == 2'h3) ) begin
						read_finish_request	<= 1;
						state				<= STATE_HANG;
					end

				end
			end	//end STATE_PARSE_BASIC

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// HANG - bad SFDP info, we can't do anything

			STATE_HANG: begin
			end	//end STATE_HANG

		endcase
    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Parse the parameter headers

	//SFDP header data. Probably not useful outside this state machine
	reg[7:0]	sfdp_minor_rev			= 0;
	reg[7:0]	sfdp_num_phdrs			= 0;

	//List of parameter IDs
	localparam	PARAM_ID_JEDEC_BASIC			= 16'hff00;	//JEDEC basic SPI flash parameters
	localparam	PARAM_ID_JEDEC_SECTOR_MAP		= 16'hff81;	//TODO
	localparam	PARAM_ID_JEDEC_FOUR_BYTE		= 16'hff84;	//TODO

	//The SFDP parameter table currently being parsed
	reg			sfdp_param_valid		= 0;
	reg[15:0]	sfdp_current_param		= 0;
	reg[15:0]	sfdp_param_rev			= 0;
	reg[7:0]	sfdp_param_len			= 0;
	reg[23:0]	sfdp_param_offset		= 0;

	localparam SFDP_STATE_READ_HEADER			= 0;
	localparam SFDP_STATE_READ_PHDR				= 1;

	reg[2:0]	sfdp_state			= SFDP_STATE_READ_HEADER;

    always @(posedge clk) begin

		sfdp_param_valid	<= 0;

		if(parsing_master_header) begin

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

			if(read_done) begin

				case(sfdp_state)

					////////////////////////////////////////////////////////////////////////////////////////////////////////
					// Master header for the entire SFDP image
					SFDP_STATE_READ_HEADER: begin

						case(sfdp_addr)

							//Signature: "SFDP"
							0: begin
								if(read_data != "S") begin
									sfdp_bad			<= 1;
									sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
								end
							end

							1: begin
								if(read_data != "F") begin
									sfdp_bad			<= 1;
									sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
								end
							end

							2: begin
								if(read_data != "D") begin
									sfdp_bad			<= 1;
									sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
								end
							end

							3: begin
								if(read_data != "P") begin
									sfdp_bad			<= 1;
									sfdp_fail_reason	<= SFDP_REASON_BAD_HEADER;
								end
							end

							//SFDP spec minor revision
							4: begin
								sfdp_minor_rev			<= read_data;
							end

							//SFDP spec major revision
							5: begin
								if(read_data != 8'h1) begin
									sfdp_bad			<= 1;
									sfdp_fail_reason	<= SFDP_REASON_BAD_MAJOR_VERSION;
								end
							end

							//Number of parameter headers
							6: begin
								sfdp_num_phdrs		<= read_data;
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
							0:	sfdp_current_param[7:0]		<= read_data;

							//Byte 1/2: PHDR minor/major rev
							1:	sfdp_param_rev[7:0]			<= read_data;
							2:	sfdp_param_rev[15:8]		<= read_data;

							//Byte 3: Length of this parameter table (in DWORDs)
							3:	sfdp_param_len				<= read_data;

							//Byte 4/5/6: parameter offset
							4:	sfdp_param_offset[7:0]		<= read_data;
							5:	sfdp_param_offset[15:8]		<= read_data;
							6:	sfdp_param_offset[23:16]	<= read_data;

							//Byte 7: PHDR ID (MSB)
							7: begin
								sfdp_param_valid			<= 1;
								sfdp_current_param[15:8]	<= read_data;
							end

						endcase

					end	//end SFDP_STATE_READ_PHDR

				endcase

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // What we've all been waiting for: the actual output of the parsing!

    //Addressing
    reg			has_3byte_addr		= 0;
    reg			has_4byte_addr		= 0;

	//Clocking
    reg			has_ddr_mode		= 0;

	//Memory size
    reg[15:0]	capacity_mbits		= 0;

	//Fast read (1-1-4)
    reg			has_114_read		= 0;
    reg[7:0]	insn_114_read		= 0;
    reg[2:0]	modeclk_114_read	= 0;
    reg[4:0]	dummyclk_114_read	= 0;

	//Fast read (1-4-4)
    reg			has_144_read		= 0;
    reg[7:0]	insn_144_read		= 0;
    reg[2:0]	modeclk_144_read	= 0;
    reg[4:0]	dummyclk_144_read	= 0;

	//Fast read (4-4-4)
    reg			has_444_read		= 0;
    reg[7:0]	insn_444_read		= 0;
    reg[2:0]	modeclk_444_read	= 0;
    reg[4:0]	dummyclk_444_read	= 0;

    //Erasing
    reg[7:0]	erase_type1_insn	= 0;
    reg[15:0]	erase_type1_kbits	= 0;
    reg[15:0]	erase_type1_ms		= 0;	//typical delay

    reg[7:0]	erase_type2_insn	= 0;
    reg[15:0]	erase_type2_kbits	= 0;
    reg[15:0]	erase_type2_ms		= 0;

    reg[7:0]	erase_type3_insn	= 0;
    reg[15:0]	erase_type3_kbits	= 0;
    reg[15:0]	erase_type3_ms		= 0;

    reg[7:0]	erase_type4_insn	= 0;
    reg[15:0]	erase_type4_kbits	= 0;
    reg[15:0]	erase_type4_ms		= 0;

    reg[31:0]	erase_chip_ms			= 0;

    //Programming
    reg[6:0]	program_first_byte_us	= 0;
    reg[6:0]	program_per_byte_us		= 0;
    reg[15:0]	program_page_us			= 0;
    reg[15:0]	page_size_bytes			= 0;

    //Time multipliers
    reg[4:0]	erase_typ_to_max	= 0;
    reg[4:0]	program_typ_to_max	= 0;

    //Status register polling
    reg			busy_poll_via_flagstatreg	= 0;	//0x70
    reg			busy_poll_via_statreg		= 0;	//0x05

	/*
		Quad enable bit
			Read status register 1 with 0x05

			0	No QE bit, just use opcode to select.
				Need to leave DQ3 high between instructions.

			1	QE is bit 1 of status register 2.
				Set via 2-byte Write Status (0x01) command

			2	QE is bit 6 of status register 1.
				Set via 1-byte Write Status command

			3	QE is bit 7 of status register 2.
				Set via 1-byte Write Status Register 2 command
				Read status register with 0x3E

			4	QE is bit 1 of status register 2.
				Set via 2-byte Write Status command.
				Same as state 1 except for truncated write handling

			5	QE is bit 1 of status register 2.
				Read with 0x35
				Write with 2-byte Write Status command

			6/7	Reserved
	 */
    reg[2:0]	quad_enable_type			= 0;

	//Methods of entering 4-4-4 mode
    reg			quad_qe_38	= 0;			//Set QE, then 0x38
    reg			quad_38		= 0;			//Issue 0x38
    reg			quad_35		= 0;			//Issue 0x35
    reg			quad_6561	= 0;			//Issue 0x65 then 61 read-modify-write
    reg			quad_6571	= 0;			//Issue 0x65/address then 71/address read-modify-write

    //Methods of leaving 4-4-4 mode
    reg			qexit_ff	= 0;			//Issue 0xff
    reg			qexit_f5	= 0;			//Issue 0xf5
    reg			qexit_reset	= 0;			//Issue 0x66/99 to reset
    reg			qexit_6571	= 0;			//Issue 0x65/address then 71/address read-modify-write

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Parse the basic flash parameter table

	reg[6:0]	basic_dword_addr	= 0;
    reg[31:0]	basic_dword			= 0;
    reg			basic_dword_valid	= 0;

	//Carve out a few bytes and name them
    wire[7:0]	basic_dword_b3		= basic_dword[23:16];
    wire[7:0]	basic_dword_b0		= basic_dword[7:0];

	//Subtract 10 with saturation to zero (for erase block info)
    reg[7:0]	basic_dword_b3_m10	= 0;
    reg[7:0]	basic_dword_b0_m10	= 0;
    always @(*) begin
		if(basic_dword_b3 < 10)
			basic_dword_b3_m10		<= 0;
		else
			basic_dword_b3_m10		<= basic_dword_b3 - 7'd10;

		if(basic_dword_b0 < 10)
			basic_dword_b0_m10		<= 0;
		else
			basic_dword_b0_m10		<= basic_dword_b0 - 7'd10;

    end

	always @(posedge clk) begin

		basic_dword_valid			<= 0;

		//Shove bytes into 32-bit chunks
		if(parsing_basic_params && read_done) begin

			basic_dword	<= {read_data, basic_dword[31:8]};
			if(sfdp_addr[1:0] == 3) begin
				basic_dword_addr	<= sfdp_addr[8:2];
				basic_dword_valid	<= 1;
			end

		end

		//Parse the data in 32-bit chunks
		if(basic_dword_valid) begin

			case(basic_dword_addr)

				0: begin

					insn_114_read	<= 8'haa;
					insn_444_read	<= basic_dword[7:0];	//FIXME

					//bits 7:0: mostly legacy, ignore
					//TODO: parse this in case of an old SFDP version?
					//bits 15:8: 4KB erase instruction, ignore
					//bits 23:16
					has_114_read	<= basic_dword[22];
					has_144_read	<= basic_dword[21];
					//1-2-2 fast read, ignore
					has_ddr_mode	<= basic_dword[19];
					case(basic_dword[18:17])
						0: begin
							has_3byte_addr	<= 1;
							has_4byte_addr	<= 0;
						end

						1: begin
							has_3byte_addr	<= 1;
							has_4byte_addr	<= 1;
						end

						2: begin
							has_3byte_addr	<= 0;
							has_4byte_addr	<= 1;
						end
					endcase

					//16: 1-1-2 fast read, ignore

					//bits 31:24: reserved, ignore
				end

				//dword 1: memory capacity, in bits or log bits
				1: begin

					//Bit 31 = log mode
					//Capacity is 2^n *bits*
					if(basic_dword[31])
						capacity_mbits	<= 1'b1 << (basic_dword[15:0] - 20);

					//Not log mode
					//Capacity is num_bits - 1
					else
						capacity_mbits	<= basic_dword[30:20] + 1;

				end

				//Dword 2: Quad mode config for x1 instruction
				2: begin

					insn_114_read		<= basic_dword[31:24];
					modeclk_114_read	<= basic_dword[23:21];
					dummyclk_114_read	<= basic_dword[20:16];

					insn_144_read		<= basic_dword[15:8];
					modeclk_144_read	<= basic_dword[7:5];
					dummyclk_144_read	<= basic_dword[4:0];

				end

				//Dword 3: Dual mode config for x1 instruction (ignore)

				//Dword 4: almost all reserved, but a few bits for quad mode with x4 instruction
				4: begin
					has_444_read		<= basic_dword[4];
				end

				//Dword 5: Dual mode config for x2 instruction (ignore)

				//Dword 6: Quad mode config for x4 instruction
				6: begin
					insn_444_read		<= basic_dword[31:24];
					modeclk_444_read	<= basic_dword[23:21];
					dummyclk_444_read	<= basic_dword[20:16];
				end

				//Dword 7: Erase configuration
				//Erase sizes are 2^n *bytes*.
				//2^(n-10) = kbytes
				//2^(n-7) = kbits
				7: begin
					erase_type2_insn	<= basic_dword[31:24];
					if(basic_dword_b3_m10[3:0])
						erase_type2_kbits	<= 1'b1 << basic_dword_b3_m10[3:0];
					erase_type1_insn	<= basic_dword[15:8];
					if(basic_dword_b0_m10[3:0])
						erase_type1_kbits	<= 1'b1 << basic_dword_b0_m10[3:0];
				end

				//Dword 8: Erase configuration
				8: begin
					erase_type4_insn	<= basic_dword[31:24];
					if(basic_dword_b3_m10[3:0])
						erase_type4_kbits	<= 1'b1 << basic_dword_b3_m10[3:0];
					erase_type3_insn	<= basic_dword[15:8];
					if(basic_dword_b0_m10[3:0])
						erase_type3_kbits	<= 1'b1 << basic_dword_b0_m10[3:0];
				end

				//Dword 9: Erase run times
				9: begin

					case(basic_dword[31:30])
						0:	erase_type4_ms	<= basic_dword[29:25];
						1:	erase_type4_ms	<= {basic_dword[29:25], 4'h0};
						2:	erase_type4_ms	<= {basic_dword[29:25], 7'h0};
						3:	erase_type4_ms	<= {basic_dword[29:25], 10'h0};
					endcase

					case(basic_dword[24:23])
						0:	erase_type3_ms	<= basic_dword[22:18];
						1:	erase_type3_ms	<= {basic_dword[22:18], 4'h0};
						2:	erase_type3_ms	<= {basic_dword[22:18], 7'h0};
						3:	erase_type3_ms	<= {basic_dword[22:18], 10'h0};
					endcase

					case(basic_dword[17:16])
						0:	erase_type2_ms	<= basic_dword[15:11];
						1:	erase_type2_ms	<= {basic_dword[15:11], 4'h0};
						2:	erase_type2_ms	<= {basic_dword[15:11], 7'h0};
						3:	erase_type2_ms	<= {basic_dword[15:11], 10'h0};
					endcase

					case(basic_dword[10:9])
						0:	erase_type1_ms	<= basic_dword[8:4];
						1:	erase_type1_ms	<= {basic_dword[8:4], 4'h0};
						2:	erase_type1_ms	<= {basic_dword[8:4], 7'h0};
						3:	erase_type1_ms	<= {basic_dword[8:4], 10'h0};
					endcase

					erase_typ_to_max		<= { basic_dword[3:0], 1'b1 };

				end

				//Dword 10: Erase/program run times, page info
				10: begin

					//Full chip erase run time
					case(basic_dword[30:29])
						0:	erase_chip_ms	<= {basic_dword[28:24], 4'h0};
						1:	erase_chip_ms	<= {basic_dword[28:24], 8'h0};
						2:	erase_chip_ms	<= {basic_dword[28:24], 12'h0};
						3:	erase_chip_ms	<= {basic_dword[28:24], 16'h0};
					endcase

					//Time to program one byte after the first byte
					if(basic_dword[23])
						program_per_byte_us	<= {basic_dword[22:19], 3'h0};
					else
						program_per_byte_us	<= basic_dword[22:19];

					//Time to program the first byte in a block
					if(basic_dword[18])
						program_first_byte_us	<= {basic_dword[17:14], 3'h0};
					else
						program_first_byte_us	<= basic_dword[17:14];

					//Time to program a whole page
					if(basic_dword[13])
						program_page_us			<= {basic_dword[12:8], 6'h0};
					else
						program_page_us			<= {basic_dword[12:8], 3'h0};

					page_size_bytes				<= 1'b1 << basic_dword[7:4];

					program_typ_to_max			<= { basic_dword[3:0], 1'b1 };
				end

				//Dword 11: Suspend stuff
				11: begin
					//For now we don't support suspend of anything. Ignore it.
				end

				//Dword 12: Suspend stuff
				12: begin
					//For now we don't support suspend of anything. Ignore it.
				end

				//Dword 13: Power-down etc
				13: begin
					//Power down config stuff

					busy_poll_via_flagstatreg	<= basic_dword[3];
					busy_poll_via_statreg		<= basic_dword[2];
				end

				//Dword 14: More config stuff
				14: begin
					//Hold/reset disable TODO

					quad_enable_type			<= basic_dword[22:20];

					//0-4-4 XIP mode: not used, ignore

					//4-4-4 mode enable
					quad_qe_38					<= basic_dword[4];
					quad_38						<= basic_dword[5];
					quad_35						<= basic_dword[6];
					quad_6571					<= basic_dword[7];
					quad_6561					<= basic_dword[8];
					qexit_ff					<= basic_dword[0];
					qexit_f5					<= basic_dword[1];
					qexit_6571					<= basic_dword[2];
					qexit_reset					<= basic_dword[3];
				end

				//Dword 15: 4-byte-address mode config
				15: begin

					//Exit 4-byte mode not supported.
					//If the device has a 4-byte mode we always use it.

				end

			endcase

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The LA

	//intclk is 66.5 MHz for current prototype at last measurement
	RedTinUartWrapper #(
		.WIDTH(384),
		.DEPTH(2048),
		//.UART_CLKDIV(16'd868),	//115200 @ 100 MHz
		.UART_CLKDIV(16'd577),		//115200 @ 66.5 MHz
		.SYMBOL_ROM(
			{
				16384'h0,
				"DEBUGROM", 				8'h0, 8'h01, 8'h00,
				32'd15037,			//period of internal clock, in ps
				32'd2048,			//Capture depth (TODO auto-patch this?)
				32'd384,			//Capture width (TODO auto-patch this?)
				{ "parsing_master_header",	8'h0, 8'h1,  8'h0 },
				{ "parsing_basic_params",	8'h0, 8'h1,  8'h0 },
				{ "phdr_num",				8'h0, 8'h6,  8'h0 },
				{ "has_3byte_addr",			8'h0, 8'h1,  8'h0 },
				{ "has_4byte_addr",			8'h0, 8'h1,  8'h0 },
				{ "erase_type1_insn",		8'h0, 8'h8,  8'h0 },
				{ "erase_type1_mbits",		8'h0, 8'h10, 8'h0 },
				{ "erase_type1_ms",			8'h0, 8'h10, 8'h0 },
				{ "erase_type2_insn",		8'h0, 8'h8,  8'h0 },
				{ "erase_type2_kbits",		8'h0, 8'h10, 8'h0 },
				{ "erase_type2_ms",			8'h0, 8'h10, 8'h0 },
				{ "erase_type3_insn",		8'h0, 8'h8,  8'h0 },
				{ "erase_type3_kbits",		8'h0, 8'h10, 8'h0 },
				{ "erase_type3_ms",			8'h0, 8'h10, 8'h0 },
				{ "erase_type4_insn",		8'h0, 8'h8,  8'h0 },
				{ "erase_type4_kbits",		8'h0, 8'h10, 8'h0 },
				{ "erase_type4_ms",			8'h0, 8'h10, 8'h0 },
				{ "erase_typ_to_max",		8'h0, 8'h5,  8'h0 },
				{ "erase_chip_ms",			8'h0, 8'h20, 8'h0 },
				{ "program_first_byte_us",	8'h0, 8'h7,  8'h0 },
				{ "program_per_byte_us",	8'h0, 8'h7,  8'h0 },
				{ "program_page_us",		8'h0, 8'h10, 8'h0 },
				{ "page_size_bytes",		8'h0, 8'h10, 8'h0 },
				{ "program_typ_to_max",		8'h0, 8'h5, 8'h0 },
				{ "busy_poll_via_flagstatreg", 8'h0, 8'h1,  8'h0 },
				{ "busy_poll_via_statreg",  8'h0, 8'h1,  8'h0 },
				{ "quad_enable_type",		8'h0, 8'h3,  8'h0 },
				{ "quad_qe_38",				8'h0, 8'h1,  8'h0 },
				{ "quad_38",				8'h0, 8'h1,  8'h0 },
				{ "quad_35",				8'h0, 8'h1,  8'h0 },
				{ "quad_6561",				8'h0, 8'h1,  8'h0 },
				{ "quad_6571",				8'h0, 8'h1,  8'h0 },
				{ "qexit_ff",				8'h0, 8'h1,  8'h0 },
				{ "qexit_f5",				8'h0, 8'h1,  8'h0 },
				{ "qexit_6571",				8'h0, 8'h1,  8'h0 },
				{ "qexit_reset",			8'h0, 8'h1,  8'h0 }
			}
		)
	) analyzer (
		.clk(clk),
		.capture_clk(clk),
		.din({
				parsing_master_header,	//1
				parsing_basic_params,	//1
				phdr_num,				//6
				has_3byte_addr,			//1
				has_4byte_addr,			//1
				erase_type1_insn,		//8
				erase_type1_kbits,		//16
				erase_type1_ms,			//16
				erase_type2_insn,		//8
				erase_type2_kbits,		//16
				erase_type2_ms,			//16
				erase_type3_insn,		//8
				erase_type3_kbits,		//16
				erase_type3_ms,			//16
				erase_type4_insn,		//8
				erase_type4_kbits,		//16
				erase_type4_ms,			//16
				erase_typ_to_max,		//5
				erase_chip_ms,			//32
				program_first_byte_us,	//7
				program_per_byte_us,	//7
				program_page_us,		//16
				page_size_bytes,		//16
				program_typ_to_max,		//5
				busy_poll_via_flagstatreg,	//1
				busy_poll_via_statreg,		//1
				quad_enable_type,		//3
				quad_qe_38,				//1
				quad_38,				//1
				quad_35,				//1
				quad_6561,				//1
				quad_6571,				//1
				qexit_ff,				//1
				qexit_f5,				//1
				qexit_6571,				//1
				qexit_reset,			//1

				112'h0					//padding
			}),
		.uart_rx(uart_rxd),
		.uart_tx(uart_txd),
		.la_ready(la_ready)
	);

endmodule
