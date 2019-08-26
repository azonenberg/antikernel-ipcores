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
	input wire		scan_start,
	output logic	scan_done	= 0,
	output logic	sfdp_bad	= 0,

	//SPI interface
	output logic		shift_en	= 0,
	input wire			shift_done,
	output logic[7:0]	spi_tx_data	= 0,
	input wire[7:0]		spi_rx_data,
	output logic		spi_cs_n	= 1,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Memory metadata
	output logic[15:0]	capacity_mbits		= 0,

	//Address size info
    output logic		has_3byte_addr		= 0,
    output logic		has_4byte_addr		= 0,

    output logic		enter_4b_b7			= 0,	//Issue 0xb7
    output logic		enter_4b_we_b7		= 0,	//Issue WE (0x06) then 0xb7
    output logic		enter_4b_nvcr		= 0,	//Enter 4-byte mode by writing to NVCR with B5/B1
    output logic		enter_4b_dedicated	= 0,	//no switch, use separate instructions

    //Erase configuration
    output logic[7:0]	erase_type2_insn	= 0,
    output logic[15:0]	erase_type2_kbits	= 0,

    input wire			la_ready
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Configuration

	localparam XILINX_7SERIES_CCLK_WORKAROUND	= 1;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Flash read-request logic

    //List of JEDEC standard SPI flash instructions
	localparam INST_READ_SFDP		= 8'h5a;

	//Indicates we're busy and not able to accept commands
    logic		read_busy				= 1;

	//Request a read from the given address
    logic		read_request			= 0;
    logic[23:0]	read_addr				= 0;

	//Request termination of the read
	//(it's done when read_busy goes low)
    logic		read_finish_request		= 0;
    logic		read_finish_pending		= 0;

	//Read state machine
	enum logic[2:0]
	{
		READ_STATE_INIT_0		= 0,
		READ_STATE_INIT_1		= 1,
		READ_STATE_IDLE			= 2,
		READ_STATE_COMMAND		= 3,
		READ_STATE_ADDR			= 4,
		READ_STATE_DUMMY		= 5,
		READ_STATE_DATA			= 6
    } read_state				= READ_STATE_INIT_0;

	logic[8:0]	count					= 0;
	logic		read_done				= 0;
	logic[7:0]	read_data				= 0;

    always_ff @(posedge clk) begin
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

	enum logic[7:0]
	{
		STATE_BOOT_WAIT			= 0,
		STATE_HEADER_PARSE		= 1,
		STATE_HEADER_FINISH_0	= 2,
		STATE_HEADER_FINISH_1	= 3,
		STATE_READ_BASIC		= 4,
		STATE_PARSE_BASIC		= 5,
		STATE_READ_SECTOR_MAP	= 6,
		STATE_PARSE_SECTOR_MAP	= 7,
		STATE_HANG				= 255
	} state		= STATE_BOOT_WAIT;

	logic		phdrs_done	= 0;

	//Most recent revision number and offset for the JEDEC Basic SPI Flash Parameters table
	logic[15:0]	basic_params_rev		= 0;
	logic[23:0]	basic_params_offset		= 0;
	logic[7:0]	basic_params_len		= 0;

	//Sector map (TODO: handle devices without one)
	logic		has_sector_map			= 0;
	logic[15:0]	sector_map_rev			= 0;
	logic[23:0]	sector_map_offset		= 0;
	logic[7:0]	sector_map_len			= 0;

	logic[8:0]	sfdp_addr				= 0;
	wire[5:0]	phdr_num				= sfdp_addr[8:3] - 6'd1;
	wire[2:0]	phdr_boff				= sfdp_addr[2:0];

	//A few helpers from the top-level state machine
	wire		parsing_master_header	= (state == STATE_HEADER_PARSE) || (state == STATE_HEADER_FINISH_0);
	wire		parsing_basic_params	= (state == STATE_PARSE_BASIC);
	wire		parsing_sector_map		= (state == STATE_PARSE_SECTOR_MAP);

	//Reasons why we failed to parse the descriptor table
	logic[7:0]	sfdp_fail_reason	= 0;

	localparam SFDP_REASON_BAD_HEADER			= 8'h1;
	localparam SFDP_REASON_BAD_MAJOR_VERSION	= 8'h2;

    always_ff @(posedge clk) begin
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
					state				<= STATE_HEADER_FINISH_0;
				end

			end	//end STATE_HEADER_PARSE

			//Once we ask for the read to stop, wait for it to actually do so
			STATE_HEADER_FINISH_0: begin
				if(!read_busy)
					state				<= STATE_HEADER_FINISH_1;
			end	//end STATE_HEADER_FINISH_0

			//Wait one more clock to parse the last parameter header
			STATE_HEADER_FINISH_1: begin
				state		<= STATE_READ_BASIC;
			end	//end STATE_HEADER_FINISH_0

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

						//Read the sector map, if we have one
						if(has_sector_map)
							state				<= STATE_READ_SECTOR_MAP;
						else
							state				<= STATE_HANG;

					end

				end
			end	//end STATE_PARSE_BASIC

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read sector map

			STATE_READ_SECTOR_MAP: begin
				if(!read_busy) begin
					read_request		<= 1;
					read_addr			<= sector_map_offset;
					state				<= STATE_PARSE_SECTOR_MAP;

					sfdp_addr			<= 9'h0;
				end
			end	//end STATE_READ_SECTOR_MAP

			STATE_PARSE_SECTOR_MAP: begin

				if(read_done) begin

					sfdp_addr			<= sfdp_addr + 1'h1;

					//If we just read the last byte of the descriptor, we're done.
					//Note that basic_params_len is a count of DWORDs so we have to multiply by 4!
					if( (sector_map_len == sfdp_addr[8:2]) && (sfdp_addr[1:0] == 2'h3) ) begin
						read_finish_request	<= 1;

						//Done
						state				<= STATE_HANG;

					end

				end

			end	//end STATE_PARSE_SECTOR_MAP

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// HANG - done

			STATE_HANG: begin
				if(!read_busy)
					scan_done					<= 1;
			end	//end STATE_HANG

		endcase
    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Parse the parameter headers

	//SFDP header data. Probably not useful outside this state machine
	logic[7:0]	sfdp_minor_rev			= 0;
	logic[7:0]	sfdp_num_phdrs			= 0;

	//List of parameter IDs
	localparam	PARAM_ID_JEDEC_BASIC		= 16'hff00;	//JEDEC basic SPI flash parameters
	localparam	PARAM_ID_JEDEC_SECTOR_MAP	= 16'hff81;	//JEDEC sector map
	localparam	PARAM_ID_JEDEC_FOUR_BYTE	= 16'hff84;	//TODO

	//The SFDP parameter table currently being parsed
	logic		sfdp_param_valid		= 0;
	logic[15:0]	sfdp_current_param		= 0;
	logic[15:0]	sfdp_param_rev			= 0;
	logic[7:0]	sfdp_param_len			= 0;
	logic[23:0]	sfdp_param_offset		= 0;

	enum logic[1:0]
	{
		SFDP_STATE_READ_HEADER			= 0,
		SFDP_STATE_READ_PHDR			= 1
	} sfdp_state			= SFDP_STATE_READ_HEADER;

    always_ff @(posedge clk) begin

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

				//It's a JEDEC sector map
				PARAM_ID_JEDEC_SECTOR_MAP: begin

					//Major rev must be 1, we only support this.
					//Ignore anything higher.
					if(sfdp_param_rev[15:8] == 8'h1) begin
						has_sector_map			<= 1;

						//If rev is higher than what we had, this is better.
						if(sfdp_param_rev > sector_map_rev) begin
							sector_map_len		<= sfdp_param_len;
							sector_map_offset	<= sfdp_param_offset;
							sector_map_rev		<= sfdp_param_rev;
						end

					end

				end	//end PARAM_ID_JEDEC_SECTOR_MAP

			endcase

		end

		if(parsing_master_header) begin

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

	//Clocking
    logic		has_ddr_mode		= 0;

	//Fast read (1-1-4)
    logic		has_114_read		= 0;
    logic[7:0]	insn_114_read		= 0;
    logic[2:0]	modeclk_114_read	= 0;
    logic[4:0]	dummyclk_114_read	= 0;

	//Fast read (1-4-4)
    logic		has_144_read		= 0;
    logic[7:0]	insn_144_read		= 0;
    logic[2:0]	modeclk_144_read	= 0;
    logic[4:0]	dummyclk_144_read	= 0;

	//Fast read (4-4-4)
    logic		has_444_read		= 0;
    logic[7:0]	insn_444_read		= 0;
    logic[2:0]	modeclk_444_read	= 0;
    logic[4:0]	dummyclk_444_read	= 0;

    //Erasing
    logic[7:0]	erase_type1_insn	= 0;
    logic[15:0]	erase_type1_kbits	= 0;
    logic[15:0]	erase_type1_ms		= 0;	//typical delay

    logic[15:0]	erase_type2_ms		= 0;

    logic[7:0]	erase_type3_insn	= 0;
    logic[15:0]	erase_type3_kbits	= 0;
    logic[15:0]	erase_type3_ms		= 0;

    logic[7:0]	erase_type4_insn	= 0;
    logic[15:0]	erase_type4_kbits	= 0;
    logic[15:0]	erase_type4_ms		= 0;

    logic[31:0]	erase_chip_ms			= 0;

    //Programming
    logic[6:0]	program_first_byte_us	= 0;
    logic[6:0]	program_per_byte_us		= 0;
    logic[15:0]	program_page_us			= 0;
    logic[15:0]	page_size_bytes			= 0;

    //Time multipliers
    logic[4:0]	erase_typ_to_max	= 0;
    logic[4:0]	program_typ_to_max	= 0;

    //Status register polling
    logic		busy_poll_via_flagstatreg	= 0;	//0x70
    logic		busy_poll_via_statreg		= 0;	//0x05

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
    logic[2:0]	quad_enable_type			= 0;

	//Methods of entering 4-4-4 mode
    logic		quad_qe_38	= 0;			//Set QE, then 0x38
    logic		quad_38		= 0;			//Issue 0x38
    logic		quad_35		= 0;			//Issue 0x35
    logic		quad_6561	= 0;			//Issue 0x65 then 61 read-modify-write
    logic		quad_6571	= 0;			//Issue 0x65/address then 71/address read-modify-write

    //Methods of leaving 4-4-4 mode
    logic		qexit_ff	= 0;			//Issue 0xff
    logic		qexit_f5	= 0;			//Issue 0xf5
    logic		qexit_reset	= 0;			//Issue 0x66/99 to reset
    logic		qexit_6571	= 0;			//Issue 0x65/address then 71/address read-modify-write

    //Methods of entering 4-byte mode
    //extended address register not supported
    //bank register not supported
    logic		enter_4b_always = 0;		//always 4-byte mode

    //Methods of resetting the device
    logic		reset_f_8clk	= 0;		//drive 0xf on all lines for 8 clocks
    logic		reset_f_10clk	= 0;		//drive 0xf on all lines for 10 clocks
    logic		reset_f_16clk	= 0;		//drive 0xf on all lines for 16 clocks
    logic		reset_f0		= 0;		//issue 0xf0
    logic		reset_66_99		= 0;		//issue 0x66, 0x99

    //Methods of accessing status register 1
    logic		sr1_nv_06		= 0;		//nonvolatile status register, WE with 0x06
    logic		sr1_v_06		= 0;		//volatile status register, WE with 0x06
    logic		sr1_v_50		= 0;		//volatile status register, WE with 0x50
    logic		sr1_vnv			= 0;		//nonvolatile status register with volatile shadow
											//WE with 0x06 NV, 50 V
	logic		sr1_mixed_06	= 0;		//mixed volatile and nonvolatile bits, WE with 0x06

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Helpers for 32-bit-wide table indexing

	logic[6:0]	table_dword_addr	= 0;
    logic[31:0]	table_dword			= 0;
    logic		basic_dword_valid	= 0;
    logic		sector_dword_valid	= 0;

    always_ff @(posedge clk) begin
		basic_dword_valid			<= 0;
		sector_dword_valid			<= 0;

		//Shove bytes into 32-bit chunks
		if( (parsing_basic_params || parsing_sector_map) && read_done) begin

			table_dword				<= {read_data, table_dword[31:8]};

			if(sfdp_addr[1:0] == 3) begin
				table_dword_addr	<= sfdp_addr[8:2];
				basic_dword_valid	<= parsing_basic_params;
				sector_dword_valid	<= parsing_sector_map;
			end

		end
    end

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Parse the basic flash parameter table

	//Carve out a few bytes and name them
    wire[7:0]	table_dword_b3		= table_dword[23:16];
    wire[7:0]	table_dword_b0		= table_dword[7:0];

	//Subtract 10 with saturation to zero (for erase block info)
    logic[7:0]	table_dword_b3_m10	= 0;
    logic[7:0]	table_dword_b0_m10	= 0;
    always_comb begin
		if(table_dword_b3 < 10)
			table_dword_b3_m10		= 0;
		else
			table_dword_b3_m10		= table_dword_b3 - 7'd10;

		if(table_dword_b0 < 10)
			table_dword_b0_m10		= 0;
		else
			table_dword_b0_m10		= table_dword_b0 - 7'd10;

    end

	always_ff @(posedge clk) begin

		//Parse the data in 32-bit chunks
		if(basic_dword_valid) begin

			case(table_dword_addr)

				0: begin

					insn_114_read	<= 8'haa;
					insn_444_read	<= table_dword[7:0];	//FIXME

					//bits 7:0: mostly legacy, ignore
					//TODO: parse this in case of an old SFDP version?
					//bits 15:8: 4KB erase instruction, ignore
					//bits 23:16
					has_114_read	<= table_dword[22];
					has_144_read	<= table_dword[21];
					//1-2-2 fast read, ignore
					has_ddr_mode	<= table_dword[19];
					case(table_dword[18:17])
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
					if(table_dword[31])
						capacity_mbits	<= 1'b1 << (table_dword[15:0] - 20);

					//Not log mode
					//Capacity is num_bits - 1
					else
						capacity_mbits	<= table_dword[30:20] + 1;

				end

				//Dword 2: Quad mode config for x1 instruction
				2: begin

					insn_114_read		<= table_dword[31:24];
					modeclk_114_read	<= table_dword[23:21];
					dummyclk_114_read	<= table_dword[20:16];

					insn_144_read		<= table_dword[15:8];
					modeclk_144_read	<= table_dword[7:5];
					dummyclk_144_read	<= table_dword[4:0];

				end

				//Dword 3: Dual mode config for x1 instruction (ignore)

				//Dword 4: almost all reserved, but a few bits for quad mode with x4 instruction
				4: begin
					has_444_read		<= table_dword[4];
				end

				//Dword 5: Dual mode config for x2 instruction (ignore)

				//Dword 6: Quad mode config for x4 instruction
				6: begin
					insn_444_read		<= table_dword[31:24];
					modeclk_444_read	<= table_dword[23:21];
					dummyclk_444_read	<= table_dword[20:16];
				end

				//Dword 7: Erase configuration
				//Erase sizes are 2^n *bytes*.
				//2^(n-10) = kbytes
				//2^(n-7) = kbits
				7: begin
					erase_type2_insn	<= table_dword[31:24];
					if(table_dword_b3_m10[3:0])
						erase_type2_kbits	<= 1'b1 << table_dword_b3_m10[3:0];
					erase_type1_insn	<= table_dword[15:8];
					if(table_dword_b0_m10[3:0])
						erase_type1_kbits	<= 1'b1 << table_dword_b0_m10[3:0];
				end

				//Dword 8: Erase configuration
				8: begin
					erase_type4_insn	<= table_dword[31:24];
					if(table_dword_b3_m10[3:0])
						erase_type4_kbits	<= 1'b1 << table_dword_b3_m10[3:0];
					erase_type3_insn	<= table_dword[15:8];
					if(table_dword_b0_m10[3:0])
						erase_type3_kbits	<= 1'b1 << table_dword_b0_m10[3:0];
				end

				//Dword 9: Erase run times
				9: begin

					case(table_dword[31:30])
						0:	erase_type4_ms	<= table_dword[29:25];
						1:	erase_type4_ms	<= {table_dword[29:25], 4'h0};
						2:	erase_type4_ms	<= {table_dword[29:25], 7'h0};
						3:	erase_type4_ms	<= {table_dword[29:25], 10'h0};
					endcase

					case(table_dword[24:23])
						0:	erase_type3_ms	<= table_dword[22:18];
						1:	erase_type3_ms	<= {table_dword[22:18], 4'h0};
						2:	erase_type3_ms	<= {table_dword[22:18], 7'h0};
						3:	erase_type3_ms	<= {table_dword[22:18], 10'h0};
					endcase

					case(table_dword[17:16])
						0:	erase_type2_ms	<= table_dword[15:11];
						1:	erase_type2_ms	<= {table_dword[15:11], 4'h0};
						2:	erase_type2_ms	<= {table_dword[15:11], 7'h0};
						3:	erase_type2_ms	<= {table_dword[15:11], 10'h0};
					endcase

					case(table_dword[10:9])
						0:	erase_type1_ms	<= table_dword[8:4];
						1:	erase_type1_ms	<= {table_dword[8:4], 4'h0};
						2:	erase_type1_ms	<= {table_dword[8:4], 7'h0};
						3:	erase_type1_ms	<= {table_dword[8:4], 10'h0};
					endcase

					erase_typ_to_max		<= { table_dword[3:0], 1'b1 };

				end

				//Dword 10: Erase/program run times, page info
				10: begin

					//Full chip erase run time
					case(table_dword[30:29])
						0:	erase_chip_ms	<= {table_dword[28:24], 4'h0};
						1:	erase_chip_ms	<= {table_dword[28:24], 8'h0};
						2:	erase_chip_ms	<= {table_dword[28:24], 12'h0};
						3:	erase_chip_ms	<= {table_dword[28:24], 16'h0};
					endcase

					//Time to program one byte after the first byte
					if(table_dword[23])
						program_per_byte_us	<= {table_dword[22:19], 3'h0};
					else
						program_per_byte_us	<= table_dword[22:19];

					//Time to program the first byte in a block
					if(table_dword[18])
						program_first_byte_us	<= {table_dword[17:14], 3'h0};
					else
						program_first_byte_us	<= table_dword[17:14];

					//Time to program a whole page
					if(table_dword[13])
						program_page_us			<= {table_dword[12:8], 6'h0};
					else
						program_page_us			<= {table_dword[12:8], 3'h0};

					page_size_bytes				<= 1'b1 << table_dword[7:4];

					program_typ_to_max			<= { table_dword[3:0], 1'b1 };
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

					busy_poll_via_flagstatreg	<= table_dword[3];
					busy_poll_via_statreg		<= table_dword[2];
				end

				//Dword 14: More config stuff
				14: begin
					//Hold/reset disable TODO

					quad_enable_type			<= table_dword[22:20];

					//0-4-4 XIP mode: not used, ignore

					//4-4-4 mode enable
					quad_qe_38					<= table_dword[4];
					quad_38						<= table_dword[5];
					quad_35						<= table_dword[6];
					quad_6571					<= table_dword[7];
					quad_6561					<= table_dword[8];
					qexit_ff					<= table_dword[0];
					qexit_f5					<= table_dword[1];
					qexit_6571					<= table_dword[2];
					qexit_reset					<= table_dword[3];
				end

				//Dword 15: 4-byte-address mode config
				15: begin

					//Exit 4-byte mode not supported.
					//If the device has a 4-byte mode we always use it for the duration of our run.

					enter_4b_b7					<= table_dword[24];
					enter_4b_we_b7				<= table_dword[25];
					enter_4b_nvcr				<= table_dword[28];
					enter_4b_dedicated			<= table_dword[29];
					enter_4b_always				<= table_dword[30];

					reset_f_8clk				<= table_dword[8];
					reset_f_10clk				<= table_dword[9];
					reset_f_16clk				<= table_dword[10];
					reset_f0					<= table_dword[11];
					reset_66_99					<= table_dword[12];

					//Methods of accessing SR1, and volatile/nonvolatile mode
					//(page 35 bottom)
					sr1_nv_06					<= table_dword[0];
					sr1_v_06					<= table_dword[1];
					sr1_v_50					<= table_dword[2];
					sr1_vnv						<= table_dword[3];
					sr1_mixed_06				<= table_dword[4];

				end

			endcase

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Parse the sector map

    always_ff @(posedge clk) begin

		//Parse the data in 32-bit chunks
		if(sector_dword_valid) begin

			case(table_dword_addr)

				0: begin
					//TODO
				end

			endcase

		end
	end

endmodule
