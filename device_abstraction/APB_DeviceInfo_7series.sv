`timescale 1ns / 1ps
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

/**
	@brief Device information block for 7 series FPGAs

	TODO: add CDC support for when clk_dna and clk_icap are not the same as PCLK
	(or do we just rely on the user to add constraints since they're pseudo-constant?)
 */
module APB_DeviceInfo_7series(

	//The APB bus
	APB.completer 	apb,

	//DNA_PORT clock
	input wire		clk_dna,

	//ICAP clock
	input wire		clk_icap
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[7:0]
	{
		REG_STATUS		= 8'h00,	//[0] = IDCODE valid
									//[1] = serial valid
		REG_IDCODE		= 8'h04,	//Device IDCODE
		REG_SERIAL_0	= 8'h0c,	//Die serial bits 63:32
		REG_SERIAL_1	= 8'h10,	//Die serial bits 31:0
		REG_USERCODE	= 8'h14		//User ID
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DNA_PORT read logic

	logic		boot_done	= 0;

	wire	dna_out;
	logic	dna_shift = 0;
	logic	dna_read = 0;
	DNA_PORT #(
		.SIM_DNA_VALUE(57'h0adbeef_c0def00d)
	) dna_port (
		.DOUT(dna_out),
		.DIN(dna_out),
		.READ(dna_read),
		.SHIFT(dna_shift),
		.CLK(clk_dna));

	logic[63:0]	die_serial 			= 0;
	logic		die_serial_valid	= 0;

	enum logic[1:0]
	{
		DNA_READ_STATE_BOOT		= 2'h0,
		DNA_READ_STATE_READ		= 2'h1,
		DNA_READ_STATE_DONE		= 2'h2
	} dna_read_state = DNA_READ_STATE_BOOT;

	logic[6:0] dna_read_count	= 0;

	always_ff @(posedge clk_dna) begin

		dna_shift	<= 0;
		dna_read	<= 0;

		case(dna_read_state)

			//Wait for boot-time init
			DNA_READ_STATE_BOOT: begin

				if(boot_done) begin
					dna_read 			<= 1;
					dna_read_state 		<= DNA_READ_STATE_READ;
				end
			end

			//Shift the data
			DNA_READ_STATE_READ: begin
				dna_shift <= 1;

				dna_read_count		<= dna_read_count + 7'h1;
				die_serial			<= {8'h0, die_serial[55:0], dna_out};	//now shift LSB first

				//Done?
				if(dna_read_count == 57) begin
					dna_read_state		<= DNA_READ_STATE_DONE;
					die_serial_valid	<= 1;
				end
			end	//end DNA_READ_STATE_READ

			DNA_READ_STATE_DONE: begin
				//do nothing
			end	//end DNA_READ_STATE_DONE

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Timer for ICAP availability (TODO: we probably don't need such a large counter)

	localparam BOOT_INIT_VAL =  1;

	logic[22:0] boot_count	= BOOT_INIT_VAL;

	always_ff @(posedge clk_icap) begin
		if(!boot_done) begin
			if(boot_count == 0)
				boot_done	<= 1;
			else
				boot_count <= boot_count + 1'h1;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IDCODE ICAP read logic

	logic		idcode_valid	= 0;
	logic[31:0]	idcode;

	logic[31:0]	icap_din	= 32'hffffffff;
	logic		icap_cs_n	= 1;
	logic		icap_wr_n	= 1;
	wire[31:0]	icap_dout;

	wire[31:0]	icap_din_bswap;
	wire[31:0]	icap_dout_bswap;
	generate
		for(genvar b=0; b<4; b++) begin
			for(genvar g=0; g<8; g++) begin
				assign icap_din_bswap[b*8 + g] = icap_din[b*8 + 7-g];
				assign icap_dout_bswap[b*8 + g] = icap_dout[b*8 + 7-g];
			end
		end
	endgenerate

	ICAPE2 icap(
		.CLK(clk_icap),
		.I(icap_din_bswap),
		.CSIB(icap_cs_n),
		.O(icap_dout),
		.RDWRB(icap_wr_n)
		);

	enum logic[2:0]
	{
		ICAP_STATE_BOOT_HOLD	= 3'h0,
		ICAP_STATE_SYNC			= 3'h1,
		ICAP_STATE_NOP			= 3'h2,
		ICAP_STATE_HEADER		= 3'h3,
		ICAP_STATE_PIPE			= 3'h4,
		ICAP_STATE_READ			= 3'h5,
		ICAP_STATE_DONE			= 3'h6
	} icap_state = ICAP_STATE_BOOT_HOLD;

	//see https://forums.xilinx.com/t5/Configuration/ICAPE2-documentation/td-p/453996
	logic[15:0] count = 0;
	always_ff @(posedge clk_icap) begin

		case(icap_state)

			//Wait a looong time for the ICAP to initialize properly
			ICAP_STATE_BOOT_HOLD: begin
				if(boot_done) begin
					icap_cs_n	<= 0;
					icap_wr_n	<= 0;
					icap_state	<= ICAP_STATE_SYNC;
				end
			end	//end ICAP_STATE_BOOT_HOLD

			//Write sync word
			ICAP_STATE_SYNC: begin
				icap_din	<= 32'haa995566;
				icap_state	<= ICAP_STATE_NOP;
			end	//end ICAP_STATE_SYNC

			//There is a two-cycle pipeline hazard period after the sync word in which we must write nops
			//and all input is ignored.
			ICAP_STATE_NOP: begin
				count		<= count + 1'h1;
				icap_din	<= 32'h20000000;

				if(count == 1)
					icap_state	<= ICAP_STATE_HEADER;
			end	//end ICAP_STATE_NOP

			//Request the read data
			ICAP_STATE_HEADER: begin
				icap_din	<= 32'h28018001;
				icap_state	<= ICAP_STATE_PIPE;
			end	//end ICAP_STATE_HEADER

			//Spam nops to flush the pipeline
			ICAP_STATE_PIPE: begin
				icap_din	<= 32'h20000000;
				count		<= count + 1;

				if(count == 2)
					icap_cs_n	<= 1;
				if(count == 3)
					icap_wr_n	<= 1;
				if(count == 4) begin
					icap_cs_n	<= 0;
					count		<= 0;
					icap_state	<= ICAP_STATE_READ;
				end
			end	//end ICAP_STATE_PIPE

			//Read pipeline delay
			ICAP_STATE_READ: begin
				count <= count + 1;
				if(count == 8) begin
					icap_state		<= ICAP_STATE_DONE;
					idcode_valid	<= 1;
					count			<= 0;
				end
				if(count == 3)
					idcode	<= icap_dout_bswap;
			end	//end ICAP_STATE_READ

			//Desync at the end
			ICAP_STATE_DONE: begin

				if(count < 9)
					count		<= count + 1;

				//clear cs#
				if(count == 0)
					icap_cs_n	<= 1;

				//go to write mode
				if(count == 1)
					icap_wr_n	<= 0;
				if(count == 2) begin
					icap_din	<= 32'h20000000;
					icap_cs_n	<= 0;
				end

				//write CMD, 1 word, DESYNC
				if(count == 4)
					icap_din	<= 32'h30008001;
				if(count == 5)
					icap_din	<= 32'h0000000d;

				//more nops
				if(count == 6)
					icap_din	<= 32'h20000000;

				if(count == 8)
					icap_cs_n	<= 1;

			end	//end ICAP_STATE_DONE

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TODO: synchronizers to APB clock domain

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// USERCODE register

	wire[31:0] usercode;

	USR_ACCESSE2 user(
		.DATA(usercode),
		.CFGCLK(),
		.DATAVALID()
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin
				case(apb.paddr)
					REG_STATUS:		apb.prdata = { 30'h0, die_serial_valid, idcode_valid };
					REG_IDCODE:		apb.prdata = idcode;
					REG_SERIAL_0:	apb.prdata = die_serial[63:32];
					REG_SERIAL_1:	apb.prdata = die_serial[31:0];
					REG_USERCODE:	apb.prdata = usercode;
					default:		apb.pslverr	= 1;
				endcase
			end

			//write
			else begin
				//nothing writable for now
				apb.pslverr	 = 1;
			end

		end
	end

endmodule
