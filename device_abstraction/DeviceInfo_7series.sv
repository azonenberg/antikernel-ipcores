`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2016 Andrew D. Zonenberg                                                                          *
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
	@brief Query information about the device
 */
module DeviceInfo_7series(

	//max 100 MHz except in -2LE where max is 75
	input wire			clk,

	//Device DNA (57 bits padded with zeros to 64)
	output logic[63:0]	die_serial = 0,

	//JTAG IDCODE (not yet implemented)
	output logic[31:0]	idcode = 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Die serial number

	wire	dna_out;
	logic	dna_shift = 0;
	logic	dna_read = 0;
	DNA_PORT dna_port(
		.DOUT(dna_out),
		.DIN(dna_out),
		.READ(dna_read),
		.SHIFT(dna_shift),
		.CLK(clk));

	enum logic[1:0]
	{
		DNA_READ_STATE_BOOT_0	= 2'h0,
		DNA_READ_STATE_BOOT_1	= 2'h1,
		DNA_READ_STATE_READ		= 2'h2,
		DNA_READ_STATE_DONE		= 2'h3
	} dna_read_state = DNA_READ_STATE_BOOT_0;

	logic[6:0] dna_read_count	= 0;
	logic boot_done				= 0;

	always_ff @(posedge clk) begin

		dna_shift	<= 0;
		dna_read	<= 0;

		case(dna_read_state)

			//Wait 128 clocks during boot to be REALLY sure everything is fully reset
			DNA_READ_STATE_BOOT_0: begin

				dna_read_count		<= dna_read_count + 7'h1;

				if(dna_read_count == 127) begin
					dna_read		<= 1;
					dna_read_count	<= 0;
					dna_read_state	<= DNA_READ_STATE_BOOT_1;
				end
			end

			//Read is done, start shifting data
			DNA_READ_STATE_BOOT_1: begin
				dna_shift 			<= 1;
				dna_read_state 		<= DNA_READ_STATE_READ;
			end	//end DNA_READ_STATE_BOOT_0

			//Shift the data
			DNA_READ_STATE_READ: begin
				dna_shift <= 1;

				dna_read_count		<= dna_read_count + 7'h1;
				die_serial			<= {die_serial[62:0], dna_out};

				//Done?
				if(dna_read_count == 55) begin
					dna_read_state	<= DNA_READ_STATE_DONE;
					boot_done		<= 1;
					die_serial[56]	<= 1'b1;
				end
			end	//end DNA_READ_STATE_READ

			DNA_READ_STATE_DONE: begin
				//do nothing
			end	//end DNA_READ_STATE_DONE

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IDCODE (read from the ICAP)

	logic[31:0]	icap_din	= 0;
	logic		icap_cs_n	= 1;
	logic		icap_wr_n	= 1;
	wire[31:0]	icap_dout;

	wire		la_trig_out;

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

	(* LOC = "ICAP_X0Y1" *)
	ICAPE2 #(
		.ICAP_WIDTH("X32")
	) icap(
		.CLK(clk),
		.I(icap_din_bswap),
		.CSIB(icap_cs_n),
		.O(icap_dout),
		.RDWRB(icap_wr_n)
		);

	enum logic[4:0]
	{
		ICAP_STATE_BOOT_HOLD	= 5'h00,
		ICAP_STATE_INIT_0		= 5'h01,
		ICAP_STATE_INIT_1		= 5'h02,
		ICAP_STATE_INIT_2		= 5'h03,
		ICAP_STATE_INIT_3		= 5'h04,
		ICAP_STATE_INIT_4		= 5'h05,
		ICAP_STATE_SYNC			= 5'h06,
		ICAP_STATE_NOP			= 5'h07,
		ICAP_STATE_HEADER		= 5'h08,
		ICAP_STATE_PIPE			= 5'h09,
		ICAP_STATE_READ			= 5'h0a,
		ICAP_STATE_DONE			= 5'h0b
	} icap_state = ICAP_STATE_BOOT_HOLD;

	//see https://forums.xilinx.com/t5/Configuration/ICAPE2-documentation/td-p/453996
	logic[3:0] count = 0;
	always_ff @(posedge clk) begin

		case(icap_state)

			ICAP_STATE_BOOT_HOLD: begin
				if(la_trig_out)
				//if(boot_done)
					icap_state	<= ICAP_STATE_INIT_0;
			end	//end ICAP_STATE_BOOT_HOLD

			//Idle state
			ICAP_STATE_INIT_0: begin
				icap_din	<= 32'hffffffff;
				icap_state	<= ICAP_STATE_INIT_1;
			end	//end ICAP_STATE_INIT_0

			//go to write mode while deselected
			ICAP_STATE_INIT_1: begin
				icap_wr_n	<= 0;
				icap_state	<= ICAP_STATE_INIT_2;
			end	//end ICAP_STATE_INIT_1

			//select and write dummy word
			ICAP_STATE_INIT_2: begin
				icap_cs_n	<= 0;
				icap_state	<= ICAP_STATE_INIT_3;
			end	//end ICAP_STATE_INIT_2

			//bus width 1
			ICAP_STATE_INIT_3: begin
				icap_cs_n	<= 0;
				icap_din	<= 32'h000000bb;
				icap_state	<= ICAP_STATE_INIT_4;
			end	//end ICAP_STATE_INIT_3

			ICAP_STATE_INIT_4: begin
				icap_cs_n	<= 0;
				icap_din	<= 32'h11220044;
				icap_state	<= ICAP_STATE_SYNC;
			end	//end ICAP_STATE_INIT_4

			//Finally we can write the sync word!
			ICAP_STATE_SYNC: begin
				icap_din	<= 32'haa995566;
				icap_state	<= ICAP_STATE_NOP;
			end	//end ICAP_STATE_SYNC

			//Need a nop before doing anything else
			ICAP_STATE_NOP: begin
				icap_din	<= 32'h20000000;
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
				if(count == 8)
					icap_state	<= ICAP_STATE_DONE;
				if(count == 3)
					idcode	<= icap_dout_bswap;
			end	//end ICAP_STATE_READ

			//TODO: desync at the end??
			ICAP_STATE_DONE: begin
				icap_cs_n	<= 1;
			end	//end ICAP_STATE_DONE

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	ila scope(
		.clk(clk),
		.probe0(icap_din_bswap),
		.probe1(icap_cs_n),
		.probe2(icap_wr_n),
		.probe3(icap_dout_bswap),
		.probe4(icap_state),
		.probe5(count),
		.probe6(32'h0),
		.probe7(32'h0),
		.trig_out(la_trig_out),
		.trig_out_ack(la_trig_out)
	);

endmodule
