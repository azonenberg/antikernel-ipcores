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
	@brief A timer, readable/writable in blocks over APB

	For now, only full width reads are supported (so no support for 32-bit counters on 16-bit APB)
 */
module APB_DeviceInfo_UltraScale(

	//The APB bus
	APB.completer 	apb,

	//DNA_PORT clock (must be max 200 MHz at 0.85/0.90V or 175 MHz at 0.72V)
	input wire		clk_dna,

	//ICAP clock (must be max 200 MHz at 0.85/0.90V or 150 MHz at 0.72V)
	input wire		clk_icap
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support up to 32-bit APB due to register alignment, throw synthesis error for anything else

	if(apb.DATA_WIDTH > 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[3:0]
	{
		REG_STATUS		= 4'h0,
		REG_IDCODE		= 4'h4		//Device IDCODE

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DNA_PORT read logic TODO

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Timer for ICAP availability (TODO: is this needed on parts newer than 7 series?)

	localparam BOOT_INIT_VAL =  1;

	logic[22:0] boot_count	= BOOT_INIT_VAL;
	logic		boot_done	= 0;

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

	ICAPE3 icap(
		.CLK(clk_icap),
		.I(icap_din_bswap),
		.CSIB(icap_cs_n),
		.O(icap_dout),
		.RDWRB(icap_wr_n),
		.AVAIL(),
		.PRDONE(),
		.PRERROR()
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
					REG_STATUS:		apb.prdata <= { 31'h0, idcode_valid };
					REG_IDCODE:		apb.prdata	= idcode;
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
