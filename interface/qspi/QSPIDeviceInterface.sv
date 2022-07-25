`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2022 Andrew D. Zonenberg                                                                          *
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
	@brief QSPI device-mode transceiver based on oversampling (does not require clock capable input for SCK)

	Only supports x4 mode for now, no x1 support.

	clk frequency must be at least 4x SCK frequency.

	A single cycle bus turnaround / dummy stage is present after the instruction/address and before data, regardless
	of whether the operation is a read or write.
 */
module QSPIDeviceInterface #(
	parameter						INSN_BYTES		= 1,

	localparam						INSN_BITS 		= INSN_BYTES*8,
	localparam						INSN_NIBBLES	= INSN_BYTES * 2
)(
	input wire						clk,

	input wire						sck,
	input wire						cs_n,
	inout wire[3:0]					dq,

	output logic					start		= 0,	//single cycle strobe indicating CS# falling edge
	output logic					insn_valid	= 0,	//single cycle strobe indicating instruction is valid
	output logic[INSN_BITS - 1 : 0]	insn		= 0,
	output logic					wr_valid	= 0,	//single cycle strobe indicating wr_data is valid
	output logic[7:0]				wr_data		= 0,

	input wire						rd_mode,			//combinatorially set by parent module simultaneous with
														//insn_valid. True if executing a read from the FPGA, false
														//if writing data to FPGA.
	output logic					rd_ready	= 0,	//single cycle strobe indicating ready for more write data
	input wire						rd_valid,			//single cycle strobe indicating rd_data is valid
	input wire[7:0]					rd_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O buffers

	logic[3:0]	dq_oe	= 4'b0;	//default all outputs to tristate

	wire[3:0]	dq_in;
	logic[3:0]	dq_out	= 0;

	BidirectionalBuffer #(
		.WIDTH(4)
	) dq_iobuf (
		.fabric_in(dq_in),
		.fabric_out(dq_out),
		.pad(dq),
		.oe(dq_oe)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input synchronizers

	wire		sck_sync;
	wire		cs_n_sync;

	wire[3:0]	dq_in_sync;

	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_sck (
		.clk_in(clk),
		.din(sck),
		.clk_out(clk),
		.dout(sck_sync)
	);

	ThreeStageSynchronizer #(
		.INIT(1),
		.IN_REG(0)
	) sync_cs (
		.clk_in(clk),
		.din(cs_n),
		.clk_out(clk),
		.dout(cs_n_sync)
	);

	for(genvar g=0; g<4; g=g+1) begin
		ThreeStageSynchronizer #(
			.INIT(0),
			.IN_REG(0)
		) sync_dq (
			.clk_in(clk),
			.din(dq_in[g]),
			.clk_out(clk),
			.dout(dq_in_sync[g])
		);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Detect falling CS# edges and rising SCK edges

	logic	cs_n_ff		= 1;
	logic	sck_ff		= 0;
	always_ff @(posedge clk) begin
		cs_n_ff		<= cs_n_sync;
		sck_ff		<= sck_sync;
		start		<= !cs_n_sync && cs_n_ff;
	end

	logic	sck_rising;
	always_comb begin
		sck_rising	= sck_sync && !sck_ff;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	enum logic[3:0]
	{
		STATE_DESELECTED 	= 4'h0,
		STATE_INSN			= 4'h1,
		STATE_ADDRESS		= 4'h2,
		STATE_TURNAROUND	= 4'h3,

		STATE_WR_HI			= 4'h4,
		STATE_WR_LO			= 4'h5

	} state = STATE_DESELECTED;

	logic[3:0] insn_count = 0;
	logic[3:0] dq_in_ff	= 0;

	always_ff @(posedge clk) begin
		insn_valid	<= 0;
		rd_ready	<= 0;
		wr_valid	<= 0;

		//Most stuff happens on SCK rising edge
		if(sck_rising) begin

			dq_in_ff	<= dq_in_sync;

			case(state)

				//Nothing to do
				STATE_DESELECTED: begin
				end	//end STATE_DESELECTED

				//Instruction
				STATE_INSN: begin
					insn			<= { insn[INSN_BITS-4:0], dq_in_sync };
					insn_count		<= insn_count + 1;

					if( (insn_count + 1) == INSN_NIBBLES ) begin
						insn_valid	<= 1;

						//TODO: optionally go to address cycle
						state		<= STATE_TURNAROUND;
					end

				end	//end STATE_INSN

				//Address TODO
				STATE_ADDRESS: begin
				end	//end STATE_ADDRESS

				//Bus turnaround
				STATE_TURNAROUND: begin

					//TODO: handle read data path
					state	<= STATE_WR_HI;

				end	//end STATE_TURNAROUND

				//Write data path
				STATE_WR_HI: begin
					state	<= STATE_WR_LO;
				end	//end STATE_WR_HI

				STATE_WR_LO: begin
					wr_valid	<= 1;
					wr_data		<= { dq_in_ff, dq_in_sync };
					state		<= STATE_WR_HI;
				end	//end STATE_WR_LO

			endcase

		end

		if(cs_n_sync)
			state		<= STATE_DESELECTED;

		if(start) begin
			state 		<= STATE_INSN;
			insn_count	<= 0;
			insn		<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug logic analyzer

	ila_2 ila(
		.clk(clk),
		.probe0(sck_sync),
		.probe1(cs_n_sync),
		.probe2(dq_in_sync),
		.probe3(dq_oe),
		.probe4(dq_out),

		.probe5(start),
		.probe6(insn),
		.probe7(insn_valid),
		.probe8(state),
		.probe9(wr_valid),
		.probe10(wr_data),
		.probe11(rd_mode),
		.probe12(rd_ready),
		.probe13(rd_valid),
		.probe14(rd_data),
		.probe15(sck_rising)
	);

endmodule
