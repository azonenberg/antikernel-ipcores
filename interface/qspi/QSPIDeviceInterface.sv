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
	@file
	@author Andrew D. Zonenberg
	@brief QSPI device-mode transceiver based on oversampling (does not require clock capable input for SCK)

	Only supports x4 mode for now, no x1 support.

	clk frequency must be at least 4x SCK frequency.

	A single cycle bus turnaround / dummy stage is present after the instruction/address and before data, regardless
	of whether the operation is a read or write.

	(TODO: update this)
 */
module QSPIDeviceInterface #(
	parameter						INSN_BYTES		= 1,
	parameter						DDR_MODE		= 0,

	localparam						INSN_BITS 		= INSN_BYTES*8,
	localparam						INSN_NIBBLES	= INSN_BYTES * 2
)(
	input wire						clk,

	input wire						sck,
	input wire						cs_n,
	inout wire[3:0]					dq,

	output logic					start		= 0,	//single cycle strobe indicating CS# falling edge
	output logic					stop		= 0,	//single cycle strobe indicating CS# rising edge
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

	logic		dq_oe	= 1'b0;	//default all outputs to tristate

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

	(* keep_hierarchy = "yes" *)
	ThreeStageSynchronizer #(
		.INIT(0),
		.IN_REG(0)
	) sync_sck (
		.clk_in(clk),
		.din(sck),
		.clk_out(clk),
		.dout(sck_sync)
	);

	(* keep_hierarchy = "yes" *)
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
		(* keep_hierarchy = "yes" *)
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
	// Detect rising/falling CS# edges and rising SCK edges

	logic	cs_n_ff		= 1;
	logic	sck_ff		= 0;
	always_ff @(posedge clk) begin
		cs_n_ff		<= cs_n_sync;
		sck_ff		<= sck_sync;
		start		<= !cs_n_sync && cs_n_ff;
		stop		<= cs_n_sync && !cs_n_ff;
	end

	logic	sck_rising;
	logic	sck_falling;
	always_comb begin
		sck_rising	= sck_sync && !sck_ff;
		sck_falling	= !sck_sync && sck_ff;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main state machine

	enum logic[3:0]
	{
		STATE_DESELECTED 	= 4'h0,
		STATE_INSN			= 4'h1,
		STATE_TURNAROUND	= 4'h2,

		STATE_WR_HI			= 4'h3,
		STATE_WR_LO			= 4'h4,
		STATE_WR_DELAY		= 4'h5,

		STATE_RD_HI			= 4'h6,
		STATE_RD_LO			= 4'h7

	} state = STATE_DESELECTED;

	logic[3:0]	insn_count = 0;
	logic[3:0]	dq_in_ff	= 0;

	//Combinatorial forwarding of read data
	//Register it so we have it ready for when it's needed
	logic[7:0]	rd_data_next	= 0;
	logic[7:0]	rd_data_next_fwd;
	always_comb begin
		rd_data_next_fwd		= rd_data_next;
		if(rd_valid)
			rd_data_next_fwd	= rd_data;
	end

	logic	sck_edge_of_interest;

	//Combinatorially assert data-ready flag
	always_comb begin
		rd_ready		= 0;

		sck_edge_of_interest = sck_rising || (DDR_MODE && sck_falling);

		//Request first data byte as soon as we get the instruction
		if(insn_valid)
			rd_ready	= 1;

		//Request additional data as we send the last of the current byte
		if( (state == STATE_RD_LO) && sck_edge_of_interest )
			rd_ready	= 1;

	end

	always_ff @(posedge clk) begin
		insn_valid		<= 0;
		wr_valid		<= 0;

		rd_data_next	<= rd_data_next_fwd;

		//Write path stuff happens on SCK rising edge, nothing odd there.
		//Read path executes on SCK rising edge too, but important to note that our view of the edge is delayed by two
		//cycles due to synchronizer sampling latency. Driving on the delayed rising edge gives plenty of hold time for
		//the receiver and maximizes our setup margin for the upcoming clock edge.
		if(sck_edge_of_interest) begin

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

						state		<= STATE_TURNAROUND;
					end

				end	//end STATE_INSN

				//Bus turnaround
				STATE_TURNAROUND: begin
					if(rd_mode) begin

						//Start driving outputs
						dq_oe	<= 1;

						if(DDR_MODE)
							state	<= STATE_RD_HI;
						else
							state	<= STATE_RD_LO;
						dq_out	<= rd_data_next_fwd[7:4];

					end
					else begin
						if(DDR_MODE)
							state	<= STATE_WR_DELAY;
						else
							state	<= STATE_WR_HI;
					end
				end	//end STATE_TURNAROUND

				//Write data path
				STATE_WR_DELAY: begin
					state	<= STATE_WR_HI;
				end	//end STATE_WR_DELAY

				STATE_WR_HI: begin
					state	<= STATE_WR_LO;
				end	//end STATE_WR_HI

				STATE_WR_LO: begin
					wr_valid	<= 1;
					wr_data		<= { dq_in_ff, dq_in_sync };
					state		<= STATE_WR_HI;
				end	//end STATE_WR_LO

				STATE_RD_HI: begin
					state		<= STATE_RD_LO;
					dq_out		<= rd_data_next_fwd[7:4];
				end	//end STATE_RD_HI

				STATE_RD_LO: begin
					state		<= STATE_RD_HI;
					dq_out		<= rd_data_next_fwd[3:0];
				end	//end STATE_RD_LO

				default: begin
				end

			endcase

		end

		if(cs_n_sync) begin
			state		<= STATE_DESELECTED;

			//Immediately tristate output when deselected no matter what else was going on
			dq_oe		<= 0;
		end

		if(start) begin
			state 		<= STATE_INSN;
			insn_count	<= 0;
			insn		<= 0;
		end

	end

endmodule
