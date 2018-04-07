`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
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
	@brief Controller for Cypress HyperRAM
 */
module HyperRAMController(

	input wire		clk,		//Main clock, up to 100 MHz in 3.3V or 166 MHz in 1.8V devices
	input wire		clk_90,		//Copy of clk with a -90 degree phase offset (+270)

	output wire		ram_clk_p,	//Differential clock to the RAM
	output wire		ram_clk_n,	//TODO: move this out of the controller
								//so we can support 3.3V parts w/ single ended clk?

	inout wire[7:0]	ram_dq,
	inout wire		ram_dqs,
	output reg		ram_cs_n = 1,
	output reg		ram_rst_n = 1
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O buffers

	//Echo our 90-degree clock to the RAM
	//(clk is center aligned to all of our signals)

	reg		clk_oe	= 0;

	DDROutputBuffer #(
		.WIDTH(1)
	) clk_buf_p (
		.clk_p(clk_90),
		.clk_n(!clk_90),
		.dout(ram_clk_p),
		.din0(1'b0),
		.din1(clk_oe)
	);

	DDROutputBuffer #(
		.WIDTH(1)
	) clk_buf_n (
		.clk_p(clk_90),
		.clk_n(!clk_90),
		.dout(ram_clk_n),
		.din0(1'b1),
		.din1(!clk_oe)
	);

	wire[7:0]	ram_dq_in_ddr;
	wire[7:0]	ram_dq_out_ddr;
	reg			ram_dq_oe = 0;
	reg			ram_dq_oe_ff;

	BidirectionalBuffer #(
		.WIDTH(8),
		.OE_INVERT(1)
	) dq_iobuf (
		.fabric_in(ram_dq_in_ddr),
		.fabric_out(ram_dq_out_ddr),
		.pad(ram_dq),
		.oe(ram_dq_oe_ff)
	);

	wire		ram_dqs_in_ddr;
	wire		ram_dm_out_ddr;
	reg			ram_dqsdm_oe	= 0;

	BidirectionalBuffer #(
		.WIDTH(1),
		.OE_INVERT(1)
	) dqs_iobuf (
		.fabric_in(ram_dqs_in_ddr),
		.fabric_out(ram_dm_out_ddr),
		.pad(ram_dqs),
		.oe(ram_dqsdm_oe)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TODO: delay lines for input capture

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DDR buffers

	reg[15:0]	ram_dq_out	= 0;

	DDROutputBuffer #(
		.WIDTH(8)
	) dq_ddr_obuf (
		.clk_p(clk),
		.clk_n(!clk),
		.dout(ram_dq_out_ddr),
		.din0(ram_dq_out[15:8]),
		.din1(ram_dq_out[7:0])
	);

	wire[15:0]	ram_dq_in;

	DDRInputBuffer #(
		.WIDTH(8)
	) dq_ddr_ibuf (
		.clk_p(clk),
		.clk_n(!clk),
		.dout0(ram_dq_in[15:8]),
		.dout1(ram_dq_in[7:0]),
		.din(ram_dq_in_ddr)
	);

	wire[1:0]	ram_dqs_in;
	DDRInputBuffer #(
		.WIDTH(1)
	) dqs_ddr_ibuf (
		.clk_p(clk),
		.clk_n(!clk),
		.dout0(ram_dqs_in[1]),
		.dout1(ram_dqs_in[0]),
		.din(ram_dqs_in_ddr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Oversampling for DQS alignment

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main control state machine

	/*
		A burst contains three 16-bit words (48 bits total) of command/address
		47		rd / wr_n
		46		register (0 = normal memory)
		45		linear burst (0 = wrapped)
		44:16	row / upper column address
		15:3	Reserved, should be zero
		2:0		lower column address
	 */

	localparam STATE_IDLE		= 4'h0;
	localparam STATE_BOOT_0		= 4'h1;
	localparam STATE_BOOT_1		= 4'h2;
	localparam STATE_BOOT_2 	= 4'h3;
	localparam STATE_BURST_0	= 4'h4;
	localparam STATE_BURST_1	= 4'h5;
	localparam STATE_BURST_2	= 4'h6;
	localparam STATE_BURST_3	= 4'h7;
	localparam STATE_BURST_4	= 4'h8;
	localparam STATE_BURST_5	= 4'h9;

	localparam STATE_BOOT_HOLD	= 4'hf;

	reg[3:0]	state			= STATE_BOOT_HOLD;
	reg[3:0]	state_ret		= STATE_IDLE;

	reg[7:0]	count			= 0;

	//Configuration for a read/write burst
	reg			burst_rd_en			= 0;	//0 for writes
	reg			burst_is_reg		= 0;	//1 for register access
	reg			burst_is_linear		= 0;	//0 for wrapped
	reg[28:0]	burst_upper_addr	= 0;
	//13-bit reserved field here, always zero
	reg[2:0]	burst_lower_addr	= 0;

	reg[7:0]	burst_count			= 0;

	wire		trig_out;

	//If dqs_in is 2'b10, we're correctly aligned to the burst
	//If 2'b01, we need to bit slip by one DDR clock cycle
	reg[15:0]	ram_dq_in_ff		= 0;
	reg[15:0]	ram_dq_in_aligned;
	always @(*) begin
		if(ram_dqs_in == 2'b10)
			ram_dq_in_aligned	<= ram_dq_in;
		else if(ram_dqs_in == 2'b01)
			ram_dq_in_aligned	<= {ram_dq_in_ff[7:0], ram_dq_in[15:8]};
		else
			ram_dq_in_aligned	<= 16'h0;
	end

	always @(posedge clk) begin

		ram_dq_in_ff	<= ram_dq_in;
		ram_dq_oe_ff 	<= ram_dq_oe;

		case(state)

			STATE_BOOT_HOLD: begin
				if(trig_out)
					state		<= STATE_BOOT_0;
			end

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for commands

			STATE_IDLE: begin

				//Default to tristating everything and not selecting the chip
				ram_dqsdm_oe	<= 0;
				ram_dq_oe		<= 0;
				ram_cs_n		<= 1;


			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BOOT - initial register configuration

			//Reset the chip
			STATE_BOOT_0: begin

				ram_dqsdm_oe	<= 0;
				ram_dq_oe		<= 0;
				ram_rst_n		<= 0;
				ram_cs_n		<= 1;
				count			<= 0;

				state			<= STATE_BOOT_1;

			end	//end STATE_BOOT_0

			//Datasheet says we need a minimum of 200 ns low
			//Assuming we're clocked at 166 MHz that's a 6 ns clock period or 33.33 cycles
			//Do 64 cycles since that's a nice round number.
			STATE_BOOT_1: begin
				count			<= count + 1'h1;
				if(count == 63) begin
					ram_rst_n	<= 1;
					count		<= 0;
					state		<= STATE_BOOT_2;
				end
			end	//end STATE_BOOT_1

			//Need 400 ns between RST# low and chip select
			//Round a bit and do 128 clocks.
			//Kick off the first config register read (figure out device capacity)
			STATE_BOOT_2: begin
				count			<= count + 1'h1;
				if(count == 127) begin
					count				<= 0;
					state_ret			<= STATE_IDLE;
					burst_rd_en			<= 1;
					burst_is_reg		<= 1;
					burst_is_linear		<= 1;
					burst_upper_addr	<= 28'h0;
					burst_lower_addr	<= 3'h0;
					state				<= STATE_BURST_0;
				end
			end	//end STATE_BOOT_2

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BURST - read/write burst

			STATE_BURST_0: begin
				ram_cs_n				<= 0;

				ram_dq_oe				<= 1;
				ram_dqsdm_oe			<= 0;
				clk_oe					<= 1;
				ram_dq_out				<= { burst_rd_en, burst_is_reg, burst_is_linear, burst_upper_addr[28:16] };

				state					<= STATE_BURST_1;
			end

			STATE_BURST_1: begin
				ram_dq_out				<= burst_upper_addr[15:0];

				state					<= STATE_BURST_2;
			end	//end STATE_BURST_1

			STATE_BURST_2: begin
				ram_dq_out				<= { 13'h0, burst_lower_addr };

				state					<= STATE_BURST_3;
			end	//end STATE_BURST_2

			STATE_BURST_3: begin

				ram_dq_oe				<= 0;
				ram_dqsdm_oe			<= 0;

				//TODO: set to actual burst length
				burst_count				<= 1;

				//Wait for DQS to go low (indicating read command is being processed)
				if(ram_dqs_in == 0)
					state					<= STATE_BURST_4;
			end	//end STATE_BURST_3

			//prepare for readback
			STATE_BURST_4: begin

				//Wait for DQS to have data on it
				if(ram_dqs_in)
					state				<= STATE_BURST_5;

			end

			STATE_BURST_5: begin

				burst_count				<= burst_count - 1'h1;

				if(burst_count == 0) begin
					state				<= state_ret;
					ram_cs_n			<= 1;
					clk_oe				<= 0;
				end
			end	//end STATE_BURST_5

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	reg		trig_out_ack	= 0;

	always @(posedge clk) begin
		trig_out_ack	<= trig_out;
	end

	ila_0 ila(
		.clk(clk),

		.probe0(ram_dq_oe),
		.probe1(ram_dqsdm_oe),
		.probe2(ram_rst_n),
		.probe3(state),
		.probe4(state_ret),
		.probe5(ram_dq_out),
		.probe6(ram_dq_in),
		.probe7(burst_rd_en),
		.probe8(burst_is_reg),
		.probe9(burst_is_linear),
		.probe10(burst_upper_addr),
		.probe11(burst_lower_addr),
		.probe12(ram_dqs_in),
		.probe13(ram_dq_oe),
		.probe14(ram_dqsdm_oe),
		.probe15(ram_cs_n),
		.probe16(ram_dq_in_aligned),

		.trig_out(trig_out),
		.trig_out_ack(trig_out_ack)
	);

endmodule
