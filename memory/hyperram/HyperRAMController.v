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

	input wire			clk,		//Main clock, up to 100 MHz in 3.3V or 166 MHz in 1.8V devices
	input wire			clk_90,		//Copy of clk with a -90 degree phase offset (+270)

	//RAM bus
	output wire			ram_clk_p,	//Differential clock to the RAM
	output wire			ram_clk_n,	//(clk_n may be unused for 3.3V parts)
	inout wire[7:0]		ram_dq,
	inout wire			ram_dqs,
	output reg			ram_cs_n = 1,
	output reg			ram_rst_n = 1,

	//Control signals

	//Status signals
	output reg			status_valid	= 0,
	output reg			ram_ok			= 0,
	output reg[4:0]		num_col_addrs	= 0,
	output reg[5:0]		num_row_addrs	= 0,
	output reg[7:0]		capacity_mbits	= 0
);

	parameter SIM = 0;

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
	// TODO: delay lines for input capture? Or do we even need this at 333 MT/s?

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
	// Oversampling for DQS alignment? Doesn't seem necessary at this point

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Inbound DQ-to-strobe alignment

	//If dqs_in is 2'b10, we're correctly aligned to the burst
	//If 2'b01, we need to bit slip by one DDR clock cycle
	//Delay the data-valid flag during that
	reg[15:0]	ram_dq_in_ff		= 0;
	reg[1:0]	ram_dqs_in_ff		= 0;
	reg[15:0]	ram_dq_in_aligned;
	reg			ram_dq_in_valid;
	always @(*) begin

		//clear everything once chip is deselected
		if(ram_cs_n) begin
			ram_dq_in_aligned	<= 16'h0;
			ram_dq_in_valid		<= 1'b0;
		end

		else if(ram_dqs_in == 2'b10) begin
			ram_dq_in_aligned	<= ram_dq_in;
			ram_dq_in_valid		<= 1;
		end
		else if(ram_dqs_in == 2'b01) begin
			ram_dq_in_aligned	<= {ram_dq_in_ff[7:0], ram_dq_in[15:8]};
			ram_dq_in_valid		<= (ram_dqs_in_ff == 2'b01);
		end
		else begin
			ram_dq_in_aligned	<= 16'h0;
			ram_dq_in_valid		<= 1'b0;
		end
	end

	always @(posedge clk) begin
		ram_dq_in_ff	<= ram_dq_in;
		ram_dqs_in_ff	<= ram_dqs_in;
		ram_dq_oe_ff 	<= ram_dq_oe;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PHY state machine

	localparam PHY_STATE_IDLE		= 4'h0;
	localparam PHY_STATE_UADDR		= 4'h1;
	localparam PHY_STATE_LADDR		= 4'h2;
	localparam PHY_STATE_DQS_WAIT	= 4'h3;
	localparam PHY_STATE_PRE_DATA	= 4'h4;
	localparam PHY_STATE_DATA		= 4'h5;

	reg[3:0]	phy_state		= PHY_STATE_IDLE;
	reg[7:0]	burst_count		= 0;

	//Burst status
	reg			burst_en			= 0;
	reg[7:0]	burst_len			= 0;
	reg			burst_done			= 0;

	//Configuration for a read/write burst
	reg			burst_rd_en			= 0;	//0 for writes
	reg			burst_is_reg		= 0;	//1 for register access
	reg			burst_is_linear		= 0;	//0 for wrapped
	reg[28:0]	burst_upper_addr	= 0;
	//13-bit reserved field here, always zero
	reg[2:0]	burst_lower_addr	= 0;

	always @(posedge clk) begin

		burst_done				<= 0;

		case(phy_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for a burst command

			PHY_STATE_IDLE: begin

				//Default to tristating everything and not selecting the chip
				ram_dqsdm_oe	<= 0;
				ram_dq_oe		<= 0;
				ram_cs_n		<= 1;

				//TODO: handle refreshes
				if(burst_en) begin
					ram_cs_n				<= 0;

					ram_dq_oe				<= 1;
					ram_dqsdm_oe			<= 0;
					clk_oe					<= 1;
					ram_dq_out				<= { burst_rd_en, burst_is_reg, burst_is_linear, burst_upper_addr[28:16] };

					phy_state				<= PHY_STATE_UADDR;

				end

			end	//end PHY_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Main burst path

			PHY_STATE_UADDR: begin
				ram_dq_out				<= burst_upper_addr[15:0];
				phy_state				<= PHY_STATE_LADDR;
			end	//end PHY_STATE_UADDR

			PHY_STATE_LADDR: begin
				ram_dq_out				<= { 13'h0, burst_lower_addr };
				phy_state				<= PHY_STATE_DQS_WAIT;
			end	//end PHY_STATE_LADDR

			PHY_STATE_DQS_WAIT: begin
				ram_dq_oe				<= 0;
				ram_dqsdm_oe			<= 0;

				burst_count				<= burst_len;

				//Wait for DQS to go low (indicating read command is being processed)
				if(ram_dqs_in == 0)
					phy_state			<= PHY_STATE_PRE_DATA;
			end	//end PHY_STATE_DQS_WAIT

			PHY_STATE_PRE_DATA: begin

				//Wait for DQS to have data on it
				if(ram_dqs_in)
					phy_state			<= PHY_STATE_DATA;

			end	//end PHY_STATE_PRE_DATA

			PHY_STATE_DATA: begin

				burst_count				<= burst_count - 1'h1;

				if(burst_count == 1) begin
					phy_state			<= PHY_STATE_IDLE;
					ram_cs_n			<= 1;
					clk_oe				<= 0;

					burst_done			<= 1;
				end

			end	//end PHY_STATE_DATA

		endcase

	end

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
	localparam STATE_BOOT_3 	= 4'h4;

	localparam STATE_BOOT_HOLD	= 4'hf;

	reg[3:0]	state			= SIM ? STATE_BOOT_0 : STATE_BOOT_HOLD;

	reg[7:0]	count			= 0;

	//debug
	wire		trig_out;

	always @(posedge clk) begin

		burst_en				<= 0;

		case(state)

			STATE_BOOT_HOLD: begin
				if(trig_out)
					state		<= STATE_BOOT_0;
			end

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for commands

			STATE_IDLE: begin
			end	//end STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// BOOT - initial register configuration

			//Reset the chip
			STATE_BOOT_0: begin
				ram_rst_n		<= 0;
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
					burst_en			<= 1;
					burst_len			<= 1;			//ID register 0 is only one word

					burst_rd_en			<= 1;
					burst_is_reg		<= 1;
					burst_is_linear		<= 1;
					burst_upper_addr	<= 28'h0;
					burst_lower_addr	<= 3'h0;
					state				<= STATE_BOOT_3;
				end
			end	//end STATE_BOOT_2

			//Wait for config read to finish
			STATE_BOOT_3: begin

				//TODO: read config register 1 too
				if(burst_done) begin
					state				<= STATE_IDLE;
					status_valid		<= 1;
				end

				if(ram_dq_in_valid) begin

					//Verify that we have a valid HyperRAM part made by Cypress
					if(ram_dq_in_aligned[3:0] != 4'b0001)
						ram_ok				<= 0;

					//Save register data
					else begin
						ram_ok				<= 1;
						num_col_addrs		<= ram_dq_in_aligned[7:4] + 1'h1;
						num_row_addrs		<= ram_dq_in_aligned[12:8] + 1'h1;
						//TODO: support multi-die parts

						//Calculate capacity in Mbits (2^20 bits)
						//Addresses are in words (16 bits). 1 Mb = 2^16 words.
						//Shift by 14 since row/col address counts are offset-1 coded.
						capacity_mbits		<= 1 << (ram_dq_in_aligned[7:4] + ram_dq_in_aligned[12:8] - 4'd14);

						//TODO: figure out actual device capacity / max address from this info
					end
				end
			end	//end STATE_BOOT_3

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	generate

		if(!SIM) begin

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
				.probe4(phy_state),
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
				.probe17(ram_dqs_in_ff),
				.probe18(ram_dq_in_valid),

				.probe19(status_valid),
				.probe20(ram_ok),
				.probe21(num_col_addrs),
				.probe22(num_row_addrs),
				.probe23(capacity_mbits),

				.trig_out(trig_out),
				.trig_out_ack(trig_out_ack)
			);

		end

		else
			assign trig_out = 0;

	endgenerate

endmodule
