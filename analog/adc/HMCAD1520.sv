`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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

`include "HMCAD1520.svh"

/**
	@brief Controller for a HMCAD1520 ADC
 */
module HMCAD1520(

	//Control domain clock (max 100 MHz)
	input wire		ctl_clk,

	//Bit clock
	input wire		lclk_p,
	input wire		lclk_n,

	//Frame clock (toggles once per sample)
	input wire		fclk_p,
	input wire		fclk_n,

	//Data bus
	input wire[3:0]	data_a_p,
	input wire[3:0]	data_a_n,
	input wire[3:0]	data_b_p,
	input wire[3:0]	data_b_n,

	//SPI bus
	output wire		spi_clk,
	output wire		spi_data,
	output logic	spi_cs_n	= 1,

	//Control signals
	output logic	pd			= 0,
	output logic	rst_n		= 1,

	//Interface to PLL (externally supplied)
	output wire		lclk,		//1/2x sample clock
	input wire		fast_clk,	//copy of LCLK
	input wire		slow_clk,	//1/8x sample clock

	//Waveform data (synchronous to slow_clk)
	output h1520bus_t	sample_bus = {$bits(h1520bus_t){1'b0}}
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SPI controller

	logic		shift_en	= 0;
	wire		shift_done;
	logic[7:0]	tx_data		= 0;

	SPITransceiver txvr(
		.clk(ctl_clk),
		.clkdiv(10),	//10 MHz @ 100 MHz ctl_clk. Allowed to be up to 20 MHz.
		.spi_sck(spi_clk),
		.spi_mosi(spi_data),
		.spi_miso(1'b0),

		.shift_en(shift_en),
		.shift_done(shift_done),
		.tx_data(tx_data),
		.rx_data()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SPI register access

	logic		reg_wr		= 0;
	logic[7:0]	reg_addr	= 0;
	logic[15:0]	reg_value	= 0;

	logic[2:0]	reg_state	= 0;
	logic		reg_done	= 0;

	always_ff @(posedge ctl_clk) begin

		shift_en	<= 0;
		reg_done	<= 0;

		case(reg_state)

			0: begin
				if(reg_wr) begin
					spi_cs_n	<= 0;
					reg_state	<= 1;
				end
			end

			1: begin
				shift_en	<= 1;
				tx_data		<= reg_addr;
				reg_state	<= 2;
			end

			2: begin
				if(shift_done) begin
					shift_en	<= 1;
					tx_data		<= reg_value[15:8];
					reg_state	<= 3;
				end
			end

			3: begin
				if(shift_done) begin
					shift_en	<= 1;
					tx_data		<= reg_value[7:0];
					reg_state	<= 4;
				end
			end

			4: begin
				if(shift_done) begin
					reg_done	<= 1;
					spi_cs_n	<= 1;
					reg_state	<= 0;
				end
			end

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Power-on initialization

	typedef struct packed
	{
		logic		regwr;
		logic[7:0]	regid;
		logic[15:0]	regval;
		logic		rstn;
		logic		pd;
	} ucode_t;

	ucode_t[15:0] ucode;

	initial begin
		ucode[0] <= {1'b0, 8'h00, 16'h0000, 1'h0, 1'h0};	//Reset cycle
		ucode[1] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[2] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h1};	//Powerdown cycle
		ucode[3] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[4] <= {1'b1, 8'h31, 16'h0001, 1'h1, 1'h0};	//Set mode to single channel

		ucode[5] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h1};	//Powerdown cycle
		ucode[6] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[7] <= {1'b1, 8'h3a, 16'h0202, 1'h1, 1'h0};	//Select channel 1 on all ADCs
		ucode[8] <= {1'b1, 8'h3b, 16'h0202, 1'h1, 1'h0};

		ucode[9] <= {1'b1, 8'h53, 16'h0000, 1'h1, 1'h0};	//??

		ucode[10] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h1};	//Powerdown cycle
		ucode[11] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[12] <= {1'b1, 8'h56, 16'h0008, 1'h1, 1'h0};	//??
		ucode[13] <= {1'b1, 8'h30, 16'h00ff, 1'h1, 1'h0};	//??

		//ucode[14] <= {1'b1, 8'h25, 16'h0040, 1'h1, 1'h0};	//full scale ramp test pattern

		ucode[14] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};	//No-op at end of table
		ucode[15] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};	//No-op at end of table
	end

	logic[3:0] ucode_addr = 0;
	ucode_t current_ucode;
	assign current_ucode = ucode[ucode_addr];

	logic[9:0]	count 		= 0;
	logic		rst_done	= 0;

	always_ff @(posedge ctl_clk) begin

		reg_wr	<= 0;

		if(!rst_done) begin

			count <= count + 1;
			if(count == 1023) begin
				count		<= 0;

				//Execute the next line of microcode
				reg_wr		<= current_ucode.regwr;
				reg_addr	<= current_ucode.regid;
				reg_value	<= current_ucode.regval;
				rst_n		<= current_ucode.rstn;
				pd			<= current_ucode.pd;

				//Move to the next line
				ucode_addr	<= ucode_addr + 1;
				if(ucode_addr == 15)
					rst_done	<= 1;

			end

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input buffering

	wire	lclk_raw;

	IBUFDS #(
		.IOSTANDARD("LVDS_25"),
		.IBUF_LOW_PWR("FALSE"),
		.DIFF_TERM("FALSE") //no ODT on INTEGRALSTICK clock input
							//TODO: fix this for other boards
	) ibuf_lclk (
		.I(lclk_p),
		.IB(lclk_n),
		.O(lclk_raw)
	);

	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("NO")
	) bufg_lclk(
		.clkin(lclk_raw),
		.ce(1),
		.clkout(lclk)
	);

	wire	fclk;

	IBUFDS #(
		.IOSTANDARD("LVDS_25"),
		.IBUF_LOW_PWR("FALSE"),
		.DIFF_TERM("TRUE")
	) ibuf_fclk (
		.I(fclk_p),
		.IB(fclk_n),
		.O(fclk)
	);

	wire[3:0]	data_a;
	wire[3:0]	data_b;
	for(genvar g=0; g<4; g++) begin
		IBUFDS #(
			.IOSTANDARD("LVDS_25"),
			.IBUF_LOW_PWR("FALSE"),
			.DIFF_TERM("TRUE")
		) ibuf_a (
			.I(data_a_p[g]),
			.IB(data_a_n[g]),
			.O(data_a[g])
		);

		IBUFDS #(
			.IOSTANDARD("LVDS_25"),
			.IBUF_LOW_PWR("FALSE"),
			.DIFF_TERM("TRUE")
		) ibuf_b (
			.I(data_b_p[g]),
			.IB(data_b_n[g]),
			.O(data_b[g])
		);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input data capture (8:1 SERDES)

	//Synchronize SERDES reset status into the RX local clock domain
	wire		rst_done_sync;
	ThreeStageSynchronizer sync_rst_done(
		.clk_in(ctl_clk),
		.din(rst_done),
		.clk_out(slow_clk),
		.dout(rst_done_sync)
	);

	//Reset all of the SERDES once the ADC is initialized
	logic		serdes_reset		= 0;
	logic[2:0]	serdes_reset_state	= 0;
	always_ff @(posedge slow_clk) begin
		case(serdes_reset_state)
			0: begin
				if(rst_done_sync) begin
					serdes_reset_state	<= 1;
					serdes_reset		<= 1;
				end
			end
			1: 	serdes_reset_state	<= 2;
			2:	serdes_reset_state	<= 3;
			3: 	serdes_reset		<= 0;
		endcase
	end

	logic		serdes_bitslip	= 0;

	//SERDES for FCLK
	wire[7:0]	fclk_data;
	ISERDESE2 #(
		.DATA_WIDTH(8),
		.INTERFACE_TYPE("NETWORKING")
	) iserdes_fclk (
		.Q1(fclk_data[7]),
		.Q2(fclk_data[6]),
		.Q3(fclk_data[5]),
		.Q4(fclk_data[4]),
		.Q5(fclk_data[3]),
		.Q6(fclk_data[2]),
		.Q7(fclk_data[1]),
		.Q8(fclk_data[0]),
		.O(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.D(fclk),
		.DDLY(),
		.CLK(fast_clk),
		.CLKB(!fast_clk),
		.CE1(1'b1),
		.CE2(1'b1),
		.RST(serdes_reset),
		.CLKDIV(slow_clk),
		.CLKDIVP(1'b0),
		.OCLK(),
		.OCLKB(),
		.BITSLIP(serdes_bitslip),
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0),
		.OFB(1'b0),
		.DYNCLKDIVSEL(1'b0),		//TODO: clock inversion
		.DYNCLKSEL(1'b0)
	);

	//Re-shuffle input data lanes into a format more amenable to ganged SERDES
	//7 = oldest, 0 = newest
	wire[7:0] rx_data = { data_a[0], data_b[0], data_a[1], data_b[1], data_a[2], data_b[2], data_a[3], data_b[3] };

	wire[7:0] rx_samples[7:0];

	for(genvar g=0; g<8; g++) begin
		ISERDESE2 #(
			.DATA_WIDTH(8),
			.INTERFACE_TYPE("NETWORKING")
		) iserdes_fclk (
			.Q1(rx_samples[g][7]),
			.Q2(rx_samples[g][6]),
			.Q3(rx_samples[g][5]),
			.Q4(rx_samples[g][4]),
			.Q5(rx_samples[g][3]),
			.Q6(rx_samples[g][2]),
			.Q7(rx_samples[g][1]),
			.Q8(rx_samples[g][0]),
			.O(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.D(rx_data[g]),
			.DDLY(),
			.CLK(fast_clk),
			.CLKB(!fast_clk),
			.CE1(1'b1),
			.CE2(1'b1),
			.RST(serdes_reset),
			.CLKDIV(slow_clk),
			.CLKDIVP(1'b0),
			.OCLK(),
			.OCLKB(),
			.BITSLIP(serdes_bitslip),
			.SHIFTIN1(1'b0),
			.SHIFTIN2(1'b0),
			.OFB(1'b0),
			.DYNCLKDIVSEL(1'b0),		//TODO: clock inversion
			.DYNCLKSEL(1'b0)
		);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Invert all data because of pin swapping

	//TODO: make this configurable
	logic[7:0] fclk_data_inv	= 0;
	always_ff @(posedge slow_clk) begin
		fclk_data_inv	<= ~fclk_data;
		for(integer i=0; i<8; i++)
			sample_bus.samples[i]	<= ~rx_samples[i];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input data phase alignment

	logic[7:0]	phase_window	= 0;
	logic[7:0]	phase_errs		= 0;

	always_ff @(posedge slow_clk) begin

		serdes_bitslip	<= 0;

		phase_window	<= phase_window + 1;

		//Nominal synchronization in 8-bit mode is FCLK high for first half of the sample (bits 0-3)
		//and low for the second half (bits 4-7)
		if(fclk_data_inv != 8'h0f)
			phase_errs	<= phase_errs + 1;

		//At end of the window, if we have more than a handful of errors, trigger a bitslip
		//Can't make the threshold too low because there's some latency in the bitslip and inversion path etc
		if(phase_window == 8'hff) begin
			phase_errs	<= 0;
			if(phase_errs > 16)
				serdes_bitslip	<= 1;
		end

	end

endmodule
