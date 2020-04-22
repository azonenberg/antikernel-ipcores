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
	output logic	rst_n		= 1

	//output wire		pd,
	//output wire		rst_n
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

	ucode_t[13:0] ucode;

	initial begin
		ucode[0] <= {1'b0, 8'h00, 16'h0000, 1'h0, 1'h0};	//Reset cycle
		ucode[1] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[2] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h1};	//Powerdown cycle
		ucode[3] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[4] <= {1'b0, 8'h31, 16'h0001, 1'h1, 1'h0};	//Set mode to single channel

		ucode[5] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h1};	//Powerdown cycle
		ucode[6] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[7] <= {1'b0, 8'h3a, 16'h0202, 1'h1, 1'h0};	//Select channel 1 on all ADCs
		ucode[8] <= {1'b0, 8'h3b, 16'h0202, 1'h1, 1'h0};

		ucode[9] <= {1'b0, 8'h53, 16'h0000, 1'h1, 1'h0};	//??

		ucode[10] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h1};	//Powerdown cycle
		ucode[11] <= {1'b0, 8'h00, 16'h0000, 1'h1, 1'h0};

		ucode[12] <= {1'b0, 8'h56, 16'h0008, 1'h1, 1'h0};	//??
		ucode[13] <= {1'b0, 8'h30, 16'h00ff, 1'h1, 1'h0};	//??
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
			if(count == 1024) begin
				count		<= 0;

				//Execute the next line of microcode
				reg_wr		<= current_ucode.regwr;
				reg_addr	<= current_ucode.regid;
				reg_value	<= current_ucode.regval;
				rst_n		<= current_ucode.rstn;
				pd			<= current_ucode.pd;

				//Move to the next line
				ucode_addr	<= ucode_addr + 1;
				if(ucode_addr == 13)
					rst_done	<= 1;

			end

		end
	end


	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Boot time initialization

	logic[2:0] init_state	= 0;

	wire	trig_out;

	always_ff @(posedge ctl_clk) begin

		reg_wr	<= 0;

		case(init_state)

			0: begin
				if(rst_done && trig_out) begin
					reg_wr		<= 1;
					reg_addr	<= 8'h31;		//channel_num / clk_divide
					reg_value	<= 16'h1;		//single channel, no clock divider

					init_state	<= 1;
				end
			end

			1: begin
				if(reg_done) begin
					reg_wr		<= 1;
					reg_addr	<= 8'h3a;		//inp_sel_adc1/2
					reg_value	<= 16'h0202;	//in1 / in1

					init_state	<= 2;
				end
			end

			2: begin
				if(reg_done) begin
					reg_wr		<= 1;
					reg_addr	<= 8'h3b;		//inp_sel_adc3/4
					reg_value	<= 16'h0202;	//in1 / in1

					init_state	<= 3;
				end
			end

			//powerdown again?

		endcase

	end
	*/

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input buffering

	wire	lclk_raw;
	wire	lclk;
	DifferentialInputBuffer #(
		.IOSTANDARD("LVDS_25"),
		.ODT(0)	//no ODT on INTEGRALSTICK clock input
				//TODO: fix this for other boards
	) ibuf_lclk(
		.pad_in_p(lclk_p),
		.pad_in_n(lclk_n),
		.fabric_out(lclk_raw)
	);
	ClockBuffer bufh_lclk(.clkin(lclk_raw), .ce(1), .clkout(lclk));

	wire	fclk;
	DifferentialInputBuffer #(
		.IOSTANDARD("LVDS_25")
	) ibuf_fclk (
		.pad_in_p(fclk_p),
		.pad_in_n(fclk_n),
		.fabric_out(fclk)
	);

	wire[3:0]	data_a;
	DifferentialInputBuffer #(
		.WIDTH(4),
		.IOSTANDARD("LVDS_25")
	) ibuf_data_a(
		.pad_in_p(data_a_p),
		.pad_in_n(data_a_n),
		.fabric_out(data_a)
	);

	wire[3:0]	data_b;
	DifferentialInputBuffer #(
		.WIDTH(4),
		.IOSTANDARD("LVDS_25")
	) ibuf_data_b(
		.pad_in_p(data_b_p),
		.pad_in_n(data_b_n),
		.fabric_out(data_b)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input data capture (4:1 serialization)

	//For now, assume all inputs inverted including FCLK

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA
	/*
	ila_1 ila(
		.clk(ctl_clk),
		.probe0(rst_done),
		.probe1(init_state),
		.probe2(reg_state),
		.probe3(reg_value),
		.probe4(reg_wr),
		.probe5(reg_addr),
		.probe6(reg_done),
		.probe7(shift_en),
		.probe8(shift_done),
		.probe9(tx_data),
		.probe10(pd),
		.probe11(rst_n),
		.trig_out(trig_out),
		.trig_out_ack(trig_out)
	);
	*/

endmodule
