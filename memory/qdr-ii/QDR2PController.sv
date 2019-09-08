`default_nettype none
`timescale 1ns/1ps
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
	@brief	QDR-II+ controller (burst length 4 only)
	@author Andrew D. Zonenberg
 */
module QDR2PController #(
	parameter RAM_WIDTH		= 36,
	parameter ADDR_BITS		= 18,

	localparam CTRL_WIDTH	= RAM_WIDTH * 4
) (

	//Controller clock (1/2 RAM clock, 1/4 bit rate).
	//All control/data signals on the FPGA side are synchronous to this clock.
	input wire						clk_ctl,

	//RAM clock (1/2 bit rate, 2x clk_ctl rate)
	//All RAM signals are synchronous to this clock.
	input wire						clk_ram,

	input wire						clk_ram_90,

	//Interface to the FPGA side RAM controller
	input wire						rd_en,
	input wire[ADDR_BITS-1:0]		rd_addr,
	output logic					rd_valid	= 0,
	output logic[CTRL_WIDTH-1:0]	rd_data		= 0,
	input wire						wr_en,
	input wire[ADDR_BITS-1:0]		wr_addr,
	input wire[CTRL_WIDTH-1:0]		wr_data,

	//Interface to the RAM itself
	output wire[RAM_WIDTH-1:0]		qdr_d,
	input wire[RAM_WIDTH-1:0]		qdr_q,
	output wire[ADDR_BITS-1:0]		qdr_a,
	output wire						qdr_wps_n,
	output wire[3:0]				qdr_bws_n,
	output wire						qdr_rps_n,
	input wire						qdr_qvld,
	output wire						qdr_dclk_p,
	output wire						qdr_dclk_n,
	input wire						qdr_qclk_p,
	input wire						qdr_qclk_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clocking

	//Forward the RAM clock to the memory
	wire	clk_ram_ddr;
	ODDR #
	(
		.DDR_CLK_EDGE("SAME_EDGE"),
		.SRTYPE("ASYNC"),
		.INIT(0)
	) ram_clk_oddr
	(
		.C(clk_ram_90),
		.D1(1'b0),
		.D2(1'b1),
		.CE(1'b1),
		.R(1'b0),
		.S(1'b0),
		.Q(clk_ram_ddr)
	);

	OBUFDS #(
		.IOSTANDARD("HSTL_1_15"),
		.SLEW("FAST")
	) ram_clk_obuf (
		.O(qdr_dclk_p),
		.OB(qdr_dclk_n),
		.I(clk_ram_ddr)
	);

	//Receive and buffer the echoed clock
	wire	qclk_bufg;
	IBUFGDS #(
		.DIFF_TERM("TRUE"),
		.IOSTANDARD("DIFF_HSTL_1_15")
	) qclk_ibuf(
		.I(qdr_qclk_p),
		.IB(qdr_qclk_n),
		.O(qclk_bufg)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset

	logic[7:0]	rst_count = 1;
	logic		serdes_rst	= 1;

	always_ff @(posedge clk_ctl) begin
		if(serdes_rst)
			rst_count	<= rst_count + 1;
		if(rst_count == 0)
			serdes_rst	<= 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register output data

	logic[CTRL_WIDTH-1:0]	wr_data_ff	= 0;
	logic					wr_en_ff	= 0;

	always_ff @(posedge clk_ctl) begin
		wr_data_ff	<= wr_data;
		wr_en_ff	<= wr_en;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output drivers

	//Select signals are on alternating edges of the controller clock
	OSERDESE2 #(
		.DATA_RATE_OQ("SDR"),
		.DATA_RATE_TQ("BUF"),
		.DATA_WIDTH(2),
		.SERDES_MODE("MASTER"),
		.TRISTATE_WIDTH(1)
	) wps_oserdes (
		.OQ(qdr_wps_n),
		.OFB(),
		.TQ(),
		.TFB(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.CLK(clk_ram),
		.CLKDIV(clk_ctl),
		.D1(!wr_en),
		.D2(1'b1),
		.TCE(1'b1),
		.OCE(1'b1),
		.TBYTEIN(),
		.TBYTEOUT(),
		.RST(serdes_rst),
		.SHIFTIN1(),
		.SHIFTIN2(),
		.T1(1'b0)
	);

	OSERDESE2 #(
		.DATA_RATE_OQ("SDR"),
		.DATA_RATE_TQ("BUF"),
		.DATA_WIDTH(2),
		.SERDES_MODE("MASTER"),
		.TRISTATE_WIDTH(1)
	) rps_oserdes (
		.OQ(qdr_rps_n),
		.OFB(),
		.TQ(),
		.TFB(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.CLK(clk_ram),
		.CLKDIV(clk_ctl),
		.D1(1'b1),
		.D2(!rd_en),
		.TCE(1'b1),
		.OCE(1'b1),
		.TBYTEIN(),
		.TBYTEOUT(),
		.RST(serdes_rst),
		.SHIFTIN1(),
		.SHIFTIN2(),
		.T1(1'b0)
	);

	//Address bus takes turns between the two ports
	for(genvar i=0; i<ADDR_BITS; i++) begin
		OSERDESE2 #(
			.DATA_RATE_OQ("SDR"),
			.DATA_RATE_TQ("BUF"),
			.DATA_WIDTH(2),
			.SERDES_MODE("MASTER"),
			.TRISTATE_WIDTH(1)
		) addr_oserdes (
			.OQ(qdr_a[i]),
			.OFB(),
			.TQ(),
			.TFB(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.CLK(clk_ram),
			.CLKDIV(clk_ctl),
			.D1(wr_addr[i]),
			.D2(rd_addr[i]),
			.TCE(1'b1),
			.OCE(1'b1),
			.TBYTEIN(),
			.TBYTEOUT(),
			.RST(serdes_rst),
			.SHIFTIN1(),
			.SHIFTIN2(),
			.T1(1'b0)
		);
	end

	//Byte write enables are active any time we're writing
	for(genvar i=0; i<4; i++) begin
		OSERDESE2 #(
			.DATA_RATE_OQ("DDR"),
			.DATA_RATE_TQ("BUF"),
			.DATA_WIDTH(2),
			.SERDES_MODE("MASTER"),
			.TRISTATE_WIDTH(1)
		) bws_oserdes (
			.OQ(qdr_bws_n[i]),
			.OFB(),
			.TQ(),
			.TFB(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.CLK(clk_ram),
			.CLKDIV(clk_ctl),
			.D1(!wr_en_ff),
			.D2(!wr_en_ff),
			.D3(!wr_en),
			.D4(!wr_en),
			.TCE(1'b1),
			.OCE(1'b1),
			.TBYTEIN(),
			.TBYTEOUT(),
			.RST(serdes_rst),
			.SHIFTIN1(),
			.SHIFTIN2(),
			.T1(1'b0)
		);
	end

	//Data bus is double-rate
	//Need to rearrange things a bit to correct for phasing of clocks
	for(genvar i=0; i<RAM_WIDTH; i++) begin
		OSERDESE2 #(
			.DATA_RATE_OQ("DDR"),
			.DATA_RATE_TQ("BUF"),
			.DATA_WIDTH(4),
			.SERDES_MODE("MASTER"),
			.TRISTATE_WIDTH(1)
		) d_oserdes (
			.OQ(qdr_d[i]),
			.OFB(),
			.TQ(),
			.TFB(),
			.SHIFTOUT1(),
			.SHIFTOUT2(),
			.CLK(clk_ram),
			.CLKDIV(clk_ctl),
			.D1(wr_data_ff[RAM_WIDTH*1 + i]),
			.D2(wr_data_ff[RAM_WIDTH*0 + i]),
			.D3(wr_data[RAM_WIDTH*3 + i]),
			.D4(wr_data[RAM_WIDTH*2 + i]),
			.TCE(1'b1),
			.OCE(1'b1),
			.TBYTEIN(),
			.TBYTEOUT(),
			.RST(serdes_rst),
			.SHIFTIN1(),
			.SHIFTIN2(),
			.T1(1'b0)
		);
	end

endmodule
