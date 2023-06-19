`default_nettype none
`timescale 1ns/1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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
	parameter CLK_PERIOD	= 2.666,

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
	output wire						pll_lock,

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
		.D1(1'b1),
		.D2(1'b0),
		.CE(1'b1),
		.R(1'b0),
		.S(1'b0),
		.Q(clk_ram_ddr)
	);

	OBUFDS #(
		.SLEW("FAST")
	) ram_clk_obuf (
		.O(qdr_dclk_p),
		.OB(qdr_dclk_n),
		.I(clk_ram_ddr)
	);

	//Inut buffer for echoed clock
	wire	qclk;
	IBUFDS qclk_ibuf(
		.I(qdr_qclk_p),
		.IB(qdr_qclk_n),
		.O(qclk)
	);

	//TODO: We might be able to save some power (and avoid burning a PLL)
	//if we were to use an IODELAY instead of phase shifting the clock?
	//Need to see if this will be good enough in terms of jitter etc

	wire	fbclk;

	wire	clk_qcapture_raw;
	wire	clk_qcapture_div2_raw;

	wire	clk_qcapture;
	wire	clk_qcapture_div2;

	PLLE2_BASE #(
		.BANDWIDTH("OPTIMIZED"),
		.CLKOUT0_DIVIDE(4),			//VCO/4 = original RAM clock frequency again
		.CLKOUT1_DIVIDE(10),
		.CLKOUT2_DIVIDE(10),
		.CLKOUT3_DIVIDE(10),		//unused
		.CLKOUT4_DIVIDE(10),		//unused
		.CLKOUT5_DIVIDE(10),		//unused

		.CLKOUT0_PHASE(90),
		.CLKOUT1_PHASE(0),
		.CLKOUT2_PHASE(0),
		.CLKOUT3_PHASE(0),
		.CLKOUT4_PHASE(0),
		.CLKOUT5_PHASE(0),

		.CLKOUT0_DUTY_CYCLE(0.5),
		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5),

		.CLKFBOUT_MULT(4),			//RAM clock * 4 should be in our VCO range
		.DIVCLK_DIVIDE(1),
		.CLKFBOUT_PHASE(0),

		.CLKIN1_PERIOD(CLK_PERIOD),
		.STARTUP_WAIT("FALSE")

	) ram_pll (
		.CLKIN1(qclk),
		.CLKFBIN(fbclk),
		.RST(1'b0),
		.PWRDWN(1'b0),
		.CLKOUT0(clk_qcapture_raw),
		.CLKOUT1(),
		.CLKOUT2(),
		.CLKOUT3(),
		.CLKOUT4(),
		.CLKOUT5(),
		.CLKFBOUT(fbclk),
		.LOCKED(pll_lock)
	);

	//Need to handle three clock regions wide
	//TODO: is BUFMR + BUFR usable here?
	//For now, use global buffers unless we run out
	BUFGCE buf_qcapture(
		.I(clk_qcapture_raw),
		.O(clk_qcapture),
		.CE(pll_lock));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register output data

	logic[CTRL_WIDTH-1:0]	wr_data_ff		= 0;
	logic[CTRL_WIDTH-1:0]	wr_data_ff2		= 0;
	logic					wr_en_ff		= 0;
	logic					wr_en_ff_ctl	= 0;
	logic					wr_phase		= 0;

	always_ff @(posedge clk_ctl) begin
		wr_en_ff_ctl	<= wr_en;
	end

	always_ff @(posedge clk_ram) begin

		//Mux low or high half of the 2-cycle burst
		if(wr_en && !wr_phase) begin
			wr_data_ff	<= wr_data;
			wr_phase	<= 1;
		end
		else if(wr_en) begin
			wr_data_ff	<= { wr_data_ff[RAM_WIDTH*2-1:0], {RAM_WIDTH*2{1'b0}} };
			wr_phase	<= 0;
		end
		else begin
			wr_phase	<= 0;

			//Avoid excess toggling and power consumption by turning both bus halves to the same value
			//during idle periods
			wr_data_ff	<= 0;
		end

		//Pipeline delays
		wr_data_ff2		<= wr_data_ff;
		wr_en_ff		<= wr_en;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// C/A bus output drivers

	//TODO: is this the best way to match things vs using fabric muxes + IOB DFFs in the double rate domain?

	//Select signals are on alternating edges of the controller clock
	DDROutputBuffer #(
		.WIDTH(1),
		.INIT(1)
	) wps_oddr (
		.clk_p(clk_ctl),
		.clk_n(!clk_ctl),
		.dout(qdr_wps_n),
		.din0(!wr_en),
		.din1(1'b1)
	);

	DDROutputBuffer #(
		.WIDTH(1),
		.INIT(1)
	) rps_oddr (
		.clk_p(clk_ctl),
		.clk_n(!clk_ctl),
		.dout(qdr_rps_n),
		.din0(1'b1),
		.din1(!rd_en)
	);

	//Address bus takes turns between the two ports
	DDROutputBuffer #(
		.WIDTH(ADDR_BITS)
	) addr_oddr (
		.clk_p(clk_ctl),
		.clk_n(!clk_ctl),
		.dout(qdr_a),
		.din0(wr_addr),
		.din1(rd_addr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// D bus output drivers

	//Byte write enables are active any time we're writing
	//Note that there's a delay of one DCLK cycle between WPS# assertion and BWS# assertion
	DDROutputBuffer #(
		.WIDTH(4)
	) bws_oddr (
		.clk_p(clk_ctl),
		.clk_n(!clk_ctl),
		.dout(qdr_bws_n),
		.din0({4{!wr_en_ff_ctl}}),
		.din1({4{!wr_en}})
	);

	//Data bus is double-rate
	//Need to rearrange things a bit to correct for phasing of clocks
	DDROutputBuffer #(
		.WIDTH(RAM_WIDTH)
	) data_oddr (
		.clk_p(clk_ram),
		.clk_n(!clk_ram),
		.dout(qdr_d),
		.din0(wr_data_ff2[RAM_WIDTH*3 +: RAM_WIDTH]),
		.din1(wr_data_ff2[RAM_WIDTH*2 +: RAM_WIDTH])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input capture

	wire[1:0]			qvld_capture;

	wire[RAM_WIDTH-1:0]	qcapture_0;
	wire[RAM_WIDTH-1:0]	qcapture_1;

	DDRInputBuffer #(
		.WIDTH(RAM_WIDTH)
	) data_iddr (
		.clk_p(clk_qcapture),
		.clk_n(!clk_qcapture),
		.din(qdr_q),
		.dout0(qcapture_0),
		.dout1(qcapture_1)
	);

	DDRInputBuffer #(
		.WIDTH(1)
	) qvld_iddr (
		.clk_p(clk_qcapture),
		.clk_n(!clk_qcapture),
		.din(qdr_qvld),
		.dout0(qvld_capture[0]),
		.dout1(qvld_capture[1])
	);

	//Pipeline delay for QVLD to match data
	logic				qvld_ff	= 0;
	logic[1:0]			qvld_pipe = 0;

	always_ff @(posedge clk_qcapture) begin
		qvld_ff			<= qvld_capture[1];
	end

	always_comb begin
		qvld_pipe		= { qvld_capture[0], qvld_ff };
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Width expansion for inbound data

	logic[RAM_WIDTH-1:0]	qcapture_0_ff = 0;
	logic[RAM_WIDTH-1:0]	qcapture_1_ff = 0;

	logic[1:0]				qvld_pipe_ff = 0;

	logic[CTRL_WIDTH-1:0] 	qcapture_cdc_in	= 0;
	logic					qcapture_cdc_en	= 0;

	always_ff @(posedge clk_qcapture) begin
		qcapture_0_ff		<= qcapture_0;
		qcapture_1_ff		<= qcapture_1;

		qvld_pipe_ff		<= qvld_pipe;

		qcapture_cdc_in		<= { qcapture_0_ff, qcapture_1_ff, qcapture_0, qcapture_1 };

		//Write to CDC FIFO if we have valid data and didn't write last cycle
		//(since bursts are two clocks long)
		qcapture_cdc_en		<= (qvld_pipe_ff == 2'b11) && (qvld_pipe == 2'b11) && !qcapture_cdc_en;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock domain crossing for inbound data

	logic					fifo_rd_en	= 0;
	wire[5:0]				fifo_rd_size;
	wire					fifo_rd_empty;

	//Use LUTRAM since we're wide but don't need much depth
	//Ignore write-side size signals because we pop basically every cycle so it can't overflow
	CrossClockFifo #(
		.WIDTH(CTRL_WIDTH),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) fifo (
		.wr_clk(clk_qcapture),
		.wr_en(qcapture_cdc_en),
		.wr_data(qcapture_cdc_in),
		.wr_size(),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(1'b0),

		.rd_clk(clk_ctl),
		.rd_en(fifo_rd_en),
		.rd_data(rd_data),
		.rd_size(fifo_rd_size),
		.rd_empty(fifo_rd_empty),
		.rd_underflow(),
		.rd_reset(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pop inbound data out of the FIFO and send to the host

	always_comb begin
		fifo_rd_en = !fifo_rd_empty;
	end

	always_ff @(posedge clk_ctl) begin
		rd_valid	<= fifo_rd_en;
	end

endmodule
