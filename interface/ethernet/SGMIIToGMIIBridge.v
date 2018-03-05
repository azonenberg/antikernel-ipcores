`default_nettype none
`timescale 1ns/1ps

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
	@brief Bridge from SGMII to GMII
 */
module SGMIIToGMIIBridge(

	//SGMII interface (connect directly to top-level pads)
	input wire			sgmii_rx_clk_p,		//625 MHz RX clock
	input wire			sgmii_rx_clk_n,

	input wire			sgmii_rx_data_p,	//1250 Mbps DDR RX data, aligned to sgmii_rx_clk (8b10b coded)
	input wire			sgmii_rx_data_n,

	output wire			gmii_rx_clk,

	output wire[7:0]	debug_led
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Differential I/O buffers

	wire	rx_clk;

	DifferentialInputBuffer #(
		.WIDTH(1),
		.IOSTANDARD("LVDS"),
		.ODT(1),
		.OPTIMIZE("SPEED")
	) ibuf_clk (
		.pad_in_p(sgmii_rx_clk_p),
		.pad_in_n(sgmii_rx_clk_n),
		.fabric_out(rx_clk)
	);

	wire	rx_data_serial;

	DifferentialInputBuffer #(
		.WIDTH(1),
		.IOSTANDARD("LVDS"),
		.ODT(1),
		.OPTIMIZE("SPEED")
	) ibuf_data (
		.pad_in_p(sgmii_rx_data_p),
		.pad_in_n(sgmii_rx_data_n),
		.fabric_out(rx_data_serial)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug toggler

	/*
	ODDR #
	(
		.DDR_CLK_EDGE("SAME_EDGE"),
		.SRTYPE("ASYNC"),
		.INIT(0)
	) ddr_obuf
	(
		.C(clk_250mhz),
		.D1(1'b0),
		.D2(1'b1),
		.CE(1'b1),
		.R(1'b0),
		.S(1'b0),
		.Q(debug_clk)
	);
	*/
	/*
	reg		toggle = 0;
	assign	debug_clk = toggle;

	always @(posedge clk_250mhz) begin
		toggle <= ~toggle;
	end
	*/

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX PLL (locked to the RX clock)

	wire 	pll_locked;

	wire	serdes_clk_raw;
	wire	gmii_rx_clk_raw;

	//TODO: abstraction for this
	wire clkfb;
	MMCME4_BASE #(
		.BANDWIDTH("OPTIMIZED"),

		.CLKIN1_PERIOD(1.600),		//625 MHz Fin
		.DIVCLK_DIVIDE(2),			//312.5 MHz to the PFD
		.CLKFBOUT_MULT_F(4),		//1.25 GHz Fvco
		.CLKFBOUT_PHASE(0),

		.STARTUP_WAIT("FALSE"),		//Don't wait for PLL to lock at startup

		.CLKOUT1_DIVIDE(2),			//625 MHz clock for SERDES
		.CLKOUT2_DIVIDE(5),			//125 MHz clock for GMII subsystem
		.CLKOUT3_DIVIDE(1),
		.CLKOUT4_DIVIDE(1),
		.CLKOUT5_DIVIDE(1),
		.CLKOUT6_DIVIDE(1),

		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5),
		.CLKOUT6_DUTY_CYCLE(0.5),

		.CLKOUT1_PHASE(0),
		.CLKOUT2_PHASE(0),
		.CLKOUT3_PHASE(0),
		.CLKOUT4_PHASE(0),
		.CLKOUT5_PHASE(0),
		.CLKOUT6_PHASE(0)

	) pll (
		.CLKFBIN(clkfb),
		.CLKFBOUT(clkfb),

		.CLKIN1(rx_clk),

		.CLKOUT1(serdes_clk_raw),
		.CLKOUT2(gmii_rx_clk_raw),
		.CLKOUT3(),
		.CLKOUT4(),
		.CLKOUT5(),
		.CLKOUT6(),

		.LOCKED(pll_locked),

		.PWRDWN(0),

		.RST(0)
	);

	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("YES")
	) bufg_gmii_clk (
		.clkin(gmii_rx_clk_raw),
		.clkout(gmii_rx_clk),
		.ce(pll_locked)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LED blinky

	assign	debug_led[7:3] = 0;
	assign	debug_led[2] = count2[21];
	assign	debug_led[1] = pll_locked;
	assign	debug_led[0] = count[21];

	reg[21:0] count = 0;
	reg[21:0] count2 = 0;
	always @(posedge gmii_rx_clk) begin
		count	<= count + 1'h1;
	end
	always @(posedge rx_clk) begin
		count2	<= count2 + 1'h1;
	end


endmodule
