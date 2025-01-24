`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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

module QuadPLL_UltraScale #(

	//Default values are for 25.78125 Gbps from a 156.25 MHz reference
	//TODO: figure out what has to be changed if we use different configs

	parameter QPLL0_MULT	= 82,	//should output 25.625 Gbps, 12.8125 GHz
	parameter QPLL1_MULT	= 66	//should output 20.625 Gbps, 10.3125 GHz
)(
	//Clock inputs
	input wire			clk_lockdet,	//Lock detector clock (shared by QPLL0/1 in this wrapper)
	input wire[1:0]		clk_ref_north,	//Northbound reference clock from quad to our south
	input wire[1:0]		clk_ref_south,	//Southbound reference clock from quad to our north
	input wire[1:0]		clk_ref,		//Reference inputs to our quad

	//Reference clock mux selectors
	input wire[2:0]		qpll0_refclk_sel,
	input wire[2:0]		qpll1_refclk_sel,

	//Clock outputs
	output wire[1:0]	qpll_clkout,
	output wire[1:0]	qpll_refout,

	//APB to DRP
	//DRP space is mapped as 32-bit words with the high 16 bits write-ignored / RAZ
	//Accesses must be aligned
	APB.completer 		apb,

	//Reset inputs
	input wire[1:0]		qpll_reset,
	input wire[1:0]		sdm_reset,
	input wire[1:0]		qpll_powerdown,

	//Status outputs
	output wire[1:0]	fbclk_lost,
	output wire[1:0]	qpll_lock,
	output wire[1:0]	refclk_lost,

	/*
		Fractional-N control inputs

		For U+: must strobe sdm_toggle to load new sdm_data
		Timing requirements:
		* SETUP: 1 system clock from data change to toggle high
		* PULSE LENGTH: 3 FBCLK cycles with toggle high and sdm_data stable
		* HOLD: 3 FBCLK cycles with toggle low before sdm_data can change

		USRCLK or DRP clock can be used as "system" clock
	 */
	input wire[24:0]	sdm_data0,
	input wire[24:0]	sdm_data1,
	input wire[1:0]		sdm_toggle
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB , throw synthesis error for anything else

	if(apb.DATA_WIDTH > 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off extra APB outputs

	assign apb.prdata[31:16]	= 0;
	assign apb.pslverr 			= 0;
	assign apb.pruser			= 0;
	assign apb.pbuser 			= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual common block

	GTYE4_COMMON #(
		.BIAS_CFG0(16'b0000000000000000),
		.BIAS_CFG1(16'b0000000000000000),
		.BIAS_CFG2(16'b0000010100100100),
		.BIAS_CFG3(16'b0000000001000001),
		.BIAS_CFG4(16'b0000000000010000),
		.BIAS_CFG_RSVD(16'b0000000000000000),
		.COMMON_CFG0(16'b0000000000000000),
		.COMMON_CFG1(16'b0000000000000000),
		.POR_CFG(16'b0000000000000000),
		.QPLL0_CFG0(16'b0011001100011100),
		.QPLL0_CFG1(16'b1101000000111000),
		.QPLL1_CFG0(16'b0011001100011100),
		.QPLL1_CFG1(16'b1101000000111000),
		.QPLL0_CFG1_G3(16'b1101000000111000),
		.QPLL1_CFG1_G3(16'b1101000000111000),
		.QPLL0_CFG2(16'b1000011111000011),
		.QPLL1_CFG2(16'b0000111111000011),
		.QPLL0_CFG2_G3(16'b1000011111000011),
		.QPLL1_CFG2_G3(16'b0000111111000011),
		.QPLL0_CFG3(16'b0000000100100000),
		.QPLL1_CFG3(16'b0000000100100000),
		.QPLL0_CFG4(16'b0000000010000100),
		.QPLL1_CFG4(16'b0000000000000010),
		.QPLL0CLKOUT_RATE("FULL"),
		.QPLL1CLKOUT_RATE("FULL"),
		.QPLL0_CP(10'b0011111111),
		.QPLL1_CP(10'b0011111111),
		.QPLL0_CP_G3(10'b0000001111),
		.QPLL1_CP_G3(10'b0001111111),
		.QPLL0_FBDIV(QPLL0_MULT),
		.QPLL0_FBDIV_G3(160),
		.QPLL1_FBDIV(QPLL1_MULT),
		.QPLL1_FBDIV_G3(80),
		.QPLL0_INIT_CFG0(16'b0000001010110010),
		.QPLL1_INIT_CFG0(16'b0000001010110010),
		.QPLL0_INIT_CFG1(8'b00000000),
		.QPLL1_INIT_CFG1(8'b00000000),
		.QPLL0_LOCK_CFG(16'b0010010111101000),
		.QPLL1_LOCK_CFG(16'b0010010111101000),
		.QPLL0_LOCK_CFG_G3(16'b0010010111101000),
		.QPLL1_LOCK_CFG_G3(16'b0010010111101000),
		.QPLL0_LPF(10'b1000011111),
		.QPLL1_LPF(10'b1000011111),
		.QPLL0_LPF_G3(10'b0111010101),
		.QPLL1_LPF_G3(10'b0111010100),
		.QPLL0_REFCLK_DIV(1),	//Refclk divisor 1-4
		.QPLL1_REFCLK_DIV(1),	//Refclk divisor 1-4
		.QPLL0_SDM_CFG0(16'b0000000000000000),
		.QPLL1_SDM_CFG0(16'b0000000010000000),
		.QPLL0_SDM_CFG1(16'b0000000000000000),
		.QPLL1_SDM_CFG1(16'b0000000000000000),
		.QPLL0_SDM_CFG2(16'b0000000000000000),
		.QPLL1_SDM_CFG2(16'b0000000000000000),
		.RSVD_ATTR0(16'b0000000000000000),
		.RSVD_ATTR1(16'b0000000000000000),
		.RSVD_ATTR2(16'b0000000000000000),
		.RSVD_ATTR3(16'b0000000000000000),
		.SDM0INITSEED0_0(16'b0000000100010001),
		.SDM1INITSEED0_0(16'b0000000100010001),
		.SDM0INITSEED0_1(9'b000010001),
		.SDM1INITSEED0_1(9'b000010001),
		.PPF0_CFG(16'b0000100000000000),
		.PPF1_CFG(16'b0000011000000000),
		.QPLL0_PCI_EN(1'b0),
		.QPLL1_PCI_EN(1'b0),
		.QPLL0_RATE_SW_USE_DRP(1'b1)
	) common (

		//Reserved ports, ignore or tie off
		.QPLLDMONITOR0(),
		.QPLLDMONITOR1(),
		.QPLL0CLKRSVD0(1'b0),
		.QPLL1CLKRSVD0(1'b0),
		.QPLL0CLKRSVD1(1'b0),
		.QPLL1CLKRSVD1(1'b0),
		.QPLLRSVD1(8'b00000000),
		.QPLLRSVD2(5'b00000),
		.QPLLRSVD3(5'b00000),
		.QPLLRSVD4(8'b00000000),
		.REFCLKOUTMONITOR0(),
		.REFCLKOUTMONITOR1(),
		.BGBYPASSB(1'b1),
		.BGMONITORENB(1'b1),
		.BGPDB(1'b1),
		.BGRCALOVRD(5'b11111),
		.BGRCALOVRDENB(1'b1),
		.RCALENB(1'b1),
		.PMARSVD0(8'b00000000),
		.PMARSVD1(8'b00000000),
		.QPLL0FBDIV(8'h0),
		.QPLL1FBDIV(8'h0),
		.SDM0FINALOUT(),
		.SDM1FINALOUT(),
		.SDM0TESTDATA(),
		.SDM1TESTDATA(),
		.PMARSVDOUT0(),
		.PMARSVDOUT1(),
		.RXRECCLK0SEL(),
		.RXRECCLK1SEL(),
		.PCIERATEQPLL0(),
		.PCIERATEQPLL1(),

		//Input reference clocks: QPLL0
		.GTNORTHREFCLK00(clk_ref_north[0]),
		.GTNORTHREFCLK10(clk_ref_north[1]),
		.GTREFCLK00(clk_ref[0]),
		.GTREFCLK10(clk_ref[1]),
		.GTSOUTHREFCLK00(clk_ref_south[0]),
		.GTSOUTHREFCLK10(clk_ref_south[1]),
		.GTGREFCLK0(),	//internally generated factory test clock, not used

		//Input reference clocks: QPLL1
		.GTNORTHREFCLK01(clk_ref_north[0]),
		.GTNORTHREFCLK11(clk_ref_north[1]),
		.GTREFCLK01(clk_ref[0]),
		.GTREFCLK11(clk_ref[1]),
		.GTSOUTHREFCLK01(clk_ref_south[0]),
		.GTSOUTHREFCLK11(clk_ref_south[1]),
		.GTGREFCLK1(),	//internally generated factory test clock, not used

		//QPLL0
		.QPLL0FBCLKLOST(fbclk_lost[0]),
		.QPLL0LOCK(qpll_lock[0]),
		.QPLL0LOCKDETCLK(clk_lockdet),
		.QPLL0LOCKEN(1'b1),
		.QPLL0OUTCLK(qpll_clkout[0]),
		.QPLL0OUTREFCLK(qpll_refout[0]),
		.QPLL0PD(qpll_powerdown[0]),
		.QPLL0REFCLKLOST(refclk_lost[0]),
		.QPLL0REFCLKSEL(qpll0_refclk_sel),
		.QPLL0RESET(qpll_reset[0]),
		.SDM0RESET(sdm_reset[0]),
		.SDM0DATA(sdm_data0),
		.SDM0WIDTH(2'b00),	//24 bit denominator
		.SDM0TOGGLE(sdm_toggle[0]),

		//QPLL1
		.QPLL1FBCLKLOST(fbclk_lost[1]),
		.QPLL1LOCK(qpll_lock[1]),
		.QPLL1LOCKDETCLK(clk_lockdet),
		.QPLL1LOCKEN(1'b1),
		.QPLL1OUTCLK(qpll_clkout[1]),
		.QPLL1OUTREFCLK(qpll_refout[1]),
		.QPLL1PD(qpll_powerdown[1]),
		.QPLL1REFCLKLOST(refclk_lost[1]),
		.QPLL1REFCLKSEL(qpll1_refclk_sel),
		.QPLL1RESET(qpll_reset[1]),
		.SDM1RESET(sdm_reset[1]),
		.SDM1DATA(sdm_data1),
		.SDM1WIDTH(2'b00),	//24 bit denominator
		.SDM1TOGGLE(sdm_toggle[1]),

		//DRP
		.DRPADDR({8'h0, apb.paddr[9:2]}),	//DADDR is 16 bit but not entire address space is used
		.DRPCLK(apb.pclk),
		.DRPEN(apb.penable),
		.DRPDI(apb.pwdata[15:0]),
		.DRPRDY(apb.pready),
		.DRPDO(apb.prdata[15:0]),
		.DRPWE(apb.pwrite),

		//Hard MicroBlaze (ignore)
		.UBDO(),
		.UBDRDY(),
		.UBENABLE(),
		.UBGPI(),
		.UBINTR(),
		.UBIOLMBRST(),
		.UBMBRST(),
		.UBMDMCAPTURE(),
		.UBMDMDBGRST(),
		.UBMDMDBGUPDATE(),
		.UBMDMREGEN(),
		.UBMDMSHIFT(),
		.UBMDMSYSRST(),
		.UBMDMTCK(),
		.UBMDMTDI(),
		.UBDADDR(),
		.UBDEN(),
		.UBDI(),
		.UBDWE(),
		.UBMDMTDO(),
		.UBRSVDOUT(),
		.UBTXUART(),
		.UBCFGSTREAMEN()
	);

endmodule
