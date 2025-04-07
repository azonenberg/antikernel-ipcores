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

/**
	@brief Wrapper around the UltraScale+ (and probably with some small changes UltraScale) GTY.
 */
module GTYLane_UltraScale #(
	parameter CPLL_FBDIV 		= 4,
	parameter CPLL_FBDIV_45 	= 4,

	parameter RX_COMMA_ALIGN	= 0,		//Set true to enable the 8b10b comma aligner.
											//If enabled, we look for K28.5 and try to put it in lane 0
	parameter RX_COMMA_ANY_LANE	= 0,		//If set true, allows commas to be in any lane position not just 0
											//(mostly used for QSGMII)

	parameter DATA_WIDTH		= 32,		//same for TX and RX, we don't support mismatched widths for now

	parameter ROUGH_RATE_GBPS	= 10,		//CDR etc settings depend on the approximate data rate
											//supported values for now: 5, 10, 25

	parameter RX_BUF_BYPASS		= 0,		//set 1 to bypass rx buffer

	parameter GEARBOX_CFG		= "OFF"		//legal values: "OFF", "64b66b" so far
) (

	//APB to DRP
	//DRP space is mapped as 32-bit words with the high 16 bits write-ignored / RAZ
	//Accesses must be aligned
	APB.completer 		apb,

	//Top level pins
	input wire			rx_p,
	input wire			rx_n,

	output wire			tx_p,
	output wire			tx_n,

	//Resets
	input wire			rx_reset,
	input wire			tx_reset,

	//Reference clock inputs
	input wire[1:0]		clk_ref_north,	//Northbound reference clock from quad to our south
	input wire[1:0]		clk_ref_south,	//Southbound reference clock from quad to our north
	input wire[1:0]		clk_ref,		//Reference inputs to our quad
	input wire			clk_lockdet,

	//Clock IOs
	input wire			rxusrclk,
	input wire			rxusrclk2,
	input wire			rxuserrdy,

	input wire			txusrclk,
	input wire			txusrclk2,
	input wire			txuserrdy,

	input wire[1:0]		rxpllclksel,
	input wire[1:0]		txpllclksel,

	output wire			rxoutclk,
	output wire			txoutclk,

	//QPLL clocks
	input wire[1:0]		qpll_clk,
	input wire[1:0]		qpll_refclk,
	input wire[1:0]		qpll_lock,

	//CPLL status/control
	input wire			cpll_pd,
	output wire			cpll_fblost,
	output wire			cpll_reflost,
	output wire			cpll_lock,
	input wire[2:0]		cpll_refclk_sel,

	//RX equalizer control
	input wire			rx_ctle_en,

	//Sub-rate configuration
	input wire[2:0]		tx_rate,
	input wire[2:0]		rx_rate,

	//Data bus
	input wire[DATA_WIDTH-1:0]	tx_data,	//TXUSRCLK2
	output wire[DATA_WIDTH-1:0]	rx_data,	//RXUSRCLK2

	//PRBS generator
	input wire[3:0]		rxprbssel,
	input wire[3:0]		txprbssel,
	output wire			rxprbserr,
	output wire			rxprbslocked,

	//TX driver config
	input wire[4:0]		txdiffctrl,
	input wire[4:0]		txpostcursor,
	input wire[4:0]		txprecursor,
	input wire			tx_invert,

	//RX input config
	input wire			rx_invert,

	//Synchronous gearbox
	output wire[1:0]	rx_data_valid,
	output wire			rx_header_valid,
	output wire[5:0]	rx_header,
	input wire			rx_gearbox_bitslip,
	input wire[5:0]		tx_header,
	input wire[6:0]		tx_sequence,

	//8B/10B coder (requires RX_COMMA_ALIGN set to be useful in most cases)
	input wire			rx_8b10b_decode,
	output wire			rx_comma_is_aligned,
	output wire[15:0]	rx_char_is_k,				//TODO only hook up the valid bits
	output wire[15:0]	rx_char_is_comma,
	output wire[15:0]	rx_disparity_err,
	output wire[15:0]	rx_symbol_err,
	output wire			rx_commadet_slip,
	input wire			tx_8b10b_encode,
	input wire[15:0]	tx_char_is_k
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
	// Clock buffers

	wire	rxoutclk_raw;
	wire	txoutclk_raw;

	BUFG_GT bufg_rxoutclk(
		.I(rxoutclk_raw),
		.O(rxoutclk),
		.CE(1'b1),
		.CEMASK(1'b0),
		.CLR(1'b0),
		.CLRMASK(1'b0),
		.DIV(3'b000));

	BUFG_GT bufg_txoutclk(
		.I(txoutclk_raw),
		.O(txoutclk),
		.CE(1'b1),
		.CEMASK(1'b0),
		.CLR(1'b0),
		.CLRMASK(1'b0),
		.DIV(3'b000));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter calculations: Padding and data width

	localparam PADDING_WIDTH = 128 - DATA_WIDTH;

	//return the INT_DATAWIDTH setting to make internal and external widths equal
	function integer int_datawidth(input integer width);
		case(width)
			16: return 0;
			20: return 0;
			32: return 1;
			40: return 1;
			64: return 2;
			80: return 2;
			default: return 0;
		endcase
	endfunction

	localparam INT_WIDTH = int_datawidth(DATA_WIDTH);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter calculations: Comma detection and alignment

	//RX comma align configuration (for now, only tested with 40 bit data width)
	localparam ALIGN_COMMA_ENABLE = RX_COMMA_ALIGN ? 10'b0001111111 : 10'b0000000000;
	localparam ALIGN_MCOMMA_DET = RX_COMMA_ALIGN : "TRUE" : "FALSE";
	localparam ALIGN_PCOMMA_DET = RX_COMMA_ALIGN : "TRUE" : "FALSE";

	//return the ALIGN_COMMA_WORD setting to put all commas in the lane 0 position (if RX_COMMA_ANY_LANE not set)
	//or constant 1 if RX_COMMA_ANY_LANE is set
	function integer comma_word(input integer width);

		if(RX_COMMA_ANY_LANE)
			return 1;
		else begin
			case(width)
				16: return 2;
				20: return 2;
				32: return 4;
				40: return 4;
				default: return 1;
			endcase
		end

	endfunction

	localparam ALIGN_COMMA_WORD = RX_COMMA_ALIGN ? comma_word(DATA_WIDTH) : 1;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter calculations: TX/RX unknown PHY

	//Return the PREIQ_FREQ_BST setting for the given data rate
	function integer preiq_freq_bst(input integer gbps);
		if(gbps < 10)
			return 0;
		else if(gbps < 20)
			return 1;
		else if(gbps < 25)
			return 2;
		else
			return 3;
	endfunction
	localparam PREIQ_FREQ_BST = preiq_freq_bst(ROUGH_RATE_GBPS);

	//TX driver
	function integer txdrv_freqband(input integer gbps);
		if(gbps < 11)
			return 0;
		else if(gbps < 21)
			return 1;
		else
			return 3;
	endfunction
	localparam TXDRV_FREQBAND = txdrv_freqband(ROUGH_RATE_GBPS);

	localparam RX_XMODE_SEL = ROUGH_RATE_GBPS <= 10 ? 1'b1 : 1'b0;

	//no idea what this does or what affects it, this is just a guesstimate
	function integer ch_hspmux(input integer gbps);
		if(gbps < 1)
			return 16'b0010000000100000;
		else if(gbps < 5)
			return 16'b0100000001000000;
		else if(gbps == 5)
			return 16'b0010000000100000;
		else if(gbps == 7)
			return 16'b0100000001000000;
		else if(gbps == 10)
			return 16'b0010000000100000;
		else if(gbps == 15)
			return 16'b0100000001000000;
		else return 16'b0110000001100000;
	endfunction
	localparam CH_HSPMUX = ch_hspmux(ROUGH_RATE_GBPS);	//16'b1001000010010000

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter calculations: RX buffer bypass

	//RX buffer bypass
	localparam RXBUF_EN = RX_BUF_BYPASS ? "FALSE" : "TRUE";
	localparam RXBUF_THRESH_OVFLW = RX_BUF_BYPASS ? 0 : 49;
	localparam RXBUF_THRESH_OVRD = RX_BUF_BYPASS ? "FALSE" : "TRUE";
	localparam RXBUF_THRESH_UNDFLW = RX_BUF_BYPASS ? 4 : 7;
	localparam RX_XCLK_SEL = RX_BUF_BYPASS : "RXUSR" : "RXDES";

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter calculations: Gearbox

	localparam RXGEARBOX_EN = (GEARBOX_CFG == "OFF") ? "FALSE" : "TRUE";
	localparam TXGEARBOX_EN = (GEARBOX_CFG == "OFF") ? "FALSE" : "TRUE";
	localparam GEARBOX_MODE = (GEARBOX_CFG == "OFF") ? 5'b00000 : 5'b00001;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parameter calculations: RX equalizers

	function[15:0] adapt_cfg1(input integer gbps);
		if(gbps < 10)
			return 16'b1111100000011100;
		else
			return 16'b1111101100011100;
	endfunction

	//not well understood
	function[15:0] rxcdr_cfg2(input integer gbps);
		if(gbps == 10)
			return 16'b0000001001101001;
		else if(gbps == 5)
			return 16'b0000001001011001;
		else
			return 16'b0000001000111001;
	endfunction

	function[15:0] rxcdr_cfg3(input integer gbps);
		if(gbps <= 20)
			return 16'b0000000000010010;
		else
			return 16'b0000000000010000;
	endfunction

	function[15:0] rxckcal1_loop_rst_cfg(input integer gbps);
		if(gbps < 20)
			return 16'b0000000000000000;
		else
			return 16'b0000000000000100;
	endfunction

	localparam ADAPT_CFG1 = adapt_cfg1(ROUGH_RATE_GBPS);
	localparam RXCDR_CFG2 = rxcdr_cfg2(ROUGH_RATE_GBPS);
	localparam RXCDR_CFG3 = rxcdr_cfg3(ROUGH_RATE_GBPS);
	localparam RXCKCAL1_IQ_LOOP_RST_CFG = rxckcal1_loop_rst_cfg(ROUGH_RATE_GBPS);
	localparam RXCKCAL1_I_LOOP_RST_CFG = rxckcal1_loop_rst_cfg(ROUGH_RATE_GBPS);
	localparam RXCKCAL1_Q_LOOP_RST_CFG = rxckcal1_loop_rst_cfg(ROUGH_RATE_GBPS);
	localparam RXCKCAL2_DX_LOOP_RST_CFG = rxckcal1_loop_rst_cfg(ROUGH_RATE_GBPS);
	localparam RXCKCAL2_D_LOOP_RST_CFG = rxckcal1_loop_rst_cfg(ROUGH_RATE_GBPS);
	localparam RXCKCAL2_S_LOOP_RST_CFG = rxckcal1_loop_rst_cfg(ROUGH_RATE_GBPS);
	localparam RXCKCAL2_X_LOOP_RST_CFG = rxckcal1_loop_rst_cfg(ROUGH_RATE_GBPS);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Resets

	wire	rst_tx_sync;
	wire	rst_rx_sync;

	GTYLane_ResetController_UltraScale rst_ctl(
		.clk_system_in(clk_lockdet),

		.rst_async_tx_in(tx_reset),
		.rst_async_rx_in(rx_reset),

		.rst_gt_tx_out(rst_tx_sync),
		.rst_gt_rx_out(rst_rx_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual channel

	wire[127:0] tx_data_padded;
	assign tx_data_padded[DATA_WIDTH-1:0] = tx_data;
	if(DATA_WIDTH != 128)
		assign tx_data_padded[127:DATA_WIDTH] = {PADDING_WIDTH{1'b0}};

	wire[127:0] rx_data_padded;
	assign rx_data = rx_data_padded[DATA_WIDTH-1:0];

	wire txuserrdy_gated;
	assign txuserrdy_gated = txuserrdy && !rst_tx_sync;

	wire rxuserrdy_gated;
	assign rxuserrdy_gated = rxuserrdy && !rst_rx_sync;

	//unless otherwise specified all configs here were generated for 25.78125 Gbps
	GTYE4_CHANNEL #(

		//RX datapath config
		.RX_DATA_WIDTH(DATA_WIDTH),
		.TX_DATA_WIDTH(DATA_WIDTH),
		.RX_INT_DATAWIDTH(INT_WIDTH),
		.TX_INT_DATAWIDTH(INT_WIDTH),

		//CPLL config
		//Values shown here are for 5 Gbps from a 156.25 MHz refclk and need to be tuned
		.CPLL_CFG0(16'b0000000111111010),
		.CPLL_CFG1(16'b0000000000101011),
		.CPLL_CFG2(16'b0000000000000010),
		.CPLL_CFG3(16'b0000000000000000),
		.CPLL_FBDIV(CPLL_FBDIV),
		.CPLL_FBDIV_45(CPLL_FBDIV_45),
		.CPLL_INIT_CFG0(16'b0000001010110010),
		.CPLL_LOCK_CFG(16'b0000000111101000),
		.CPLL_REFCLK_DIV(1),

		//Channel bonding TODO
		.CHAN_BOND_KEEP_ALIGN("FALSE"),
		.CHAN_BOND_MAX_SKEW(1),
		.CHAN_BOND_SEQ_1_1(10'b0000000000),
		.CHAN_BOND_SEQ_1_2(10'b0000000000),
		.CHAN_BOND_SEQ_1_3(10'b0000000000),
		.CHAN_BOND_SEQ_1_4(10'b0000000000),
		.CHAN_BOND_SEQ_1_ENABLE(4'b1111),
		.CHAN_BOND_SEQ_2_1(10'b0000000000),
		.CHAN_BOND_SEQ_2_2(10'b0000000000),
		.CHAN_BOND_SEQ_2_3(10'b0000000000),
		.CHAN_BOND_SEQ_2_4(10'b0000000000),
		.CHAN_BOND_SEQ_2_ENABLE(4'b1111),
		.CHAN_BOND_SEQ_2_USE("FALSE"),
		.CHAN_BOND_SEQ_LEN(1),

		//Elastic buffer clock correction TODO
		.CLK_CORRECT_USE("FALSE"),
		.CLK_COR_KEEP_IDLE("FALSE"),
		.CLK_COR_MAX_LAT(24),
		.CLK_COR_MIN_LAT(16),
		.CLK_COR_PRECEDENCE("TRUE"),
		.CLK_COR_REPEAT_WAIT(0),
		.CLK_COR_SEQ_1_1(10'b0000000000),
		.CLK_COR_SEQ_1_2(10'b0000000000),
		.CLK_COR_SEQ_1_3(10'b0000000000),
		.CLK_COR_SEQ_1_4(10'b0000000000),
		.CLK_COR_SEQ_1_ENABLE(4'b1111),
		.CLK_COR_SEQ_2_1(10'b0000000000),
		.CLK_COR_SEQ_2_2(10'b0000000000),
		.CLK_COR_SEQ_2_3(10'b0000000000),
		.CLK_COR_SEQ_2_4(10'b0000000000),
		.CLK_COR_SEQ_2_ENABLE(4'b1111),
		.CLK_COR_SEQ_2_USE("FALSE"),
		.CLK_COR_SEQ_LEN(1),

		//RX CDR config
		//Values shown here are for 5 Gbps from a 156.25 MHz refclk
		.RXCDRFREQRESET_TIME(5'b00001),
		.RXCDRPHRESET_TIME(5'b00001),
		.RXCDR_CFG0(16'b0000000000000011),
		.RXCDR_CFG0_GEN3(16'b0000000000000011),
		.RXCDR_CFG1(16'b0000000000000000),
		.RXCDR_CFG1_GEN3(16'b0000000000000000),
		.RXCDR_CFG2(RXCDR_CFG2),
		.RXCDR_CFG2_GEN2(10'b1001101001),
		.RXCDR_CFG2_GEN3(16'b0000001001101001),
		.RXCDR_CFG2_GEN4(16'b0000000101100100),
		.RXCDR_CFG3(RXCDR_CFG3),
		.RXCDR_CFG3_GEN2(6'b010000),
		.RXCDR_CFG3_GEN3(16'b0000000000010000),
		.RXCDR_CFG3_GEN4(16'b0000000000010000),
		.RXCDR_CFG4(16'b0101110011110110),	//no change for 5G vs 25G
		.RXCDR_CFG4_GEN3(16'b0101110011110110),
		.RXCDR_CFG5(16'b1011010001101011),	//no change for 5G vs 25G
		.RXCDR_CFG5_GEN3(16'b0001010001101011),
		.RXCDR_FR_RESET_ON_EIDLE(1'b0),
		.RXCDR_HOLD_DURING_EIDLE(1'b0),
		.RXCDR_LOCK_CFG0(16'b0010001000000001),
		.RXCDR_LOCK_CFG1(16'b1001111111111111),
		.RXCDR_LOCK_CFG2(15'b000000000000000),
		.RXCDR_LOCK_CFG3(16'b0000000000000000),
		.RXCDR_LOCK_CFG4(16'b0000000000000000),
		.RXCDR_PH_RESET_ON_EIDLE(1'b0),
		.RXCKCAL1_IQ_LOOP_RST_CFG(RXCKCAL1_IQ_LOOP_RST_CFG),
		.RXCKCAL1_I_LOOP_RST_CFG(RXCKCAL1_I_LOOP_RST_CFG),
		.RXCKCAL1_Q_LOOP_RST_CFG(RXCKCAL1_Q_LOOP_RST_CFG),
		.RXCKCAL2_DX_LOOP_RST_CFG(RXCKCAL2_DX_LOOP_RST_CFG),
		.RXCKCAL2_D_LOOP_RST_CFG(RXCKCAL2_D_LOOP_RST_CFG),
		.RXCKCAL2_S_LOOP_RST_CFG(RXCKCAL2_S_LOOP_RST_CFG),
		.RXCKCAL2_X_LOOP_RST_CFG(RXCKCAL2_X_LOOP_RST_CFG),
		.RX_WIDEMODE_CDR(2'b01),		//TODO: 2'b01 up to 15 Gbps, 2'b10 for 20 and up
		.RX_WIDEMODE_CDR_GEN3(2'b00),
		.RX_WIDEMODE_CDR_GEN4(2'b01),
		.CDR_SWAP_MODE_EN(1'b0),

		//Data rate dependent stuff for TX and RX
		.PREIQ_FREQ_BST(PREIQ_FREQ_BST),
		.TXDRV_FREQBAND(3),

		//RX equalizer config shared by both DFE and CTLE
		.RXDFELPMRESET_TIME(7'b0001111),
		.RXDFELPM_KL_CFG0(15'b000000000000000),
		.RXDFELPM_KL_CFG1(16'b1010000010000010),
		.RXDFELPM_KL_CFG2(16'b0000000100000000),

		//RX CTLE config
		.ADAPT_CFG0(16'b0000000000000000),
		.ADAPT_CFG1(ADAPT_CFG1),
		.ADAPT_CFG2(16'b0000000000000000),
		.RXLPM_CFG(16'b0000000000000000),
		.RXLPM_GC_CFG(16'b1111100000000000),
		.RXLPM_KH_CFG0(16'b0000000000000000),
		.RXLPM_KH_CFG1(16'b1010000000000010),
		.RXLPM_OS_CFG0(16'b0000000000000000),
		.RXLPM_OS_CFG1(16'b1000000000000010),
		.RX_CTLE_PWR_SAVING(1'b0),
		.RX_CTLE_RES_CTRL(4'b0000),

		//RX DFE config
		.RXDFE_CFG0(16'b0000101000000000),
		.RXDFE_CFG1(16'b0000000000000000),
		.RXDFE_GC_CFG0(16'b0000000000000000),
		.RXDFE_GC_CFG1(16'b1000000000000000),
		.RXDFE_GC_CFG2(16'b1111111111100000),
		.RXDFE_H2_CFG0(16'b0000000000000000),
		.RXDFE_H2_CFG1(16'b0000000000000010),
		.RXDFE_H3_CFG0(16'b0000000000000000),
		.RXDFE_H3_CFG1(16'b1000000000000010),
		.RXDFE_H4_CFG0(16'b0000000000000000),
		.RXDFE_H4_CFG1(16'b1000000000000010),
		.RXDFE_H5_CFG0(16'b0000000000000000),
		.RXDFE_H5_CFG1(16'b1000000000000010),
		.RXDFE_H6_CFG0(16'b0000000000000000),
		.RXDFE_H6_CFG1(16'b1000000000000010),
		.RXDFE_H7_CFG0(16'b0000000000000000),
		.RXDFE_H7_CFG1(16'b1000000000000010),
		.RXDFE_H8_CFG0(16'b0000000000000000),
		.RXDFE_H8_CFG1(16'b1000000000000010),
		.RXDFE_H9_CFG0(16'b0000000000000000),
		.RXDFE_H9_CFG1(16'b1000000000000010),
		.RXDFE_HA_CFG0(16'b0000000000000000),
		.RXDFE_HA_CFG1(16'b1000000000000010),
		.RXDFE_HB_CFG0(16'b0000000000000000),
		.RXDFE_HB_CFG1(16'b1000000000000010),
		.RXDFE_HC_CFG0(16'b0000000000000000),
		.RXDFE_HC_CFG1(16'b1000000000000010),
		.RXDFE_HD_CFG0(16'b0000000000000000),
		.RXDFE_HD_CFG1(16'b1000000000000010),
		.RXDFE_HE_CFG0(16'b0000000000000000),
		.RXDFE_HE_CFG1(16'b1000000000000010),
		.RXDFE_HF_CFG0(16'b0000000000000000),
		.RXDFE_HF_CFG1(16'b1000000000000010),
		.RXDFE_KH_CFG0(16'b1000000000000000),
		.RXDFE_KH_CFG1(16'b1111111000000000),
		.RXDFE_KH_CFG2(16'b0010100000011100),
		.RXDFE_KH_CFG3(16'b0100000100100000),
		.RXDFE_OS_CFG0(16'b0010000000000000),
		.RXDFE_OS_CFG1(16'b1000000000000000),
		.RXDFE_UT_CFG0(16'b0000000000000000),
		.RXDFE_UT_CFG1(16'b0000000000000011),
		.RXDFE_UT_CFG2(16'b0000000000000000),
		.RXDFE_VP_CFG0(16'b0000000000000000),
		.RXDFE_VP_CFG1(16'b0000000000110011),

		//RX buffer config
		.RXBUFRESET_TIME(5'b00011),
		.RXBUF_ADDR_MODE("FAST"),
		.RXBUF_EIDLE_HI_CNT(4'b1000),
		.RXBUF_EIDLE_LO_CNT(4'b0000),
		.RXBUF_EN(RXBUF_EN),
		.RXBUF_RESET_ON_CB_CHANGE("TRUE"),
		.RXBUF_RESET_ON_COMMAALIGN("FALSE"),
		.RXBUF_RESET_ON_EIDLE("FALSE"),
		.RXBUF_RESET_ON_RATE_CHANGE("TRUE"),
		.RXBUF_THRESH_OVFLW(RXBUF_THRESH_OVFLW),
		.RXBUF_THRESH_OVRD(RXBUF_THRESH_OVRD),
		.RXBUF_THRESH_UNDFLW(RXBUF_THRESH_UNDFLW),

		//RX comma aligner
		.ALIGN_COMMA_DOUBLE("FALSE"),
		.ALIGN_COMMA_ENABLE(ALIGN_COMMA_ENABLE),
		.ALIGN_COMMA_WORD(ALIGN_COMMA_WORD),
		.ALIGN_MCOMMA_DET(ALIGN_MCOMMA_DET),
		.ALIGN_MCOMMA_VALUE(10'b1010000011),
		.ALIGN_PCOMMA_DET(ALIGN_PCOMMA_DET),
		.ALIGN_PCOMMA_VALUE(10'b0101111100),
		.SHOW_REALIGN_COMMA("TRUE"),
		.RXSLIDE_AUTO_WAIT(7),
		.RXSLIDE_MODE("OFF"),

		//Gearbox
		.GEARBOX_MODE(GEARBOX_MODE),
		.RXGEARBOX_EN(RXGEARBOX_EN),
		.TXGEARBOX_EN(TXGEARBOX_EN),
		.TXGBOX_FIFO_INIT_RD_ADDR(4),
		.RXGBOX_FIFO_INIT_RD_ADDR(3),

		//Clock configuration
		.RXOUT_DIV(1),
		.RX_CLK25_DIV(7),
		.RX_XCLK_SEL(RX_XCLK_SEL),
		.RXPMACLK_SEL("DATA"),
		.RX_XMODE_SEL(RX_XMODE_SEL),

		.TXOUT_DIV(1),
		.TX_CLK25_DIV(7),
		.TX_XCLK_SEL("TXOUT"),
		.TXREFCLKDIV2_SEL(1'b0),

		//Eye scan (default values but normally overwritten by DRP)
		.ES_CLK_PHASE_SEL(1'b0),
		.ES_CONTROL(6'b000000),
		.ES_ERRDET_EN("FALSE"),
		.ES_EYE_SCAN_EN("FALSE"),
		.ES_HORZ_OFFSET(12'b000000000000),
		.ES_PRESCALE(5'b00000),
		.ES_QUALIFIER0(16'b0000000000000000),
		.ES_QUALIFIER1(16'b0000000000000000),
		.ES_QUALIFIER2(16'b0000000000000000),
		.ES_QUALIFIER3(16'b0000000000000000),
		.ES_QUALIFIER4(16'b0000000000000000),
		.ES_QUALIFIER5(16'b0000000000000000),
		.ES_QUALIFIER6(16'b0000000000000000),
		.ES_QUALIFIER7(16'b0000000000000000),
		.ES_QUALIFIER8(16'b0000000000000000),
		.ES_QUALIFIER9(16'b0000000000000000),
		.ES_QUAL_MASK0(16'b0000000000000000),
		.ES_QUAL_MASK1(16'b0000000000000000),
		.ES_QUAL_MASK2(16'b0000000000000000),
		.ES_QUAL_MASK3(16'b0000000000000000),
		.ES_QUAL_MASK4(16'b0000000000000000),
		.ES_QUAL_MASK5(16'b0000000000000000),
		.ES_QUAL_MASK6(16'b0000000000000000),
		.ES_QUAL_MASK7(16'b0000000000000000),
		.ES_QUAL_MASK8(16'b0000000000000000),
		.ES_QUAL_MASK9(16'b0000000000000000),
		.ES_SDATA_MASK0(16'b0000000000000000),
		.ES_SDATA_MASK1(16'b0000000000000000),
		.ES_SDATA_MASK2(16'b0000000000000000),
		.ES_SDATA_MASK3(16'b0000000000000000),
		.ES_SDATA_MASK4(16'b0000000000000000),
		.ES_SDATA_MASK5(16'b0000000000000000),
		.ES_SDATA_MASK6(16'b0000000000000000),
		.ES_SDATA_MASK7(16'b0000000000000000),
		.ES_SDATA_MASK8(16'b0000000000000000),
		.ES_SDATA_MASK9(16'b0000000000000000),
		.EYESCAN_VP_RANGE(0),
		.EYE_SCAN_SWAP_EN(1'b0),

		//SATA/SAS features TODO

		//PCIe features TODO
		.PCI3_AUTO_REALIGN("OVR_1K_BLK"),
		.PCI3_PIPE_RX_ELECIDLE(1'b0),
		.PCI3_RX_ASYNC_EBUF_BYPASS(2'b00),
		.PCI3_RX_ELECIDLE_EI2_ENABLE(1'b0),
		.PCI3_RX_ELECIDLE_H2L_COUNT(6'b000000),
		.PCI3_RX_ELECIDLE_H2L_DISABLE(3'b000),
		.PCI3_RX_ELECIDLE_HI_COUNT(6'b000000),
		.PCI3_RX_ELECIDLE_LP4_DISABLE(1'b0),
		.PCI3_RX_FIFO_DISABLE(1'b0),
		.PCIE3_CLK_COR_EMPTY_THRSH(5'b00000),
		.PCIE3_CLK_COR_FULL_THRSH(6'b010000),
		.PCIE3_CLK_COR_MAX_LAT(5'b00100),
		.PCIE3_CLK_COR_MIN_LAT(5'b00000),
		.PCIE3_CLK_COR_THRSH_TIMER(6'b001000),
		.PCIE_64B_DYN_CLKSW_DIS("FALSE"),
		.PCIE_BUFG_DIV_CTRL(16'b0011010100000000),
		.PCIE_GEN4_64BIT_INT_EN("FALSE"),
		.PCIE_PLL_SEL_MODE_GEN12(2'b10),
		.PCIE_PLL_SEL_MODE_GEN3(2'b10),
		.PCIE_PLL_SEL_MODE_GEN4(2'b10),
		.PCIE_RXPCS_CFG_GEN3(16'b0000101010100101),
		.PCIE_RXPMA_CFG(16'b0010100000001010),
		.PCIE_TXPCS_CFG_GEN3(16'b0010010010100100),
		.PCIE_TXPMA_CFG(16'b0010100000001010),

		//USB3 features TODO
		.USB_BOTH_BURST_IDLE(1'b0),
		.USB_BURSTMAX_U3WAKE(7'b1111111),
		.USB_BURSTMIN_U3WAKE(7'b1100011),
		.USB_CLK_COR_EQ_EN(1'b0),
		.USB_EXT_CNTL(1'b1),
		.USB_IDLEMAX_POLLING(10'b1010111011),
		.USB_IDLEMIN_POLLING(10'b0100101011),
		.USB_LFPSPING_BURST(9'b000000101),
		.USB_LFPSPOLLING_BURST(9'b000110001),
		.USB_LFPSPOLLING_IDLE_MS(9'b000000100),
		.USB_LFPSU1EXIT_BURST(9'b000011101),
		.USB_LFPSU2LPEXIT_BURST_MS(9'b001100011),
		.USB_LFPSU3WAKE_BURST_MS(9'b111110011),
		.USB_LFPS_TPERIOD(4'b0011),
		.USB_LFPS_TPERIOD_ACCURATE(1'b1),
		.USB_MODE(1'b0),
		.USB_PCIE_ERR_REP_DIS(1'b0),
		.USB_PING_SATA_MAX_INIT(21),
		.USB_PING_SATA_MIN_INIT(12),
		.USB_POLL_SATA_MAX_BURST(8),
		.USB_POLL_SATA_MIN_BURST(4),
		.USB_RAW_ELEC(1'b0),
		.USB_RXIDLE_P0_CTRL(1'b1),
		.USB_TXIDLE_TUNE_ENABLE(1'b1),
		.USB_U1_SATA_MAX_WAKE(7),
		.USB_U1_SATA_MIN_WAKE(4),
		.USB_U2_SAS_MAX_COM(64),
		.USB_U2_SAS_MIN_COM(36),

		//TODO
		.ACJTAG_DEBUG_MODE(1'b0),
		.ACJTAG_MODE(1'b0),
		.ACJTAG_RESET(1'b0),
		.A_RXOSCALRESET(1'b0),
		.A_RXPROGDIVRESET(1'b0),
		.A_RXTERMINATION(1'b1),
		.A_TXDIFFCTRL(5'b01100),
		.A_TXPROGDIVRESET(1'b0),
		.CBCC_DATA_SOURCE_SEL("ENCODED"),
		.CFOK_PWRSVE_EN(1'b1),
		.CH_HSPMUX(CH_HSPMUX),
		.CKCAL1_CFG_0(16'b1100000011000000),
		.CKCAL1_CFG_1(16'b0001000011000000),
		.CKCAL1_CFG_2(16'b0010000000001000),
		.CKCAL1_CFG_3(16'b0000000000000000),
		.CKCAL2_CFG_0(16'b1100000011000000),
		.CKCAL2_CFG_1(16'b1000000011000000),
		.CKCAL2_CFG_2(16'b0001000000000000),
		.CKCAL2_CFG_3(16'b0000000000000000),
		.CKCAL2_CFG_4(16'b0000000000000000),
		.CTLE3_OCAP_EXT_CTRL(3'b000),
		.CTLE3_OCAP_EXT_EN(1'b0),
		.DDI_CTRL(2'b00),
		.DDI_REALIGN_WAIT(15),
		.DEC_MCOMMA_DETECT("FALSE"),
		.DEC_PCOMMA_DETECT("FALSE"),
		.DEC_VALID_COMMA_ONLY("FALSE"),
		.DELAY_ELEC(1'b0),
		.DMONITOR_CFG0(10'b0000000000),
		.DMONITOR_CFG1(8'b00000000),
		.FTS_DESKEW_SEQ_ENABLE(4'b1111),
		.FTS_LANE_DESKEW_CFG(4'b1111),
		.FTS_LANE_DESKEW_EN("FALSE"),
		.ISCAN_CK_PH_SEL2(1'b0),
		.LOCAL_MASTER(1'b1),
		.LPBK_BIAS_CTRL(4),
		.LPBK_EN_RCAL_B(1'b0),
		.LPBK_EXT_RCAL(4'b1000),
		.LPBK_IND_CTRL0(5),
		.LPBK_IND_CTRL1(5),
		.LPBK_IND_CTRL2(5),
		.LPBK_RG_CTRL(2),
		.OOBDIVCTL(2'b00),
		.OOB_PWRUP(1'b0),
		.PCS_PCIE_EN("FALSE"),
		.PCS_RSVD0(16'b0000000000000000),
		.PD_TRANS_TIME_FROM_P2(12'b000000111100),
		.PD_TRANS_TIME_NONE_P2(8'b00011001),
		.PD_TRANS_TIME_TO_P2(8'b01100100),
		.RATE_SW_USE_DRP(1'b1),
		.RCLK_SIPO_DLY_ENB(1'b0),
		.RCLK_SIPO_INV_EN(1'b0),
		.RTX_BUF_CML_CTRL(3'b011),
		.RTX_BUF_TERM_CTRL(2'b00),
		.RXCFOK_CFG0(16'b0000000000000000),
		.RXCFOK_CFG1(16'b1000000000010101),
		.RXCFOK_CFG2(16'b0000001010101110),
		.RXDLY_CFG(16'b0000000000010000),
		.RXDLY_LCFG(16'b0000000000110000),
		.RXELECIDLE_CFG("SIGCFG_4"),
		.RXISCANRESET_TIME(5'b00001),
		.RXOOB_CFG(9'b000000110),
		.RXOOB_CLK_CFG("PMA"),
		.RXOSCALRESET_TIME(5'b00011),
		.RXPCSRESET_TIME(5'b00011),
		.RXPHBEACON_CFG(16'b0000000000000000),
		.RXPHDLY_CFG(16'b0010000001110000),
		.RXPHSAMP_CFG(16'b0010000100000000),
		.RXPHSLIP_CFG(16'b1001100100110011),
		.RXPH_MONITOR_SEL(5'b00000),
		.RXPI_CFG0(16'b0011000000000110),
		.RXPI_CFG1(16'b0000000001010100),
		.RXPMARESET_TIME(5'b00011),
		.RXPRBS_ERR_LOOPBACK(1'b0),
		.RXPRBS_LINKACQ_CNT(15),
		.RXREFCLKDIV2_SEL(1'b0),
		.RXSYNC_MULTILANE(1'b0),
		.RXSYNC_OVRD(1'b0),
		.RXSYNC_SKIP_DA(1'b0),
		.RX_AFE_CM_EN(1'b0),
		.RX_BIAS_CFG0(16'b0001001010110000),
		.RX_BUFFER_CFG(6'b000000),
		.RX_CAPFF_SARC_ENB(1'b0),
		.RX_CLKMUX_EN(1'b1),
		.RX_CLK_SLIP_OVRD(5'b00000),
		.RX_CM_BUF_CFG(4'b1010),
		.RX_CM_BUF_PD(1'b0),
		.RX_CM_SEL(3),
		.RX_CM_TRIM(10),
		.RX_DDI_SEL(6'b000000),
		.RX_DEFER_RESET_BUF_EN("TRUE"),
		.RX_DEGEN_CTRL(3'b111),
		.RX_DFELPM_CFG0(10),
		.RX_DFELPM_CFG1(1'b1),
		.RX_DFELPM_KLKH_AGC_STUP_EN(1'b1),
		.RX_DFE_AGC_CFG1(4),
		.RX_DFE_KL_LPM_KH_CFG0(3),
		.RX_DFE_KL_LPM_KH_CFG1(2),
		.RX_DFE_KL_LPM_KL_CFG0(2'b11),
		.RX_DFE_KL_LPM_KL_CFG1(2),
		.RX_DFE_LPM_HOLD_DURING_EIDLE(1'b0),
		.RX_DISPERR_SEQ_MATCH("TRUE"),
		.RX_DIVRESET_TIME(5'b00001),
		.RX_EN_CTLE_RCAL_B(1'b0),
		.RX_EN_SUM_RCAL_B(0),
		.RX_EYESCAN_VS_CODE(7'b0000000),
		.RX_EYESCAN_VS_NEG_DIR(1'b0),
		.RX_EYESCAN_VS_RANGE(2'b10),
		.RX_EYESCAN_VS_UT_SIGN(1'b0),
		.RX_FABINT_USRCLK_FLOP(1'b0),
		.RX_I2V_FILTER_EN(1'b1),
		.RX_PMA_POWER_SAVE(1'b0),
		.RX_PMA_RSV0(16'b0000000000101111),
		.RX_PROGDIV_CFG(0.0),
		.RX_PROGDIV_RATE(16'b0000000000000001),
		.RX_RESLOAD_CTRL(4'b0000),
		.RX_RESLOAD_OVRD(1'b0),
		.RX_SAMPLE_PERIOD(3'b111),
		.RX_SIG_VALID_DLY(11),
		.RX_SUM_DEGEN_AVTT_OVERITE(1),
		.RX_SUM_DFETAPREP_EN(1'b0),
		.RX_SUM_IREF_TUNE(4'b0000),
		.RX_SUM_PWR_SAVING(0),
		.RX_SUM_RES_CTRL(4'b00000),
		.RX_SUM_VCMTUNE(4'b1001),
		.RX_SUM_VCM_BIAS_TUNE_EN(1'b1),
		.RX_SUM_VCM_OVWR(1'b0),
		.RX_SUM_VREF_TUNE(3'b100),
		.RX_TUNE_AFE_OS(2'b10),
		.RX_VREG_CTRL(3'b010),
		.RX_VREG_PDB(1'b1),
		.SAMPLE_CLK_PHASE(1'b0),
		.SAS_12G_MODE(1'b0),
		.SATA_BURST_SEQ_LEN(4'b1111),
		.SATA_BURST_VAL(3'b100),
		.SATA_CPLL_CFG("VCO_3000MHZ"),
		.SATA_EIDLE_VAL(3'b100),
		.SIM_MODE("FAST"),
		.SIM_RECEIVER_DETECT_PASS("TRUE"),
		.SIM_RESET_SPEEDUP("TRUE"),
		.SIM_TX_EIDLE_DRIVE_LEVEL("Z"),
		.SIM_DEVICE("ULTRASCALE_PLUS"),
		.SRSTMODE(1'b0),
		.TAPDLY_SET_TX(2'b00),
		.TERM_RCAL_CFG(15'b100001000000010),
		.TERM_RCAL_OVRD(3'b001),
		.TRANS_TIME_RATE(8'b00001110),
		.TST_RSV0(8'b00000000),
		.TST_RSV1(8'b00000000),
		.TXBUF_EN("TRUE"),
		.TXBUF_RESET_ON_RATE_CHANGE("TRUE"),
		.TXDLY_CFG(16'b1000000000010000),
		.TXDLY_LCFG(16'b0000000000110000),
		.TXFE_CFG0(16'b0000001111000110),
		.TXFE_CFG1(16'b1111100000000000),
		.TXFE_CFG2(16'b1111100000000000),
		.TXFE_CFG3(16'b1111100000000000),
		.TXFIFO_ADDR_CFG("LOW"),
		.TXPCSRESET_TIME(5'b00011),
		.TXPHDLY_CFG0(16'b0110000001110000),
		.TXPHDLY_CFG1(16'b0000000000001110),
		.TXPH_CFG(16'b0000011100100011),
		.TXPH_CFG2(16'b0000000000000000),
		.TXPH_MONITOR_SEL(5'b00000),
		.TXPI_CFG0(16'b0011000000000000),
		.TXPI_CFG1(16'b0000000000000000),
		.TXPI_GRAY_SEL(1'b0),
		.TXPI_INVSTROBE_SEL(1'b0),
		.TXPI_PPM(1'b0),
		.TXPI_PPM_CFG(8'b000000000),
		.TXPI_SYNFREQ_PPM(3'b001),
		.TXPMARESET_TIME(5'b00011),
		.TXSWBST_BST(1),
		.TXSWBST_EN(1),
		.TXSWBST_MAG(4),
		.TXSYNC_MULTILANE(1'b0),
		.TXSYNC_OVRD(1'b0),
		.TXSYNC_SKIP_DA(1'b0),
		.TX_CLKMUX_EN(1'b1),
		.TX_DCC_LOOP_RST_CFG(16'b0000000000000100),
		.TX_DEEMPH0(6'b000000),
		.TX_DEEMPH1(6'b000000),
		.TX_DEEMPH2(6'b000000),
		.TX_DEEMPH3(6'b000000),
		.TX_DIVRESET_TIME(5'b00001),
		.TX_DRIVE_MODE("DIRECT"),
		.TX_EIDLE_ASSERT_DELAY(3'b100),
		.TX_EIDLE_DEASSERT_DELAY(3'b011),
		.TX_FABINT_USRCLK_FLOP(1'b0),
		.TX_FIFO_BYP_EN(1'b0),
		.TX_IDLE_DATA_ZERO(1'b0),
		.TX_LOOPBACK_DRIVE_HIZ("FALSE"),
		.TX_MAINCURSOR_SEL(1'b0),
		.TX_MARGIN_FULL_0(7'b1011000),
		.TX_MARGIN_FULL_1(7'b1010111),
		.TX_MARGIN_FULL_2(7'b1010101),
		.TX_MARGIN_FULL_3(7'b1010011),
		.TX_MARGIN_FULL_4(7'b1010001),
		.TX_MARGIN_LOW_0(7'b1001100),
		.TX_MARGIN_LOW_1(7'b1001011),
		.TX_MARGIN_LOW_2(7'b1001000),
		.TX_MARGIN_LOW_3(7'b1000010),
		.TX_MARGIN_LOW_4(7'b1000000),
		.TX_PHICAL_CFG0(16'b0000000000100000),
		.TX_PHICAL_CFG1(16'b0000000001000000),
		.TX_PI_BIASSET(3),
		.TX_PMADATA_OPT(1'b0),
		.TX_PMA_POWER_SAVE(1'b0),
		.TX_PMA_RSV0(16'b0000000000000000),
		.TX_PMA_RSV1(16'b0000000000000000),
		.TX_PROGCLK_SEL("PREPI"),
		.TX_PROGDIV_CFG(0.0),
		.TX_PROGDIV_RATE(16'b0000000000000001),
		.TX_RXDETECT_CFG(14'b00000000110010),
		.TX_RXDETECT_REF(5),
		.TX_SAMPLE_PERIOD(3'b111),
		.TX_SW_MEAS(2'b00),
		.TX_VREG_CTRL(3'b011),
		.TX_VREG_PDB(1'b1),
		.TX_VREG_VREFSEL(2'b10),
		.USE_PCS_CLK_PHASE_SEL(1'b0),
		.Y_ALL_MODE(1'b0)
	) channel (

		//Top level ports
		.GTYRXP(rx_p),
		.GTYRXN(rx_n),

		.GTYTXP(tx_p),
		.GTYTXN(tx_n),

		//Data
		.TXDATA(tx_data_padded),
		.TXDATAEXTENDRSVD(1'b0),

		.RXDATA(rx_data_padded),
		.RXDATAEXTENDRSVD(),
		.RXDATAVALID(rx_data_valid),
		.RXHEADER(rx_header),
		.RXHEADERVALID(rx_header_valid),
		.RXGEARBOXSLIP(rx_gearbox_bitslip),

		//Idle, power down, and loopback controls
		.LOOPBACK(3'b000),		//normal operation
		.RXELECIDLEMODE(2'b11),	//not using OOB
		.RXELECIDLE(),
		.RXPD(2'b00),			//normal, powered up
		.TXPD(2'b00),			//normal, powered up
		.TXPDELECIDLEMODE(0),	//TXELECIDLE synchronous
		.TXELECIDLE(1'b0),		//not sending electrical idle

		//TX driver control
		.TXDEEMPH(1'b0),		//pcie mode, not used
		.TXDIFFCTRL(txdiffctrl),
		.TXINHIBIT(1'b0),		//transmitter always on
		.TXMAINCURSOR(7'h7f),
		.TXMARGIN(2'b00),
		.TXPOLARITY(tx_invert),
		.TXPOSTCURSOR(txpostcursor),
		.TXPRECURSOR(txprecursor),
		.TXSWING(1'b0),

		//RX PHY controls
		.RXPOLARITY(rx_invert),
		.RXTERMINATION(1'b0),

		//Reference clocks
		.GTGREFCLK(1'b0),
		.GTNORTHREFCLK0(clk_ref_north[0]),
		.GTNORTHREFCLK1(clk_ref_north[1]),
		.GTREFCLK0(clk_ref[0]),
		.GTREFCLK1(clk_ref[1]),
		.GTSOUTHREFCLK0(clk_ref_south[0]),
		.GTSOUTHREFCLK1(clk_ref_south[1]),

		//Input clocks
		.RXLATCLK(rxusrclk),
		.RXUSERRDY(rxuserrdy_gated),
		.RXUSRCLK(rxusrclk),
		.RXUSRCLK2(rxusrclk2),
		.SIGVALIDCLK(rxusrclk),
		.TXLATCLK(txusrclk),
		.TXPHDLYTSTCLK(txusrclk),
		.TXUSERRDY(txuserrdy_gated),
		.TXUSRCLK(txusrclk),
		.TXUSRCLK2(txusrclk2),

		//Output clocks
		.RXOUTCLKSEL(3'b010),	//RXOUTCLKPMA, recovered clock
		.RXPLLCLKSEL(rxpllclksel),
		.RXSLIPOUTCLK(1'b0),
		.RXSYSCLKSEL(rxpllclksel),
		.TXOUTCLKSEL(3'b010),	//TXOUTCLKPMA for TX
		.TXPLLCLKSEL(txpllclksel),
		.TXSYSCLKSEL(txpllclksel),
		.RXOUTCLK(rxoutclk_raw),
		.RXOUTCLKFABRIC(),
		.RXOUTCLKPCS(),
		.RXRECCLKOUT(),
		.TXOUTCLK(txoutclk_raw),
		.TXOUTCLKFABRIC(),
		.TXOUTCLKPCS(),

		//Clock outputs or something?
		.BUFGTCE(),
		.BUFGTCEMASK(),
		.BUFGTDIV(),

		//Clocks and status inputs from QPLL
		.QPLL0CLK(qpll_clk[0]),
		.QPLL0FREQLOCK(qpll_lock[0]),
		.QPLL0REFCLK(qpll_refclk[0]),
		.QPLL1CLK(qpll_clk[1]),
		.QPLL1FREQLOCK(qpll_lock[1]),
		.QPLL1REFCLK(qpll_refclk[1]),

		//CPLL status outputs
		.CPLLFBCLKLOST(cpll_fblost),
		.CPLLLOCK(cpll_lock),
		.CPLLREFCLKLOST(cpll_reflost),
		.CPLLFREQLOCK(1'b0),
		.CPLLLOCKDETCLK(clk_lockdet),
		.CPLLLOCKEN(1'b1),
		.CPLLPD(cpll_pd),
		.CPLLREFCLKSEL(cpll_refclk_sel),
		.CPLLRESET(1'b0),

		//Sub-rate control
		.RXRATE(rx_rate),
		.RXRATEMODE(1'b1),	//async control
		.TXRATE(tx_rate),
		.TXRATEMODE(1'b1),	//async control
		.RXRATEDONE(),
		.TXRATEDONE(),

		//Resets
		.GTRXRESET(rst_rx_sync),
		.GTRXRESETSEL(1'b0),	//sequential reset
		.GTTXRESET(rst_tx_sync),
		.GTTXRESETSEL(1'b0),	//sequential reset
		.RESETOVRD(1'b0),
		.RXBUFRESET(1'b0),
		.RXCDRFREQRESET(1'b0),
		.RXCDRRESET(1'b0),
		.RXCKCALRESET(1'b0),
		.RXDFELPMRESET(1'b0),
		.RXDLYSRESET(1'b0),
		.RXOOBRESET(1'b0),
		.RXOSCALRESET(1'b0),
		.RXPCSRESET(1'b0),
		.RXPHDLYRESET(1'b0),
		.RXPMARESET(1'b0),
		.RXPROGDIVRESET(1'b0),
		.TXDCCRESET(1'b0),
		.TXDLYSRESET(1'b0),
		.TXLFPSTRESET(1'b0),
		.TXPCSRESET(1'b0),
		.TXPHDLYRESET(1'b0),
		.TXPMARESET(1'b0),
		.TXPROGDIVRESET(1'b0),
		.BUFGTRESET(),
		.BUFGTRSTMASK(),

		.RXDLYSRESETDONE(),
		.RXPMARESETDONE(),
		.RXPRGDIVRESETDONE(),
		.RXRESETDONE(),
		.TXDLYSRESETDONE(),
		.TXPMARESETDONE(),
		.TXPRGDIVRESETDONE(),
		.TXRESETDONE(),

		//PRBS generator and checker
		.RXPRBSCNTRESET(1'b0),
		.RXPRBSSEL(rxprbssel),
		.TXPRBSFORCEERR(1'b0),
		.TXPRBSSEL(txprbssel),
		.RXPRBSERR(rxprbserr),
		.RXPRBSLOCKED(rxprbslocked),

		//RX 8B10B decoder and comma aligner
		.RX8B10BEN(rx_8b10b_decode),
		.RXCOMMADETEN(RX_COMMA_ALIGN),
		.RXCOMMADET(),
		.RXMCOMMAALIGNEN(RX_COMMA_ALIGN),
		.RXPCOMMAALIGNEN(RX_COMMA_ALIGN),
		.RXBYTEISALIGNED(rx_comma_is_aligned),
		.RXBYTEREALIGN(rx_commadet_slip),
		.RXCTRL0(rx_char_is_k),
		.RXCTRL1(rx_disparity_err),
		.RXCTRL2(rx_char_is_comma),
		.RXCTRL3(rx_symbol_err),
		.RXSLIDE(1'b0),

		//RX CDR
		.RXCDRHOLD(1'b0),
		.RXCDROVRDEN(1'b0),
		.RXCDRLOCK(),
		.RXCDRPHDONE(),

		//RX channel bonding
		.RXCHBONDEN(1'b0),
		.RXCHBONDI(),
		.RXCHBONDLEVEL(),
		.RXCHBONDMASTER(),
		.RXCHBONDSLAVE(),
		.RXCHANBONDSEQ(),
		.RXCHANISALIGNED(),
		.RXCHANREALIGN(),
		.RXCHBONDO(),

		//RX CTLE
		.RXLPMEN(rx_ctle_en),
		.RXLPMGCHOLD(),
		.RXLPMGCOVRDEN(),
		.RXLPMHFHOLD(),
		.RXLPMHFOVRDEN(),
		.RXLPMLFHOLD(),
		.RXLPMLFKLOVRDEN(),
		.RXLPMOSHOLD(),
		.RXLPMOSOVRDEN(),

		//RX DFE
		.RXDFEAGCHOLD(),
		.RXDFEAGCOVRDEN(),
		.RXDFECFOKFCNUM(),
		.RXDFECFOKFEN(),
		.RXDFECFOKFPULSE(),
		.RXDFECFOKHOLD(),
		.RXDFECFOKOVREN(),
		.RXDFEKHHOLD(),
		.RXDFEKHOVRDEN(),
		.RXDFELFHOLD(),
		.RXDFELFOVRDEN(),
		.RXDFETAP2HOLD(),
		.RXDFETAP2OVRDEN(),
		.RXDFETAP3HOLD(),
		.RXDFETAP3OVRDEN(),
		.RXDFETAP4HOLD(),
		.RXDFETAP4OVRDEN(),
		.RXDFETAP5HOLD(),
		.RXDFETAP5OVRDEN(),
		.RXDFETAP6HOLD(),
		.RXDFETAP6OVRDEN(),
		.RXDFETAP7HOLD(),
		.RXDFETAP7OVRDEN(),
		.RXDFETAP8HOLD(),
		.RXDFETAP8OVRDEN(),
		.RXDFETAP9HOLD(),
		.RXDFETAP9OVRDEN(),
		.RXDFETAP10HOLD(),
		.RXDFETAP10OVRDEN(),
		.RXDFETAP11HOLD(),
		.RXDFETAP11OVRDEN(),
		.RXDFETAP12HOLD(),
		.RXDFETAP12OVRDEN(),
		.RXDFETAP13HOLD(),
		.RXDFETAP13OVRDEN(),
		.RXDFETAP14HOLD(),
		.RXDFETAP14OVRDEN(),
		.RXDFETAP15HOLD(),
		.RXDFETAP15OVRDEN(),
		.RXDFEUTHOLD(),
		.RXDFEUTOVRDEN(),
		.RXDFEVPHOLD(),
		.RXDFEVPOVRDEN(),
		.RXDFEXYDEN(),

		//RX phase aligner
		.RXDLYBYPASS(/*1*/),
		.RXDLYEN(/*0*/),
		.RXDLYOVRDEN(/*0*/),
		.RXPHALIGN(/*0*/),
		.RXPHALIGNEN(/*0*/),
		.RXPHDLYPD(/*0*/),
		.RXPHALIGNDONE(),
		.RXPHALIGNERR(),
		.RXSYNCMODE(),

		//RX eye scan
		.EYESCANRESET(1'b0),
		.EYESCANTRIGGER(),
		.EYESCANDATAERROR(),

		//TX 8B/10B coder
		.TX8B10BBYPASS(),
		.TX8B10BEN(tx_8b10b_encode),
		.TXCTRL0(16'h0),	//TODO: handle data bit 8 when bypassing encoder
		.TXCTRL1(16'h0),	//TODO: handle data bit 9 when bypassing encoder
		.TXCTRL2(tx_char_is_k),

		//TX gearbox
		.TXHEADER(tx_header),
		.TXSEQUENCE(tx_sequence),

		//SATA/SAS (not used)
		.TXCOMINIT(),
		.TXCOMSAS(),
		.TXCOMWAKE(),
		.TXCOMFINISH(),
		.RXCOMINITDET(),
		.RXCOMSASDET(),
		.RXCOMWAKEDET(),

		//TX delay
		.TXDLYBYPASS(),
		.TXDLYEN(),
		.TXDLYHOLD(),
		.TXDLYOVRDEN(),
		.TXDLYUPDOWN(),

		//TX phase aligner
		.TXPHALIGN(1'b0),
		.TXPHALIGNEN(1'b0),
		.TXPHDLYPD(1'b0),
		.TXPHINIT(1'b0),
		.TXPHOVRDEN(1'b0),
		.TXPHALIGNDONE(),
		.TXPHINITDONE(),

		//TX interpolator
		.TXPIPPMEN(1'b0),
		.TXPIPPMOVRDEN(1'b0),
		.TXPIPPMPD(1'b0),
		.TXPIPPMSEL(1'b1),
		.TXPIPPMSTEPSIZE(1'b0),

		//TODO
		.TXSYNCDONE(),
		.TXSYNCOUT(),
		.RXSLIDERDY(),
		.RXSLIPDONE(),
		.RXSLIPOUTCLKRDY(),
		.RXSLIPPMARDY(),
		.RXSTARTOFSEQ(),
		.RXSTATUS(),
		.RXSYNCDONE(),
		.RXSYNCOUT(),
		.RXVALID(),
		.TXBUFSTATUS(),
		.RXLFPSTRESETDET(),
		.RXLFPSU2LPEXITDET(),
		.RXLFPSU3WAKEDET(),
		.RXMONITOROUT(),
		.RXOSINTDONE(),
		.RXOSINTSTARTED(),
		.RXOSINTSTROBEDONE(),
		.RXOSINTSTROBESTARTED(),
		.RXCKCALDONE(),
		.RXCLKCORCNT(),
		.RXBUFSTATUS(),
		.PHYSTATUS(),
		.PINRSRVDAS(),
		.POWERPRESENT(),
		.RESETEXCEPTION(),
		.GTPOWERGOOD(),
		.GTREFCLKMONITOR(),
		.DMONITOROUT(),
		.DMONITOROUTCLK(),
		.TXSYNCALLIN(),
		.TXSYNCIN(),
		.TXSYNCMODE(1'h0),
		.TXMUXDCDEXHOLD(),
		.TXMUXDCDORWREN(),
		.TXONESZEROS(),
		.TXLFPSU2LPEXIT(),
		.TXLFPSU3WAKE(),
		.TXDETECTRX(),
		.TXDCCFORCESTART(),
		.TSTIN(),
		.RXSYNCALLIN(),
		.RXSYNCIN(),
		.RXSLIPPMA(),
		.RXMONITORSEL(),
		.RXEQTRAINING(),
		.RXCKCALSTART(),
		.RXAFECFOKEN(),
		.CDRSTEPDIR(),
		.CDRSTEPSQ(),
		.CDRSTEPSX(),
		.DMONITORCLK(),
		.FREQOS(),
		.GTRSVD(),
		.INCPCTRL(),
		.RXOSHOLD(),
		.RXOSOVRDEN(),
		.TXDCCDONE(),

		//PCIe mode, ignore for now
		.PCIEEQRXEQADAPTDONE(),
		.PCIERSTIDLE(),
		.PCIERSTTXSYNCSTART(),
		.PCIEUSERRATEDONE(),
		.PCSRSVDIN(),
		.PCIERATEGEN3(),
		.PCIERATEIDLE(),
		.PCIERATEQPLLPD(),
		.PCIERATEQPLLRESET(),
		.PCIESYNCTXSYNCDONE(),
		.PCIEUSERGEN3RDY(),
		.PCIEUSERPHYSTATUSRST(),
		.PCIEUSERRATESTART(),
		.PCSRSVDOUT(),

		//DRP (bridged to APB)
		.DRPADDR({8'h0, apb.paddr[9:2]}),	//DADDR is 16 bit but not entire address space is used
		.DRPCLK(apb.pclk),
		.DRPEN(apb.penable),
		.DRPDI(apb.pwdata[15:0]),
		.DRPRDY(apb.pready),
		.DRPDO(apb.prdata[15:0]),
		.DRPWE(apb.pwrite),
		.DRPRST(!apb.preset_n),

		//Reserved ports, ignore or tie off
		.CLKRSVD0(1'h0),
		.CLKRSVD1(1'h0),
		.DMONFIFORESET(1'h0),
		.TXPISOPD(1'b0),
		.CFGRESET(1'b0)
	);

endmodule
