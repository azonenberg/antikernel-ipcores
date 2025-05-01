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

import EthernetBus::*;

/**
	@brief 10Gbase-R MAC and support logic
 */
module AXIS_XGEthernetMACWrapper #(
	parameter SERDES_TYPE = "GTY"		//legal values: GTY = UltraScale+ GTY
)(

	//SFP+ signals
	input wire				sfp_rx_p,
	input wire				sfp_rx_n,

	output wire				sfp_tx_p,
	output wire				sfp_tx_n,

	//System clock for resets etc
	input wire				clk_sys,

	//SERDES reference clocks
	input wire[1:0]			clk_ref_north,
	input wire[1:0]			clk_ref_south,
	input wire[1:0]			clk_ref,
	input wire				clk_lockdet,
	input wire[1:0]			qpll_clk,
	input wire[1:0]			qpll_refclk,
	input wire[1:0]			qpll_lock,

	//GPIO for loss-of-signal status
	input wire				sfp_rx_los,

	//SERDES control signals (TODO make this APB controlled?)
	input wire[1:0]			rxpllclksel,
	input wire[1:0]			txpllclksel,

	//AXI interfaces
	AXIStream.transmitter	eth_rx_data,
	AXIStream.receiver		eth_tx_data,

	//Status outputs (RX clock domain)
	output wire				link_up
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Validate parameters

	initial begin
		if(SERDES_TYPE != "GTY") begin
			$fatal("Invalid SERDES_TYPE (expected \"GTY\")");
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset block

	wire	rst_tx;
	wire	rst_rx;

	GTYLane_ResetController_UltraScale rst_ctl(
		.clk_system_in(clk_sys),

		.rst_async_tx_in(1'b0),
		.rst_async_rx_in(1'b0),

		.rst_gt_tx_out(rst_tx),
		.rst_gt_rx_out(rst_rx)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The transceiver

	//TODO: make the apb do something
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(10), .USER_WIDTH(0)) serdes_apb();
	assign serdes_apb.pclk = clk_sys;
	assign serdes_apb.preset_n = 1'b0;

	wire		rxusrclk;
	wire[31:0]	rx_data;
	wire		rx_data_valid;
	wire		rx_header_valid;
	wire[1:0]	rx_header;
	wire		rx_bitslip;

	wire		txusrclk;
	wire[5:0]	tx_sequence;
	wire[1:0]	tx_header;
	wire[31:0]	tx_data;

	GTYLane_UltraScale #(
		.RX_COMMA_ALIGN(0),
		.DATA_WIDTH(32),
		.ROUGH_RATE_GBPS(10),
		.GEARBOX_CFG("64b66b"),
		.RX_BUF_BYPASS(1)
	) serdes (
		.apb(serdes_apb),

		.rx_p(sfp_rx_p),
		.rx_n(sfp_rx_n),
		.tx_p(sfp_tx_p),
		.tx_n(sfp_tx_n),

		.rx_reset(rst_rx),
		.tx_reset(rst_tx),

		.clk_ref_north(clk_ref_north),
		.clk_ref_south(clk_ref_south),
		.clk_ref(clk_ref),
		.clk_lockdet(clk_lockdet),

		.rxusrclk(rxusrclk),
		.rxusrclk2(rxusrclk),
		.rxuserrdy(1'b1),
		.rxoutclk(rxusrclk),

		.txusrclk(txusrclk),
		.txusrclk2(txusrclk),
		.txuserrdy(1'b1),
		.txoutclk(txusrclk),

		.rxpllclksel(rxpllclksel),
		.txpllclksel(txpllclksel),

		.qpll_clk(qpll_clk),
		.qpll_refclk(qpll_refclk),
		.qpll_lock(qpll_lock),

		//TODO: long term we want to fix the reset derpiness and
		//use the CPLL for 10Gbase-R to keep QPLL free for other stuff
		.cpll_pd(1'b1),
		.cpll_fblost(),
		.cpll_reflost(),
		.cpll_lock(),
		.cpll_refclk_sel(3'b0),

		//For now just use the CTLE not the DFE
		.rx_ctle_en(1),

		//Assuming QPLL is set at 2x 10.3125 Gbps we want to run in half rate mode
		.tx_rate(3'h2),
		.rx_rate(3'h2),

		//Data bus TODO
		.tx_data(tx_data),
		.rx_data(rx_data),

		//Gearbox
		.rx_data_valid(rx_data_valid),
		.rx_header_valid(rx_header_valid),
		.rx_header(rx_header),
		.rx_gearbox_bitslip(rx_bitslip),
		.tx_header(tx_header),
		.tx_sequence(tx_sequence),

		//PRBS control
		.rxprbssel(4'b0),
		.txprbssel(4'b0),
		.rxprbserr(),
		.rxprbslocked(),

		.txdiffctrl(8),
		.txpostcursor(4),
		.txprecursor(),

		.tx_invert(1'b0),
		.rx_invert(1'b1),

		//not using 8b10b
		.rx_8b10b_decode(1'b0),
		.rx_comma_is_aligned(),
		.rx_char_is_k(),
		.rx_char_is_comma(),
		.rx_disparity_err(),
		.rx_symbol_err(),
		.rx_commadet_slip(),
		.tx_8b10b_encode(1'b0),
		.tx_char_is_k(16'h0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The PCS

	wire		xgmii_rx_clk;
	XgmiiBus 	xgmii_rx_bus;

	wire		xgmii_tx_clk;
	XgmiiBus	xgmii_tx_bus;

	wire		block_sync_good;
	wire		remote_fault;

	XGEthernetPCS pcs(
		.rx_clk(rxusrclk),
		.tx_clk(txusrclk),

		.rx_data_valid(rx_data_valid),
		.rx_header_valid(rx_header_valid),
		.rx_data(rx_data),
		.rx_header(rx_header),
		.rx_bitslip(rx_bitslip),

		.tx_sequence(tx_sequence),
		.tx_header(tx_header),
		.tx_data(tx_data),

		.tx_header_valid(),
		.tx_data_valid(),

		.xgmii_rx_clk(xgmii_rx_clk),
		.xgmii_rx_bus(xgmii_rx_bus),

		.xgmii_tx_clk(xgmii_tx_clk),
		.xgmii_tx_bus(xgmii_tx_bus),

		.sfp_los(sfp_rx_los),
		.block_sync_good(block_sync_good),
		.link_up(link_up),
		.remote_fault(remote_fault)
	);

	//DEBUG: always report link up, never send any traffic
	assign xgmii_tx_bus.ctl = 4'b1111;
	assign xgmii_tx_bus.data = { XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE, XGMII_CTL_IDLE };

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The MAC

	AXIS_XGEthernetMAC mac(
		.xgmii_rx_clk(xgmii_rx_clk),
		.xgmii_rx_bus(xgmii_rx_bus),

		.xgmii_tx_clk(xgmii_tx_clk),
		//.xgmii_tx_bus(xgmii_tx_bus),

		.link_up(link_up),

		.axi_rx(eth_rx_data),
		.axi_tx(eth_tx_data)
	);

endmodule
