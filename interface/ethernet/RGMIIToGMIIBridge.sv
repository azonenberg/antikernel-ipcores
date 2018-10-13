`timescale 1ns / 1ps
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
	@brief GMII to [R]GMII converter
 */
module RGMIIToGMIIBridge(

	//RGMII signals to off-chip device
	input wire			rgmii_rxc,
	input wire[3:0]		rgmii_rxd,
	input wire			rgmii_rx_ctl,

	output wire			rgmii_txc,
	output wire[3:0]	rgmii_txd,
	output wire			rgmii_tx_ctl,

	//GMII signals to MAC
	output wire			gmii_rxc,
	output wire[7:0]	gmii_rxd,
	output wire			gmii_rx_dv,
	output wire			gmii_rx_er,

	input wire			gmii_txc,
	input wire			gmii_txc_90,
	input wire[7:0]		gmii_txd,
	input wire			gmii_tx_en,
	input wire			gmii_tx_er,

	//In-band status (if supported by the PHY)
	output logic		link_up		= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side

	//Delay the GMII clock to add additional skew as required by RGMII
	//UI is 4000 ps (250 MT/s).
	//Skew at the sender is +/- 500 ps so our valid eye ignoring rise time is from +500 to +3500 ps
	//We need at least 10 ps setup and 340ps hold for Kintex-7, sampling eye is now now +510 to +3160 ps
	//Assuming equal rise/fall time we want to center in this window so use phase offset of 1835 ps.
	//We want the clock moved forward by +1835 ps, which is the same as moving it back by (8000 - 1835) = 2165 ps and
	//inverting it.
	//Rather than inverting the clock, we instead just switch the phases off the DDR buffers.
	wire	gmii_rxc_raw;
	IODelayBlock #(
		.WIDTH(1),
		.INPUT_DELAY(2165),
		.OUTPUT_DELAY(0),
		.DIRECTION("IN"),
		.IS_CLOCK(1)
	) rgmii_rxc_idelay (
		.i_pad(rgmii_rxc),
		.i_fabric(gmii_rxc_raw),
		.i_fabric_serdes(),
		.o_pad(),								//not using output datapath
		.o_fabric(1'h0),
		.input_en(1'b1)
	);

	ClockBuffer #(
		.CE("NO"),
		.TYPE("GLOBAL")
	) rxc_buf (
		.clkin(gmii_rxc_raw),
		.ce(1'b1),
		.clkout(gmii_rxc)
	);

	//Convert DDR to SDR signals
	wire[7:0]	gmii_rxd_parallel;
	DDRInputBuffer #(.WIDTH(4)) rgmii_rxd_iddr2(
		.clk_p(gmii_rxc),
		.clk_n(~gmii_rxc),
		.din(rgmii_rxd),
		.dout0(gmii_rxd_parallel[7:4]),
		.dout1(gmii_rxd_parallel[3:0])
		);

	wire gmii_rx_er_raw;
	wire[1:0]	gmii_rxc_parallel;
	DDRInputBuffer #(.WIDTH(1)) rgmii_rxe_iddr2(
		.clk_p(gmii_rxc),
		.clk_n(~gmii_rxc),
		.din(rgmii_rx_ctl),
		.dout0(gmii_rxc_parallel[1]),
		.dout1(gmii_rxc_parallel[0])
		);

	//Register the signals by one cycle so we can compensate for the clock inversion
	logic[7:0]	gmii_rxd_parallel_ff	= 0;
	logic[1:0]	gmii_rxc_parallel_ff	= 0;
	always_ff @(posedge gmii_rxc) begin
		gmii_rxd_parallel_ff	<= gmii_rxd_parallel;
		gmii_rxc_parallel_ff	<= gmii_rxc_parallel;
	end

	//Shuffle the nibbles data back to where they should be.
	assign gmii_rxd	= { gmii_rxd_parallel_ff[3:0], gmii_rxd_parallel[7:4] };

	//rx_er flag is encoded specially to reduce transitions (see RGMII spec section 3.4)
	assign gmii_rx_dv = gmii_rxc_parallel_ff[0];
	wire gmii_rx_er_int = gmii_rxc_parallel_ff[0] ^ gmii_rxc_parallel_ff[1];

	assign gmii_rx_er = gmii_rx_er_int && gmii_rx_dv;

	/*
		If gmii_rx_er_int is set without gmii_rx_dv, that's a special symbol of some sort
		For now, we just ignore them
		0xFF = carrier sense (not meaningful in full duplex)
	 */

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side

	//For some reason we have to invert the transmit clock on 7 series.
	//No idea why this is necessary.
	`ifdef XILINX_7SERIES
		DDROutputBuffer #
		(
			.WIDTH(1)
		) txc_output
		(
			.clk_p(gmii_txc_90),
			.clk_n(~gmii_txc_90),
			.dout(rgmii_txc),
			.din0(1'b1),
			.din1(1'b0)
		);

	`else
		DDROutputBuffer #
		(
			.WIDTH(1)
		) txc_output
		(
			.clk_p(gmii_txc_90),
			.clk_n(~gmii_txc_90),
			.dout(rgmii_txc),
			.din0(1'b0),
			.din1(1'b1)
		);
	`endif

	DDROutputBuffer #(.WIDTH(4)) rgmii_txd_oddr2(
		.clk_p(gmii_rxc),
		.clk_n(~gmii_rxc),
		.dout(rgmii_txd),
		.din0(gmii_txd[3:0]),
		.din1(gmii_txd[7:4])
		);

	DDROutputBuffer #(.WIDTH(1)) rgmii_txe_oddr2(
		.clk_p(gmii_rxc),
		.clk_n(~gmii_rxc),
		.dout(rgmii_tx_ctl),
		.din0(gmii_tx_en),
		.din1(gmii_tx_en ^ gmii_tx_er)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decode in-band link state

	always_ff @(posedge gmii_rxc) begin
		if(!gmii_rx_dv && !gmii_rx_er_int) begin

			//Inter-frame gap
			link_up		<= gmii_rxd[0];

		end
	end

endmodule
