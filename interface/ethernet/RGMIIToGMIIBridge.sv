`timescale 1ns / 1ps
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

`include "GmiiBus.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief GMII to [R]GMII converter
 */
module RGMIIToGMIIBridge #(
	parameter PHY_INTERNAL_DELAY_RX = 1		//when enabled, the PHY provides internal delays and we don't have to
											//do any phase shifting on the input
)(

	//RGMII signals to off-chip device
	input wire			rgmii_rxc,
	input wire[3:0]		rgmii_rxd,
	input wire			rgmii_rx_ctl,

	output wire			rgmii_txc,
	output wire[3:0]	rgmii_txd,
	output wire			rgmii_tx_ctl,

	//GMII signals to MAC
	output wire			gmii_rxc,
	output GmiiBus		gmii_rx_bus = {1'b0, 1'b0, 1'b0, 8'b0},

	input wire			gmii_txc,		//must be 125 MHz regardless of speed
	input wire			clk_250mhz,		//must be phase aligned with and 4x speed of gmii_txc
	input wire GmiiBus	gmii_tx_bus,

	//In-band status (if supported by the PHY)
	output logic		link_up		= 0,
	output lspeed_t		link_speed	= LINK_SPEED_10M
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side

	wire	gmii_rxc_raw;
	generate
		if(PHY_INTERNAL_DELAY_RX) begin
			assign gmii_rxc_raw = rgmii_rxc;
		end
		else begin
			//Delay the GMII clock to add additional skew as required by RGMII
			//UI is 4000 ps (250 MT/s).
			//Skew at the sender is +/- 500 ps so our valid eye ignoring rise time is from +500 to +3500 ps
			//We need at least 10 ps setup and 340ps hold for Kintex-7, sampling eye is now now +510 to +3160 ps
			//Assuming equal rise/fall time we want to center in this window so use phase offset of 1835 ps.
			//We want the clock moved forward by +1835 ps, which is the same as moving it back by (8000 - 1835) = 2165 ps and
			//inverting it.
			//Rather than inverting the clock, we instead just switch the phases off the DDR buffers.
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
		end

		ClockBuffer #(
			.CE("NO"),
			.TYPE("GLOBAL")
		) rxc_buf (
			.clkin(gmii_rxc_raw),
			.ce(1'b1),
			.clkout(gmii_rxc)
		);

		wire[7:0]	gmii_rx_data_parallel;
		wire[1:0]	gmii_rxc_parallel;
		logic[7:0]	gmii_rx_data_parallel_ff	= 0;
		logic[1:0]	gmii_rxc_parallel_ff		= 0;

		logic[7:0]	rx_parallel_data;

		//Convert DDR to SDR signals
		if(PHY_INTERNAL_DELAY_RX) begin
			DDRInputBuffer #(.WIDTH(4)) rgmii_rxd_iddr2(
				.clk_p(gmii_rxc),
				.clk_n(~gmii_rxc),
				.din(rgmii_rxd),
				.dout0(gmii_rx_data_parallel[3:0]),
				.dout1(gmii_rx_data_parallel[7:4])
				);
			DDRInputBuffer #(.WIDTH(1)) rgmii_rxe_iddr2(
				.clk_p(gmii_rxc),
				.clk_n(~gmii_rxc),
				.din(rgmii_rx_ctl),
				.dout0(gmii_rxc_parallel[0]),
				.dout1(gmii_rxc_parallel[1])
				);

			//Register the signals by one cycle
			always_ff @(posedge gmii_rxc) begin
				gmii_rx_data_parallel_ff	<= gmii_rx_data_parallel;
				gmii_rxc_parallel_ff		<= gmii_rxc_parallel;
			end

			//Shuffle the nibbles data back to where they should be.
			always_comb begin
				if(link_speed == LINK_SPEED_1000M)
					rx_parallel_data <= gmii_rx_data_parallel_ff;
				else
					rx_parallel_data <= { gmii_rx_data_parallel[7:4], gmii_rx_data_parallel_ff[3:0] };
			end

		end
		else begin
			DDRInputBuffer #(.WIDTH(4)) rgmii_rxd_iddr2(
				.clk_p(gmii_rxc),
				.clk_n(~gmii_rxc),
				.din(rgmii_rxd),
				.dout0(gmii_rx_data_parallel[7:4]),
				.dout1(gmii_rx_data_parallel[3:0])
				);
			DDRInputBuffer #(.WIDTH(1)) rgmii_rxe_iddr2(
				.clk_p(gmii_rxc),
				.clk_n(~gmii_rxc),
				.din(rgmii_rx_ctl),
				.dout0(gmii_rxc_parallel[1]),
				.dout1(gmii_rxc_parallel[0])
				);

			//Register the signals by one cycle so we can compensate for the clock inversion
			always_ff @(posedge gmii_rxc) begin
				gmii_rx_data_parallel_ff	<= gmii_rx_data_parallel;
				gmii_rxc_parallel_ff		<= gmii_rxc_parallel;
			end

			//Shuffle the nibbles data back to where they should be.
			assign rx_parallel_data	= { gmii_rx_data_parallel[7:4], gmii_rx_data_parallel_ff[3:0] };
		end

		//10/100 un-DDR-ing
		logic		gmii_rx_bus_en_ff	= 0;
		logic		dvalid_raw			= 0;
		logic[31:0]	rx_parallel_data_ff	= 0;
		always_ff @(posedge gmii_rxc) begin
			gmii_rx_bus_en_ff	<= gmii_rx_bus.en;
			rx_parallel_data_ff	<= rx_parallel_data;

			//Sync on start of packet
			if(gmii_rx_bus.en && !gmii_rx_bus_en_ff)
				dvalid_raw		<= 1;

			//Toggle the rest of the time
			else
				dvalid_raw		<= !dvalid_raw;

		end

		always_comb begin
			gmii_rx_bus.data	<= rx_parallel_data;

			if(link_speed == LINK_SPEED_1000M)
				gmii_rx_bus.dvalid	<= 1;

			//special lock for start of packet
			else if(gmii_rx_bus.en && !gmii_rx_bus_en_ff)
				gmii_rx_bus.dvalid	<= 0;

			else if(dvalid_raw) begin
				gmii_rx_bus.data	<= rx_parallel_data_ff;
				gmii_rx_bus.dvalid	<= 1;
			end

			else
				gmii_rx_bus.dvalid	<= 0;


		end

		//rx_er flag is encoded specially to reduce transitions (see RGMII spec section 3.4)
		assign gmii_rx_bus.en = gmii_rxc_parallel_ff[0];
		wire gmii_rx_er_int = gmii_rxc_parallel_ff[0] ^ gmii_rxc_parallel_ff[1];

		assign gmii_rx_bus.er = gmii_rx_er_int && gmii_rx_bus.en;

		/*
			If gmii_rx_er_int is set without gmii_rx_bus.en, that's a special symbol of some sort
			For now, we just ignore them
			0xFF = carrier sense (not meaningful in full duplex)
		 */

	endgenerate

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize link speed into TX clock domain

	lspeed_t	link_speed_ff		= LINK_SPEED_10M;
	logic		link_speed_change	= 0;

	always_ff @(posedge gmii_rxc) begin
		link_speed_change	<= (link_speed_ff != link_speed);
		link_speed_ff		<= link_speed;
	end

	wire[1:0]	link_speed_sync_raw;
	lspeed_t	link_speed_sync = lspeed_t'(link_speed_sync_raw);

	RegisterSynchronizer #(
		.WIDTH(2),
		.INIT(0)
	) sync_link_speed(
		.clk_a(gmii_rxc),
		.en_a(link_speed_change),
		.ack_a(),
		.reg_a(link_speed),

		.clk_b(gmii_txc),
		.updated_b(),
		.reg_b(link_speed_sync_raw)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side

	logic[3:0]	clock_dout 		= 4'b0000;
	logic[7:0]	clock_count		= 8'h0;
	logic		update_data		= 0;

	//Clock phasing control
	always_ff @(posedge gmii_txc) begin

		//Gig mode: send clock phased to rise half a cycle after our data goes out
		if(link_speed_sync == LINK_SPEED_1000M) begin
			clock_dout		<= 4'b1001;
			update_data		<= 1;
		end

		//10/100 mode: nice long delays between toggles
		else begin

			update_data		<= 0;

			if(gmii_tx_bus.dvalid) begin
				clock_count	<= 0;
				clock_dout	<= 4'b0000;
			end

			clock_count <= clock_count + 1'h1;

			//10M: we want 2.5 MHz so toggle at 5 MT/s (every 25 cycles)
			if( (link_speed_sync == LINK_SPEED_10M) && (clock_count == 24) ) begin
				clock_count		<= 0;
				if(clock_dout)
					update_data	<= 1;
				clock_dout		<= ~clock_dout;
			end

			//100M: we want 25 MHz. this adds a bit of fun since we're dividing by 5
			//edge is at nominal cycle 2.5
			else if(link_speed_sync == LINK_SPEED_100M) begin

				case(clock_count)

					0:	clock_dout	<= 4'b0000;
					1:	clock_dout	<= 4'b0000;
					2:	clock_dout	<= 4'b1100;	//serdes sends LSB first so 1100 is a rising edge!
					3:	clock_dout	<= 4'b1111;
					4: begin
						clock_dout	<= 4'b1111;
						clock_count	<= 0;
						update_data	<= 1;
					end

				endcase

			end

		end

	end

	//reset for 256 clocks after boot
	logic		oserdes_rst	= 1;
	logic[7:0]	rst_count	= 0;
	always_ff @(posedge gmii_txc) begin
		if(oserdes_rst) begin
			rst_count <= rst_count + 1'h1;
			if(rst_count == 8'hff)
				oserdes_rst	<= 0;
		end
	end

	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"),
		.DATA_RATE_TQ("BUF"),
		.DATA_WIDTH(4),
		.SERDES_MODE("MASTER"),
		.TRISTATE_WIDTH(1)
	) txc_output (
		.OQ(rgmii_txc),
		.OFB(),
		.TQ(),
		.TFB(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.CLK(clk_250mhz),
		.CLKDIV(gmii_txc),
		.D1(clock_dout[0]),
		.D2(clock_dout[1]),
		.D3(clock_dout[2]),
		.D4(clock_dout[3]),
		.D5(1'b0),	//unused
		.D6(1'b0),	//unused
		.D7(1'b0),	//unused
		.D8(1'b0),	//unused
		.TCE(1'b1),
		.OCE(1'b1),
		.TBYTEIN(1'b0),
		.TBYTEOUT(),
		.RST(oserdes_rst),
		.SHIFTIN1(1'b0),
		.SHIFTIN2(1'b0),
		.T1(1'b0),
		.T2(1'b0),
		.T3(1'b0),
		.T4(1'b0)
	);

	//In 10/100 mode alternate sending low nibble twice and high nibble twice
	logic	tx_phase	= 0;
	logic	tx_en_ff	= 0;

	always_ff @(posedge gmii_txc) begin
		tx_en_ff		<= gmii_tx_bus.en;
		if(gmii_tx_bus.en && !tx_en_ff)
			tx_phase	<= 0;
		if(update_data)
			tx_phase	<= !tx_phase;
	end

	logic[3:0]	tx_data_hi;
	logic[3:0]	tx_data_lo;

	always_comb begin
		if(link_speed_sync == LINK_SPEED_1000M) begin
			tx_data_lo	<= gmii_tx_bus.data[3:0];
			tx_data_hi	<= gmii_tx_bus.data[7:4];
		end
		else if(tx_phase == 0) begin
			tx_data_lo	<= gmii_tx_bus.data[3:0];
			tx_data_hi	<= gmii_tx_bus.data[3:0];
		end
		else if(tx_phase == 1) begin
			tx_data_lo	<= gmii_tx_bus.data[7:4];
			tx_data_hi	<= gmii_tx_bus.data[7:4];
		end
	end

	DDROutputBuffer #(.WIDTH(4)) rgmii_txd_oddr2(
		.clk_p(gmii_txc),
		.clk_n(~gmii_txc),
		.dout(rgmii_txd),
		.din0(tx_data_lo),
		.din1(tx_data_hi)
		);

	DDROutputBuffer #(.WIDTH(1)) rgmii_txe_oddr2(
		.clk_p(gmii_txc),
		.clk_n(~gmii_txc),
		.dout(rgmii_tx_ctl),
		.din0(gmii_tx_bus.en),
		.din1(gmii_tx_bus.en ^ gmii_tx_bus.er)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decode in-band link state

	//In-band status flags
	logic	false_carrier	= 0;

	always_ff @(posedge gmii_rxc) begin
		false_carrier	<= 0;

		//Decode in-band status
		if(!gmii_rx_bus.en) begin

			//Normal inter-frame gap. Decode link state/speed
			if(!gmii_rx_er_int) begin
				link_up		<= gmii_rx_bus.data[0];
				link_speed	<= lspeed_t'(gmii_rx_bus.data[2:1]);
			end

			else begin
				if(gmii_rx_bus.data == 8'h0e)
					false_carrier	<= 1;
			end

		end
	end

endmodule
