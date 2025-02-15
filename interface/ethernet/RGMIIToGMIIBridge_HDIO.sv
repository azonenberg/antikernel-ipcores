`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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
	@file
	@author Andrew D. Zonenberg
	@brief GMII to [R]GMII converter for UltraScale+ HDIO banks

	Note that the GMII bus isn't quite standard GMII in 10/100 mode!

	Requirements from the PHY:
		In-band status enabled
		Internal delay with optimized timing
			For KSZ9031RNX:
				mmd 2 reg 8 = 3ffa (adjust GTX_CLK / RX_CLK delay lines)

	Timing constraints for above register settings:
		set_input_delay -clock [get_clocks rgmii_rxc] -clock_fall -min -add_delay 0.54 [get_ports [ list rgmii_rxd[3] rgmii_rxd[2] rgmii_rxd[1] rgmii_rxd[0] rgmii_rx_ctl ]]
		set_input_delay -clock [get_clocks rgmii_rxc] -clock_fall -max -add_delay 2.14 [get_ports [ list rgmii_rxd[3] rgmii_rxd[2] rgmii_rxd[1] rgmii_rxd[0] rgmii_rx_ctl ]]
		set_input_delay -clock [get_clocks rgmii_rxc] -min -add_delay 0.54 [get_ports [ list rgmii_rxd[3] rgmii_rxd[2] rgmii_rxd[1] rgmii_rxd[0] rgmii_rx_ctl ]]
		set_input_delay -clock [get_clocks rgmii_rxc] -max -add_delay 2.14 [get_ports [ list rgmii_rxd[3] rgmii_rxd[2] rgmii_rxd[1] rgmii_rxd[0] rgmii_rx_ctl ]]

		set_output_delay -clock [get_clocks rgmii_txc] -clock_fall -min -add_delay 1.96 [get_ports [ list rgmii_txd[0] rgmii_txd[1] rgmii_txd[2] rgmii_txd[3] rgmii_tx_ctl ]]
		set_output_delay -clock [get_clocks rgmii_txc] -clock_fall -max -add_delay 0.04 [get_ports [ list rgmii_txd[0] rgmii_txd[1] rgmii_txd[2] rgmii_txd[3] rgmii_tx_ctl ]]
		set_output_delay -clock [get_clocks rgmii_txc] -min -add_delay 1.96 [get_ports [ list rgmii_txd[0] rgmii_txd[1] rgmii_txd[2] rgmii_txd[3] rgmii_tx_ctl ]]
		set_output_delay -clock [get_clocks rgmii_txc] -max -add_delay 0.04 [get_ports [ list rgmii_txd[0] rgmii_txd[1] rgmii_txd[2] rgmii_txd[3] rgmii_tx_ctl ]]
 */
module RGMIIToGMIIBridge_HDIO(

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
	//TODO: secondary clock?
	input wire GmiiBus	gmii_tx_bus,

	//In-band status (if supported by the PHY)
	output logic		link_up			= 0,
	output lspeed_t		link_speed		= LINK_SPEED_10M,
	output logic		false_carrier	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side

	ClockBuffer #(
		.CE("NO"),
		.TYPE("GLOBAL")
	) rxc_buf (
		.clkin(rgmii_rxc),
		.ce(1'b1),
		.clkout(gmii_rxc)
	);

	wire[7:0]	gmii_rx_data_parallel;
	wire[1:0]	gmii_rxc_parallel;
	logic[7:0]	gmii_rx_data_parallel_ff	= 0;
	logic[1:0]	gmii_rxc_parallel_ff		= 0;

	logic[7:0]	rx_parallel_data;

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

	//10/100 un-DDR-ing
	logic		gmii_rx_bus_en_ff	= 0;
	logic		dvalid_raw			= 0;
	logic[7:0]	rx_parallel_data_ff	= 0;
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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decode in-band link state

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize link speed into TX clock domain
	// Use separate syncs, OK if they glitch for a cycle during transition

	lspeed_t	link_speed_sync;

	ThreeStageSynchronizer sync_link_speed_0(
		.clk_in(gmii_rxc),
		.din(link_speed[0]),
		.clk_out(gmii_txc),
		.dout(link_speed_sync[0])
	);

	ThreeStageSynchronizer sync_link_speed_1(
		.clk_in(gmii_rxc),
		.din(link_speed[1]),
		.clk_out(gmii_txc),
		.dout(link_speed_sync[1])
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline and edge-detect TX side data bus

	logic		tx_valid_ff	= 0;
	logic		tx_en_ff	= 0;
	logic		tx_er_ff	= 0;
	logic		tx_en_ff2	= 0;
	logic[7:0]	tx_data_ff	= 0;
	logic		tx_starting;

	always_ff @(posedge gmii_txc) begin
		tx_valid_ff	<= gmii_tx_bus.dvalid;
		tx_en_ff	<= gmii_tx_bus.en;
		tx_er_ff	<= gmii_tx_bus.er;
		tx_en_ff2	<= tx_en_ff;
		tx_data_ff	<= gmii_tx_bus.data;
	end

	always_comb begin
		tx_starting	= gmii_tx_bus.en && !tx_en_ff;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side clock generation

	//In 10/100 mode alternate sending low nibble twice and high nibble twice
	logic		tx_phase	= 0;
	logic		tx_en_ff	= 0;

	logic[7:0]	clock_count		= 8'h0;

	//Clock phasing control
	logic[1:0]	clock_dout		= 2'b01;
	logic[1:0]	clock_dout_ff	= 2'b01;
	always_ff @(posedge gmii_txc) begin

		tx_en_ff			<= gmii_tx_bus.en;

		//Gig mode: always pushing new data every cycle
		if(link_speed_sync == LINK_SPEED_1000M) begin
			clock_dout		<= 2'b01;
		end

		//10/100 mode: nice long delays between toggles
		else begin

			clock_count <= clock_count + 1'h1;

			//10M: we want 2.5 MHz so toggle at 5 MT/s (every 25 cycles)
			if( (link_speed_sync == LINK_SPEED_10M) && (clock_count == 24) ) begin
				clock_count		<= 0;
				if(clock_dout) begin
					tx_phase	<= !tx_phase;
					clock_dout	<= 2'b00;
				end
				else
					clock_dout	<= 2'b11;
			end

			//100M: we want 25 MHz. this adds a bit of fun since we're dividing by 5
			//edge is at nominal cycle 2.5
			else if(link_speed_sync == LINK_SPEED_100M) begin

				case(clock_count)

					0:	clock_dout	<= 2'b11;
					1:	clock_dout	<= 2'b11;
					2: 	clock_dout	<= 2'b01;
					3: 	clock_dout	<= 2'b00;
					4: begin
						clock_dout	<= 2'b00;
						clock_count	<= 0;
						tx_phase	<= !tx_phase;
					end

				endcase

			end

			//Reset at start of frame
			if(tx_starting) begin
				clock_count	<= 0;
				clock_dout	<= 2'b00;
				tx_phase	<= 0;
			end

		end

	end

	//pipeline delay
	always_ff @(posedge gmii_txc) begin
		clock_dout_ff	<= clock_dout;
	end

	//Echo the TX clock (use constraints to phase shift clock routing a bit)
	//This actually works better than using a second PLL phase, since vivado has trouble constraining that apparently
	DDROutputBuffer #(.WIDTH(1)) rgmii_txc_oddr(
		.clk_p(gmii_txc),
		.clk_n(~gmii_txc),
		.dout(rgmii_txc),
		.din0(clock_dout_ff[0]),
		.din1(clock_dout_ff[1])
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side datapath

	logic[3:0]	tx_data_hi;
	logic[3:0]	tx_data_lo;

	always_comb begin
		if(link_speed_sync == LINK_SPEED_1000M) begin
			tx_data_lo	= tx_data_ff[3:0];
			tx_data_hi	= tx_data_ff[7:4];
		end
		else if(tx_phase == 0) begin
			tx_data_lo	= tx_data_ff[3:0];
			tx_data_hi	= tx_data_ff[3:0];
		end
		else/* if(tx_phase == 1)*/ begin
			tx_data_lo	= tx_data_ff[7:4];
			tx_data_hi	= tx_data_ff[7:4];
		end
	end

	//Register output data to improve timing
	logic[3:0]	tx_data_lo_ff = 0;
	logic[3:0]	tx_data_hi_ff = 0;
	logic		tx_ctl_lo_ff = 0;
	logic		tx_ctl_hi_ff = 0;
	always_ff @(posedge gmii_txc) begin
		tx_data_lo_ff	<= tx_data_lo;
		tx_data_hi_ff	<= tx_data_hi;
		tx_ctl_lo_ff	<= tx_en_ff;
		tx_ctl_hi_ff	<= tx_en_ff ^ tx_er_ff;
	end

	DDROutputBuffer #(.WIDTH(4)) rgmii_txd_oddr(
		.clk_p(gmii_txc),
		.clk_n(~gmii_txc),
		.dout(rgmii_txd),
		.din0(tx_data_lo_ff),
		.din1(tx_data_hi_ff)
		);

	DDROutputBuffer #(.WIDTH(1)) rgmii_txe_oddr(
		.clk_p(gmii_txc),
		.clk_n(~gmii_txc),
		.dout(rgmii_tx_ctl),
		.din0(tx_ctl_lo_ff),
		.din1(tx_ctl_hi_ff)
		);

endmodule
