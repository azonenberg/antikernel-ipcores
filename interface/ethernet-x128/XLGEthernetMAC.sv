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
	@file 	XLGEthernetMAC.sv
	@author Andrew D. Zonenberg
	@brief	40Gbps Ethernet MAC
 */
`include "EthernetBus.svh"

/**
	@brief 40G Ethernet MAC
 */
module XLGEthernetMAC(

	input wire				rx_clk,
	input wire xlgmii128	rx_mac_bus,

	output EthernetRxBus	rx_bus
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX: Pipeline the XLGMII bus by one cycle and detect control characters.

	xlgmii128		rx_mac_bus_ff	= 0;

	logic			start_left		= 0;
	logic			start_right		= 0;

	logic[15:0]		end_found		= 0;
	logic[15:0]		err_found		= 0;

	logic			ctl_left		= 0;
	logic			ctl_left_low	= 0;
	logic			ctl_right		= 0;
	logic			ctl_right_low	= 0;

	always_ff @(posedge rx_clk) begin
		rx_mac_bus_ff	<= rx_mac_bus;

		start_left		<= rx_mac_bus.ctl[15] && (rx_mac_bus.data[127:120] == XLGMII_CTL_START);
		start_right		<= rx_mac_bus.ctl[7] && (rx_mac_bus.data[63:56] == XLGMII_CTL_START);

		ctl_left		<= (rx_mac_bus.ctl[15:8] != 0);
		ctl_left_low	<= (rx_mac_bus.ctl[14:8] != 0);
		ctl_right		<= (rx_mac_bus.ctl[7:0] != 0);
		ctl_right_low	<= (rx_mac_bus.ctl[6:0] != 0);

		for(integer i=0; i<16; i++) begin
			end_found[i]	<= rx_mac_bus.ctl[i] && (rx_mac_bus.data[i*8 +: 8] == XLGMII_CTL_TERMINATE);

			//Data isn't an error
			if(!rx_mac_bus.ctl[i])
				err_found[i]	<= 0;

			//Neither is known non-error control characters
			else if(	(rx_mac_bus.data[i*8 +: 8] == XLGMII_CTL_LPI) ||
						(rx_mac_bus.data[i*8 +: 8] == XLGMII_CTL_IDLE) ||
						(rx_mac_bus.data[i*8 +: 8] == XLGMII_CTL_SEQUENCE) ||
						(rx_mac_bus.data[i*8 +: 8] == XLGMII_CTL_START) ||
						(rx_mac_bus.data[i*8 +: 8] == XLGMII_CTL_TERMINATE) ) begin
				err_found[i]	<= 0;
			end

			//But anything else is!
			else
				err_found[i]	<= 1;

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX start/stop detection, preamble stripping, and block alignment

	EthernetRxBus	rx_bus_internal;

	logic[63:0]		rx_data_ff;

	enum logic[1:0]
	{
		RX_STATE_IDLE	= 0,
		RX_STATE_FIRST	= 1,
		RX_STATE_ACTIVE	= 2
	} rx_state = RX_STATE_IDLE;

	logic			rx_phase 			= 0;
	logic[4:0]		rx_last_valid		= 0;

	logic			output_final_word	= 0;

	always_ff @(posedge rx_clk) begin

		rx_bus_internal		<= 0;

		//Save low half of incoming data in case we're phase shifting
		rx_data_ff			<= rx_mac_bus_ff.data[63:0];

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Waiting for frame to start

			RX_STATE_IDLE: begin

				if(output_final_word) begin
					output_final_word	<= 0;
					rx_bus_internal.data		<= { rx_data_ff, 64'h0 };
					rx_bus_internal.commit		<= 1;
					rx_bus_internal.data_valid	<= 1;
					rx_bus_internal.bytes_valid	<= rx_last_valid;
				end

				//Ignore error/idle characters
				//Look for start sequence in the first lane of either the left or right half

				//Start sequence in left half? Left half is preamble, right half is beginning of frame.
				if(start_left) begin

					//If we have any control characters elsewhere in the frame, something went wrong.
					//Discard this frame as a runt.
					if(ctl_left_low || ctl_right) begin
						//TODO: performance counter
					end

					//All good
					else begin
						rx_phase	<= 0;
						rx_state	<= RX_STATE_FIRST;
					end

				end

				//Start sequence in right half?
				else if(start_right) begin

					//If we have any control characters elsewhere in the frame, something went wrong.
					//Discard this frame as a runt.
					if(ctl_right_low) begin
						//TODO: performance counter
					end

					//All good
					else begin
						rx_phase	<= 1;
						rx_state	<= RX_STATE_FIRST;
					end
				end

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// First cycle of new frame

			RX_STATE_FIRST: begin

				//We should not have any control characters in the first 16 bytes of a frame,
				//since min frame size is 64 bytes.
				//If we see anything, discard it the frame as a runt.
				if(ctl_left || ctl_right) begin
					rx_state		<= RX_STATE_IDLE;
					//TODO: performance counter
				end

				//Good data
				else begin

					rx_bus_internal.start		<= 1;
					rx_bus_internal.data_valid	<= 1;
					rx_bus_internal.bytes_valid	<= 16;
					rx_state					<= RX_STATE_ACTIVE;

					//Preamble was in right half of last cycle.
					//We now have a full cycle of data.
					if(rx_phase)
						rx_bus_internal.data	<= rx_mac_bus_ff.data;

					//Preamble was in left half of last cycle. Need to shift to align stuff.
					else
						rx_bus_internal.data	<= { rx_data_ff, rx_mac_bus_ff.data[127:64] };

				end

			end	//end RX_STATE_FIRST

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Frame data

			RX_STATE_ACTIVE: begin

				rx_bus_internal.data_valid		<= 1;

				//Pass the data along
				if(rx_phase)
					rx_bus_internal.data	<= rx_mac_bus_ff.data;
				else
					rx_bus_internal.data	<= { rx_data_ff, rx_mac_bus_ff.data[127:64] };

				//Abort if we see an error
				if(err_found)
					rx_bus_internal.drop		<= 1;

				//If no control characters, we have 16 bytes of frame data.
				else if(!ctl_left && !ctl_right)
					rx_bus_internal.bytes_valid	<= 16;

				//Control character. Frame is probably ending.
				//Could be at any offset, and we could be at two phases, so lots of fun possibilities here!
				else if(rx_phase) begin

					rx_bus_internal.commit			<= 1;

					//Ended immediately?
					if(end_found[15])
						rx_bus_internal.data_valid		<= 0;

					//No, we have frame data
					else begin

						for(integer i=0; i<15; i++) begin
							if(end_found[i])
								rx_bus_internal.bytes_valid		<= 15 - i;
						end

						//If starting a frame in the right half, prepare to handle that
						if(start_right) begin
							rx_phase	<= 1;
							rx_state	<= RX_STATE_FIRST;
						end

						//otherwise no action needed
						else
							rx_state	<= RX_STATE_IDLE;

					end

				end

				else begin

					//If ending in the left half, we're done this cycle
					if(end_found[15:8]) begin

						rx_bus_internal.commit					<= 1;

						for(integer i=8; i<16; i++) begin
							if(end_found[i])
								rx_bus_internal.bytes_valid		<= (15 - i) + 8;
						end

						//If starting a frame in the right half, prepare to handle that
						if(start_right) begin
							rx_phase	<= 1;
							rx_state	<= RX_STATE_FIRST;
						end

						//otherwise no action needed
						else
							rx_state	<= RX_STATE_IDLE;

					end

					//Ending in the right half.
					else begin
						rx_bus_internal.bytes_valid		<= 16;

						//No data in second half. Done this cycle.
						if(end_found[7])
							rx_bus_internal.commit		<= 1;

						//Some data in second half. Done next cycle.
						else
							output_final_word			<= 1;

						rx_last_valid					<= 0;
						for(integer i=0; i<8; i++) begin
							if(end_found[i])
								rx_last_valid			<= (7-i);
						end

						rx_state						<= RX_STATE_IDLE;
					end

				end

			end	//end RX_STATE_ACTIVE

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX: Separate CRC from actual frame data

	EthernetRxBus	rx_bus_internal_ff;

	logic[31:0]		rx_crc_expected	= 0;
	EthernetRxBus	rx_crc_bus;

	always_ff @(posedge rx_clk) begin

		rx_bus_internal_ff		<= rx_bus_internal;

		//Clear flags and push data down the pipe
		rx_crc_bus.start		<= rx_bus_internal_ff.start;
		rx_crc_bus.data			<= rx_bus_internal_ff.data;
		rx_crc_bus.drop			<= rx_bus_internal_ff.drop;
		rx_crc_bus.data_valid	<= 0;
		rx_crc_bus.bytes_valid	<= 0;
		rx_crc_bus.commit		<= 0;

		//We want to push all but the last 4 bytes to the CRC.
		//If we have a full frame this cycle and at least a CRC worth next cycle, push it.
		if(rx_bus_internal_ff.data_valid && (rx_bus_internal_ff.bytes_valid == 16) && !rx_bus_internal_ff.commit &&
			rx_bus_internal.data_valid && (rx_bus_internal.bytes_valid >= 4) ) begin

			rx_crc_bus.data_valid	<= 1;
			rx_crc_bus.bytes_valid	<= 16;

			//If we have *exactly* a CRC next cycle, commit now.
			if(rx_bus_internal.bytes_valid == 4) begin
				rx_crc_bus.commit	<= 1;
				rx_crc_expected		<= rx_bus_internal.data[96 +: 32];
			end

		end

		//We have full data this cycle, but less than a CRC next cycle.
		else if(rx_bus_internal_ff.data_valid && (rx_bus_internal_ff.bytes_valid == 16) && !rx_bus_internal_ff.commit &&
			rx_bus_internal.data_valid && (rx_bus_internal.bytes_valid < 4) ) begin

			rx_crc_bus.data_valid	<= 1;
			rx_crc_bus.bytes_valid	<= 12 + rx_bus_internal.bytes_valid;
			rx_crc_bus.commit		<= 1;

			case(rx_bus_internal.bytes_valid[1:0])
				0: rx_crc_expected	<= rx_bus_internal_ff.data[31:0];
				1: rx_crc_expected	<= { rx_bus_internal_ff.data[23:0], rx_bus_internal.data[127:120] };
				2: rx_crc_expected	<= { rx_bus_internal_ff.data[15:0], rx_bus_internal.data[127:112] };
				3: rx_crc_expected	<= { rx_bus_internal_ff.data[7:0], 	rx_bus_internal.data[127:104] };
			endcase

		end

		//This is the last cycle. Push all but the last 4 bytes.
		else if(rx_bus_internal_ff.commit && (rx_bus_internal_ff.bytes_valid > 4) ) begin

			rx_crc_bus.data_valid	<= 1;
			rx_crc_bus.bytes_valid	<= rx_bus_internal_ff.bytes_valid - 4;
			rx_crc_bus.commit		<= 1;

			case(rx_bus_internal_ff.bytes_valid)
				5: rx_crc_expected	<= rx_bus_internal_ff.data[88 +: 32];
				6: rx_crc_expected	<= rx_bus_internal_ff.data[80 +: 32];
				7: rx_crc_expected	<= rx_bus_internal_ff.data[72 +: 32];
				8: rx_crc_expected	<= rx_bus_internal_ff.data[64 +: 32];
				9: rx_crc_expected	<= rx_bus_internal_ff.data[56 +: 32];
				10: rx_crc_expected	<= rx_bus_internal_ff.data[48 +: 32];
				11: rx_crc_expected	<= rx_bus_internal_ff.data[40 +: 32];
				12: rx_crc_expected	<= rx_bus_internal_ff.data[32 +: 32];
				13: rx_crc_expected	<= rx_bus_internal_ff.data[24 +: 32];
				14: rx_crc_expected	<= rx_bus_internal_ff.data[16 +: 32];
				15: rx_crc_expected	<= rx_bus_internal_ff.data[8 +: 32];
				16: rx_crc_expected	<= rx_bus_internal_ff.data[0 +: 32];

				default: begin
				end
			endcase

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX: CRC calculation

	wire[31:0]	rx_crc_out;

	CRC32_Ethernet_x128 rx_crc(
		.clk(rx_clk),
		.reset(rx_crc_bus.start),
		.update(rx_crc_bus.data_valid),
		.last(rx_crc_bus.commit),
		.din_len(rx_crc_bus.bytes_valid),
		.din(rx_crc_bus.data),
		.crc_flipped(rx_crc_out)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX: Delay data to compensate for CRC pipeline latency

	EthernetRxBus	rx_crc_bus_ff;
	EthernetRxBus	rx_crc_bus_ff2;
	EthernetRxBus	rx_crc_bus_ff3;
	EthernetRxBus	rx_crc_bus_ff4;

	always_ff @(posedge rx_clk) begin
		rx_crc_bus_ff	<= rx_crc_bus;
		rx_crc_bus_ff2	<= rx_crc_bus_ff;
		rx_crc_bus_ff3	<= rx_crc_bus_ff2;
		rx_crc_bus_ff4	<= rx_crc_bus_ff3;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX: Final CRC verification and output

	always_ff @(posedge rx_clk) begin
		rx_bus.start			<= rx_crc_bus_ff4.start;
		rx_bus.data_valid		<= rx_crc_bus_ff4.data_valid;
		rx_bus.bytes_valid		<= rx_crc_bus_ff4.bytes_valid;
		rx_bus.data				<= rx_crc_bus_ff4.data;
		rx_bus.drop				<= rx_crc_bus_ff4.drop;
		rx_bus.commit			<= 0;

		if(rx_crc_bus_ff4.commit) begin
			if(rx_crc_out == rx_crc_expected)
				rx_bus.commit	<= 1;
			else
				rx_bus.drop		<= 1;
		end

	end

endmodule
