`timescale 1ns/1ps
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

`include "EthernetBus.svh"
`include "IPv4Bus.svh"

/**
	@brief IPv4 protocol stack
 */
module IPv4Protocol(
	input wire					rx_clk,
	input wire EthernetRxBus	rx_bus,
	input wire EthernetHeaders	rx_headers,

	input wire IPv4Config 		ip_config,

	output IPv4Headers			rx_l3_headers	= 0,
	output EthernetRxBus		rx_l3_bus		= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side

	enum logic[3:0]
	{
		RX_STATE_IDLE,
		RX_STATE_SECOND,
		RX_STATE_THIRD,
		RX_STATE_HANG
	} rx_state = RX_STATE_IDLE;

	logic[15:0]	rx_checksum_expected	= 0;

	logic[17:0]	rx_header_checksum_stage1[1:0];
	logic[16:0]	rx_header_checksum_stage2[1:0];
	logic[16:0]	rx_header_checksum_stage3;
	logic[15:0]	rx_header_checksum_final			= 0;

	EthernetRxBus	rx_bus_ff;

	always_ff @(posedge rx_clk) begin

		rx_bus_ff	<= rx_bus;

		//DEBUG
		rx_l3_bus.data					<= rx_header_checksum_final;

		case(rx_state)

			RX_STATE_IDLE: begin

				//Look for new packets
				if(rx_bus.start) begin

					//Extract fields up here to avoid lots of nested conditionals on the critical path.
					rx_l3_headers.payload_len	<= rx_bus.data[111:96] - 'd20;
					rx_l3_headers.protocol		<= ipproto_t'(rx_bus.data[55:48]);
					rx_l3_headers.src_ip		<= rx_bus.data[31:0];
					rx_checksum_expected		<= ~rx_bus.data[47:32];

					//Calculate first stage of the checksum
					rx_header_checksum_stage1[0]	<=
					{
						1'h0,
						rx_bus.data[78],
						!rx_bus.data[78],
						6'h05,
						rx_bus.data[119:112]
					} + rx_bus.data[15:0] + rx_bus.data[95:80];
					rx_header_checksum_stage1[1]	<= rx_bus.data[111:96] + rx_bus.data[63:48] + rx_bus.data[31:16];

					//Match valid IPv4 packets. Silently discard anything we don't care about.
					if(
						rx_bus.data_valid && (rx_bus.bytes_valid == 16) &&			//Full sized word
						(rx_headers.ethertype == ETHERTYPE_IPV4) &&					//IPv4 ethertype
						(rx_bus.data[127:124] == 4'h4) &&							//IPv4
						(rx_bus.data[123:120] == 5) &&								//Header length 20 bytes (no options).
																					//Discard anything with options.
						//119:114 is DSCP, ignore
						//113:112 is ECN, ignore
						(rx_bus.data[111:96] >= 20) &&								//valid length
						//95:80 is identification, ignore
						!rx_bus.data[79] &&											//evil bit (reserved) not set
						//78 is DF bit, ignore
						!rx_bus.data[77] &&											//more fragments bit not set
																					//(we don't support fragmentation)
						(rx_bus.data[76:64] == 0)									//fragment offset is zero
						//63:56 is TTL, ignore
						) begin

						rx_state	<= RX_STATE_SECOND;

					end

				end

			end	//end RX_STATE_IDLE

			//Second word of an IPv4 packet
			RX_STATE_SECOND: begin

				//As a minimum, any IPv4 packet needs to have full headers. So bail if there's not enough payload.
				if(!rx_bus.data_valid || (rx_bus.bytes_valid < 4) ) begin
				end

				//Valid traffic
				else begin

					//Extract the last header field
					rx_l3_headers.dst_ip			<= rx_bus.data[127:96];

					//Second stage of header checksum calculation
					rx_header_checksum_stage2[0]	= rx_header_checksum_stage1[0][15:0] +
														rx_header_checksum_stage1[1][15:0];
					rx_header_checksum_stage2[1]	= rx_bus.data[127:112] + rx_bus.data[111:96] +
														rx_header_checksum_stage1[0][17:16] +
														rx_header_checksum_stage1[1][17:16];

					//Move on without actually sending any traffic to the upper layer protocol yet.
					rx_state						<= RX_STATE_THIRD;

				end

			end	//end RX_STATE_SECOND

			RX_STATE_THIRD: begin

				rx_header_checksum_stage3	= rx_header_checksum_stage2[0][15:0] +
													rx_header_checksum_stage2[1][15:0] +
													rx_header_checksum_stage2[0][16] +
													rx_header_checksum_stage2[1][16];
				rx_header_checksum_final	<= rx_header_checksum_stage3[15:0] + rx_header_checksum_stage3[16];

				rx_state	<= RX_STATE_HANG;

			end	//end RX_STATE_THIRD

		endcase

		//Abort if we're dropping stuff
		if(rx_bus.drop) begin
			rx_l3_bus.drop		<= 1;
			rx_l3_bus.commit	<= 0;
			rx_state			<= RX_STATE_IDLE;
		end

	end

endmodule
