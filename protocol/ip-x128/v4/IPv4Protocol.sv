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
		RX_STATE_BODY,
		RX_STATE_HANG
	} rx_state = RX_STATE_IDLE;

	logic[15:0]	rx_checksum_expected	= 0;

	logic[17:0]	rx_header_checksum_stage1[1:0];
	logic[16:0]	rx_header_checksum_stage2[1:0];

	//Finish calculating the header checksum
	wire[16:0] rx_header_checksum_stage3	= rx_header_checksum_stage2[0][15:0] +
											rx_header_checksum_stage2[1][15:0] +
											rx_header_checksum_stage2[0][16] +
											rx_header_checksum_stage2[1][16];
	wire[15:0] rx_header_checksum_final	= rx_header_checksum_stage3[15:0] + rx_header_checksum_stage3[16];

	EthernetRxBus	rx_bus_ff;

	wire[15:0]	first_payload_len	= rx_bus_ff.bytes_valid - 4;
	wire[15:0]	second_payload_len	= first_payload_len + rx_bus.bytes_valid;

	logic[15:0]		rx_bytes_left	= 0;

	always_ff @(posedge rx_clk) begin

		rx_bus_ff	<= rx_bus;
		rx_l3_bus	<= 0;

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
					rx_header_checksum_stage2[1]	<= rx_bus.data[127:112] + rx_bus.data[111:96] +
														rx_header_checksum_stage1[0][17:16] +
														rx_header_checksum_stage1[1][17:16];

					//Move on without actually sending any traffic to the upper layer protocol yet.
					rx_state						<= RX_STATE_THIRD;

				end

			end	//end RX_STATE_SECOND

			RX_STATE_THIRD: begin

				//TODO: Pseudo-header checksum

				//Assume we're starting a new frame (might cancel later if header checksum is bad)
				rx_l3_bus.start			<= 1;
				rx_l3_bus.data_valid	<= 1;

				//If last cycle was the end of the packet, it was tiny.
				//Send whatever partial data there was (after trimming off the dest IP address)
				if(rx_bus_ff.commit) begin

					//Trim off padding if we have a tiny payload
					if(rx_l3_headers.payload_len < first_payload_len)
						rx_l3_bus.bytes_valid	<= rx_l3_headers.payload_len;
					else
						rx_l3_bus.bytes_valid	<= first_payload_len;

					rx_l3_bus.data			<= { rx_bus_ff.data[95:0], 32'h0 };

					//Verify checksum
					if(rx_header_checksum_final == rx_checksum_expected)
						rx_l3_bus.commit	<= 1;
					else
						rx_l3_bus.start		<= 0;

					//Done
					rx_state			<= RX_STATE_IDLE;

				end

				//If this cycle is the end, but we have <= 4 bytes, it's still a single cycle payload at the output,
				//but we have to combine data from two input cycles.
				else if(rx_bus.commit && (rx_bus.bytes_valid <= 4) ) begin

					//Trim off padding if we have a tiny payload
					if(rx_l3_headers.payload_len < second_payload_len)
						rx_l3_bus.bytes_valid	<= rx_l3_headers.payload_len;
					else
						rx_l3_bus.bytes_valid	<= second_payload_len;
					rx_l3_bus.data			<= { rx_bus_ff.data[95:0], rx_bus.data[127:96] };

					//Verify checksum
					if(rx_header_checksum_final == rx_checksum_expected)
						rx_l3_bus.commit	<= 1;
					else
						rx_l3_bus.start		<= 0;

					//Done
					rx_state			<= RX_STATE_IDLE;

				end

				//We have a full 16 bytes of data this cycle;
				else begin

					//Trim off padding if we have a tiny payload
					if(rx_l3_headers.payload_len < 16) begin
						rx_l3_bus.bytes_valid	<= rx_l3_headers.payload_len;
						rx_l3_bus.commit		<= 1;
						rx_state				<= RX_STATE_IDLE;
					end

					//Full length cycle
					else begin
						rx_l3_bus.bytes_valid	<= 16;
						rx_bytes_left			<= rx_l3_headers.payload_len - 16;
						rx_state				<= RX_STATE_BODY;
					end

					rx_l3_bus.data			<= { rx_bus_ff.data[95:0], rx_bus.data[127:96] };

					//Verify checksum
					if(rx_header_checksum_final != rx_checksum_expected) begin
						rx_l3_bus.start		<= 0;
						rx_state			<= RX_STATE_IDLE;
					end

				end

			end	//end RX_STATE_THIRD

			RX_STATE_BODY: begin
				//TODO
			end	//end RX_STATE_BODY

		endcase

		//Abort if we're dropping stuff
		if(rx_bus.drop) begin
			rx_l3_bus.drop		<= 1;
			rx_l3_bus.commit	<= 0;
			rx_state			<= RX_STATE_IDLE;
		end

	end

endmodule
