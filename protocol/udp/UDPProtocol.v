`timescale 1ns / 1ps
`default_nettype none
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

module UDPProtocol(

	//Clocks
	input wire			clk,

	//Incoming data bus from IP stack
	//TODO: make this parameterizable for IPv4/IPv6, for now we only do v4
	input wire			rx_l3_start,
	input wire[15:0]	rx_l3_payload_len,
	input wire			rx_l3_protocol_is_udp,
	input wire[31:0]	rx_l3_src_ip,
	input wire[31:0]	rx_l3_dst_ip,
	input wire			rx_l3_data_valid,
	input wire[2:0]		rx_l3_bytes_valid,
	input wire[31:0]	rx_l3_data,
	input wire			rx_l3_commit,
	input wire			rx_l3_drop,
	input wire			rx_l3_headers_valid,
	input wire[15:0]	rx_l3_pseudo_header_csum,

	//Outbound bus to applications
	output reg			rx_l4_start			= 0,
	output reg			rx_l4_headers_valid	= 0,
	output reg[15:0]	rx_l4_src_port		= 0,
	output reg[15:0]	rx_l4_dst_port		= 0,
	output reg			rx_l4_data_valid	= 0,
	output reg[2:0]		rx_l4_bytes_valid	= 0,
	output reg[31:0]	rx_l4_data			= 0,
	output reg			rx_l4_commit		= 0,
	output reg			rx_l4_drop			= 0,
	output reg[15:0]	rx_l4_payload_len	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX datapath

	wire[15:0]	rx_checksum_expected;

	InternetChecksum32bit rx_csum(
		.clk(clk),
		.load(rx_l3_headers_valid),
		.reset(1'b0),
		.process(rx_l3_data_valid),
		.din(rx_l3_data_valid ? rx_l3_data : rx_l3_pseudo_header_csum),
		.sumout(),
		.csumout(rx_checksum_expected)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX datapath

	localparam RX_STATE_IDLE		= 4'h0;
	localparam RX_STATE_HEADER_0	= 4'h1;
	localparam RX_STATE_HEADER_1	= 4'h2;
	localparam RX_STATE_HEADER_2	= 4'h3;
	localparam RX_STATE_BODY		= 4'h4;
	localparam RX_STATE_PADDING		= 4'h5;
	localparam RX_STATE_CHECKSUM	= 4'h6;

	reg[3:0]	rx_state			= RX_STATE_IDLE;
	reg[15:0]	rx_checksum			= 0;

	reg[15:0]	rx_l4_bytes_left	= 0;

	always @(posedge clk) begin

		//Forward flags from IP stack
		rx_l4_drop			<= rx_l3_drop;

		//Clear flags
		rx_l4_start			<= 0;
		rx_l4_data_valid	<= 0;
		rx_l4_bytes_valid	<= 0;
		rx_l4_commit		<= 0;
		rx_l4_headers_valid	<= 0;

		case(rx_state)

			//Start when we get a new packet
			RX_STATE_IDLE: begin

				if(rx_l3_start) begin
					rx_l4_start			<= 1;
					rx_state			<= RX_STATE_HEADER_0;
				end

			end	//end RX_STATE_IDLE

			//Wait until layer 3 says it's actually a UDP packet
			//Drop anything too small for UDP headers, or not UDP
			RX_STATE_HEADER_0: begin
				if(rx_l3_headers_valid) begin
					if( (rx_l3_payload_len < 8) || !rx_l3_protocol_is_udp ) begin
						rx_l4_drop			<= 1;
						rx_state			<= RX_STATE_IDLE;
					end
					else
						rx_state			<= RX_STATE_HEADER_1;
				end
			end	//end RX_STATE_HEADER_0

			//Port header
			RX_STATE_HEADER_1: begin
				if(rx_l3_data_valid) begin

					//Drop truncated packets
					if(rx_l3_bytes_valid != 4) begin
						rx_l4_drop	<= 1;
						rx_state	<= RX_STATE_IDLE;
					end

					else begin
						rx_l4_src_port	<= rx_l3_data[31:16];
						rx_l4_dst_port	<= rx_l3_data[15:0];
						rx_state		<= RX_STATE_HEADER_2;
					end

				end
			end	//end RX_STATE_HEADER_1

			RX_STATE_HEADER_2: begin

				if(rx_l3_data_valid) begin

					//Drop truncated packets
					if(rx_l3_bytes_valid != 4) begin
						rx_l4_drop	<= 1;
						rx_state	<= RX_STATE_IDLE;
					end

					//Drop anything with an invalid size
					else if(rx_l3_data[31:16] < 8) begin
						rx_l4_drop	<= 1;
						rx_state	<= RX_STATE_IDLE;
					end

					else begin
						rx_checksum			<= rx_l3_data[15:0];
						rx_l4_payload_len	<= rx_l3_data[31:16] - 16'h8;
						rx_l4_bytes_left	<= rx_l3_data[31:16] - 16'h8;	//start off at payload length
						rx_l4_headers_valid	<= 1;
						rx_state			<= RX_STATE_BODY;
					end

				end

			end	//end RX_STATE_HEADER_2

			RX_STATE_BODY: begin

				//Forward data
				if(rx_l3_data_valid) begin

					rx_l4_data_valid		<= 1;
					rx_l4_data				<= rx_l3_data;

					//Data plus padding? Send only the valid data
					if(rx_l4_bytes_left < rx_l3_bytes_valid) begin
						rx_l4_bytes_valid	<= rx_l4_bytes_left;
						rx_state			<= RX_STATE_PADDING;
					end

					//More data left? Send all of it
					else
						rx_l4_bytes_valid	<= rx_l4_bytes_left;

					//Update byte counter
					rx_l4_bytes_left		<= rx_l4_bytes_left - rx_l3_data_valid;
				end

				//Verify checksum etc
				if(rx_l3_commit)
					rx_state				<= RX_STATE_CHECKSUM;

			end	//end RX_STATE_BODY

			RX_STATE_PADDING: begin

				//Verify checksum etc
				if(rx_l3_commit)
					rx_state				<= RX_STATE_CHECKSUM;

			end	//end RX_STATE_PADDING

			RX_STATE_CHECKSUM: begin
				if(rx_checksum_expected == 16'h0000) begin
					rx_l4_commit	<= 1;
					rx_state		<= RX_STATE_IDLE;
				end
				else begin
					rx_l4_drop		<= 1;
					rx_state		<= RX_STATE_IDLE;
				end
			end	//end RX_STATE_CHECKSUM

		endcase

		//If we get a drop request from the IP stack, abort everything
		if(rx_l3_drop)
			rx_state	<= RX_STATE_IDLE;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LA for debugging

	/*
	wire	trig_out;
	reg		trig_out_ack	= 0;

	always @(posedge clk) begin
		trig_out_ack	<= trig_out;
	end

	ila_0 ila(
		.clk(clk),

		.probe0(rx_l3_start),
		.probe1(rx_l3_payload_len),
		.probe2(rx_l3_protocol_is_udp),
		.probe3(rx_l3_src_ip),
		.probe4(rx_l3_dst_ip),
		.probe5(rx_l3_data_valid),
		.probe6(rx_l3_bytes_valid),
		.probe7(rx_l3_data),
		.probe8(rx_l3_commit),
		.probe9(rx_l3_drop),
		.probe10(rx_l3_headers_valid),

		.probe11(rx_l4_start),
		.probe12(rx_l4_headers_valid),
		.probe13(rx_l4_src_port),
		.probe14(rx_l4_dst_port),
		.probe15(rx_l4_data_valid),
		.probe16(rx_l4_bytes_valid),
		.probe17(rx_l4_data),
		.probe18(rx_l4_commit),
		.probe19(rx_l4_drop),
		.probe20(rx_state),
		.probe21(rx_l4_payload_len),
		.probe22(rx_l4_bytes_left),
		.probe23(rx_checksum_expected),
		.probe24(rx_checksum),
		.probe25(rx_l3_pseudo_header_csum),
		.probe26(1'b0),

		.trig_out(trig_out),
		.trig_out_ack(trig_out_ack)
	);
	*/

endmodule
