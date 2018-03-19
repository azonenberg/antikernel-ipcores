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
	input wire			rx_l3_headers_valid
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX datapath

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LA for debugging

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

		.trig_out(trig_out),
		.trig_out_ack(trig_out_ack)
	);

endmodule
