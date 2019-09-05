`timescale 1ns / 1ps
`default_nettype none
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

`include "EthernetBus.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief Ethernet-II type decoding

	Supports jumbo frames up to 9000 bytes and 802.1q VLAN tag decoding.

	NOTE: unlike the MAC, it is possible for rx_l2_bus.data_valid to be asserted simultaneous with rx_l2_bus.commit
 */
module Ethernet2TypeDecoder(

	//Clocks
	input wire					rx_clk,

	//Incoming frames from the MAC
	input wire EthernetRxBus	mac_rx_bus,

	//Destination MAC filtering
	input wire[47:0]			our_mac_address,
	input wire					promisc_mode,

	//Outbound data
	output EthernetRxL2Bus		rx_l2_bus	= {$bits(EthernetRxL2Bus){1'b0}},

	//Performance counters
	output EthernetDecoderPerformanceCounters	perf	= { 64'h0, 64'h0, 64'h0, 64'h0 }
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Performance counters

	always_ff @(posedge rx_clk) begin
		if(rx_l2_bus.commit) begin
			perf.rx_total		<= perf.rx_total + 1'h1;

			if(rx_l2_bus.ethertype_is_ipv4)
				perf.rx_ipv4	<= perf.rx_ipv4 + 1'h1;
			if(rx_l2_bus.ethertype_is_ipv6)
				perf.rx_ipv6	<= perf.rx_ipv6 + 1'h1;
			if(rx_l2_bus.ethertype_is_arp)
				perf.rx_arp		<= perf.rx_arp + 1'h1;

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet-II frame decoding

	`include "Ethertypes.svh"

	logic				rx_active			= 0;
	logic[13:0]			rx_count			= 0;

	logic[15:0]			rx_temp_buf			= 0;
	logic[1:0]			rx_temp_valid		= 0;

	always_ff @(posedge rx_clk) begin

		rx_l2_bus.data_valid	<= 0;
		rx_temp_valid			<= 0;
		rx_l2_bus.headers_valid	<= 0;

		//Forward flags
		rx_l2_bus.start		<= mac_rx_bus.start;
		rx_l2_bus.commit	<= mac_rx_bus.commit;
		rx_l2_bus.drop		<= mac_rx_bus.drop;

		//Save the low half of the incoming data word so we can use it next clock
		//(fixing phase alignment so frame body is on a 32-bit boundary)
		rx_temp_buf			<= mac_rx_bus.data[15:0];

		//If we get a drop request, abort and stop whatever we're doing immediately.
		//The frame is corrupted, no point in wasting any more time on it.
		if(mac_rx_bus.drop)
			rx_active		<= 0;

		//Drop excessively long jumbo frames
		else if(rx_count > 'd9038) begin
			rx_l2_bus.drop	<= 1;
			rx_active		<= 0;
		end

		//Process frame data
		else if(rx_active) begin

			//Send the last data words at the end of the packet
			//(no new data can arrive simultaneous with commit)
			if(mac_rx_bus.commit) begin
				rx_active				<= 0;

				rx_temp_valid			<= 0;
				rx_l2_bus.bytes_valid	<= rx_temp_valid;

				rx_l2_bus.data_valid	<= (rx_temp_valid != 0);

				if(rx_temp_valid == 1)
					rx_l2_bus.data		<= { rx_temp_buf[15:8], 24'h0 };
				else if(rx_temp_valid == 2)
					rx_l2_bus.data		<= { rx_temp_buf[15:0], 16'h0 };
			end

			//New data word?
			else if(mac_rx_bus.data_valid) begin

				rx_count	<= rx_count + 1'h1;

				//Read dst/src mac addresses (always same location in frame)
				//Don't care about mac_rx_bus.bytes_valid since the packet can never be this short
				//(if it ends early we'll get a drop request from the MAC and skip it)
				if(rx_count == 0)
					rx_l2_bus.dst_mac[47:16]		<= mac_rx_bus.data;
				else if(rx_count == 1) begin
					rx_l2_bus.dst_mac[15:0]			<= mac_rx_bus.data[31:16];
					rx_l2_bus.src_mac[47:32]		<= mac_rx_bus.data[15:0];
				end
				else if(rx_count == 2)
					rx_l2_bus.src_mac[31:0]			<= mac_rx_bus.data;

				//Next block is either the ethertype plus two bytes of data, or an 802.1q tag
				else if(rx_count == 3) begin

					//If ethertype is 802.1q, parse the VLAN tag
					if(mac_rx_bus.data[31:16] == ETHERTYPE_DOT1Q) begin
						rx_l2_bus.has_vlan_tag	<= 1;

						rx_l2_bus.qos_pri		<= mac_rx_bus.data[15:13];
						rx_l2_bus.drop_eligible	<= mac_rx_bus.data[12];
						rx_l2_bus.vlan_id		<= mac_rx_bus.data[11:0];
					end

					//Nope, insert a dummy vlan tag with default values and store the ethertype
					else begin
						rx_l2_bus.has_vlan_tag	<= 0;

						rx_l2_bus.qos_pri		<= 0;
						rx_l2_bus.drop_eligible	<= 1;
						rx_l2_bus.vlan_id		<= 1;

						rx_l2_bus.ethertype			<= ethertype_t'(mac_rx_bus.data[31:16]);
						rx_l2_bus.ethertype_is_ipv4	<= (mac_rx_bus.data[31:16] == ETHERTYPE_IPV4);
						rx_l2_bus.ethertype_is_ipv6	<= (mac_rx_bus.data[31:16] == ETHERTYPE_IPV6);
						rx_l2_bus.ethertype_is_arp	<= (mac_rx_bus.data[31:16] == ETHERTYPE_ARP);
						rx_l2_bus.headers_valid		<= 1;
					end

					//If not in promiscuous mode, and we get a unicast that's not for us, drop it
					//(accept all multicasts)
					if(!promisc_mode && !rx_l2_bus.dst_mac[40] && (rx_l2_bus.dst_mac != our_mac_address) ) begin
						rx_l2_bus.drop			<= 1;
						rx_active				<= 0;
					end

				end

				//If we have a 802.1q tag, the NEXT cycle has the ethertype
				else if( (rx_count == 4) && (rx_l2_bus.has_vlan_tag) ) begin
					rx_l2_bus.ethertype			<= ethertype_t'(mac_rx_bus.data[31:16]);
					rx_l2_bus.ethertype_is_ipv4	<= (mac_rx_bus.data[31:16] == ETHERTYPE_IPV4);
					rx_l2_bus.ethertype_is_ipv6	<= (mac_rx_bus.data[31:16] == ETHERTYPE_IPV6);
					rx_l2_bus.ethertype_is_arp	<= (mac_rx_bus.data[31:16] == ETHERTYPE_ARP);
					rx_l2_bus.headers_valid		<= 1;
				end

				//Nope, just normal frame payload.
				//Go ahead and forward it
				else begin
					rx_l2_bus.data_valid		<= 1;

					case(mac_rx_bus.bytes_valid)

						1: begin
							rx_l2_bus.bytes_valid	<= 3;
							rx_l2_bus.data			<= { rx_temp_buf, mac_rx_bus.data[31:24], 8'h0 };
							rx_temp_valid			<= 0;
						end

						2: begin
							rx_l2_bus.bytes_valid	<= 4;
							rx_l2_bus.data			<= { rx_temp_buf, mac_rx_bus.data[31:16] };
							rx_temp_valid			<= 0;
						end

						3: begin
							rx_l2_bus.bytes_valid	<= 4;
							rx_l2_bus.data			<= { rx_temp_buf, mac_rx_bus.data[31:16] };
							rx_temp_valid			<= 1;
						end

						4: begin
							rx_l2_bus.bytes_valid	<= 4;
							rx_l2_bus.data			<= { rx_temp_buf, mac_rx_bus.data[31:16] };
							rx_temp_valid			<= 2;
						end

					endcase

				end

			end

			//Idle period between message data blocks, do nothing
			else begin
			end

		end

		//Start a new frame
		else if(mac_rx_bus.start) begin
			rx_count				<= 0;
			rx_active				<= 1;
			rx_l2_bus.bytes_valid	<= 0;
			rx_l2_bus.data			<= 0;

			rx_l2_bus.headers_valid	<= 0;
		end

	end

endmodule
