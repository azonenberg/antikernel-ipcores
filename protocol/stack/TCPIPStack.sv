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
`include "IPv4Bus.svh"
`include "UDPv4Bus.svh"
`include "TCPv4Bus.svh"

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	The entire TCP/IP stack
 */
module TCPIPStack #(
	parameter LINK_SPEED_IS_10G			= 0,				//true for 10G, false for 1G
	parameter CLK_IPSTACK_HZ			= 125000000,		//used for aging timers
	parameter ARP_CACHE_LINES_PER_WAY	= 128,
	parameter ARP_CACHE_WAYS			= 4
) (

	//Core clock
	input wire					clk_ipstack,

	//Configuration
	input wire IPv4Config		ip_config,
	input wire[47:0]			mac_address,
	input wire					promisc_mode,

	//Link to the MAC
	input wire					mac_rx_clk,
	input wire EthernetRxBus	mac_rx_bus,
	input wire					mac_tx_clk,
	output EthernetTxBus		mac_tx_bus,
	input wire					mac_tx_ready,

	//UDP socket interface
	output UDPv4RxBus			udpv4_rx_bus,
	input UDPv4TxBus			udpv4_tx_bus,

	//TCP socket interface
	output TCPv4RxBus			tcpv4_rx_l4_bus,
	input TCPv4TxBus			tcpv4_tx_l4_bus

	//TODO: performance counters
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC of incoming packets to the IP stack

	EthernetRxBus	cdc_rx_bus;

	EthernetRxClockCrossing rx_cdc(
		.gmii_rxc(mac_rx_clk),
		.sys_clk(clk_ipstack),

		.mac_rx_bus(mac_rx_bus),
		.cdc_rx_bus(cdc_rx_bus),

		.perf_rx_cdc_frames()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Ethernet protocol decoder

	EthernetRxL2Bus	rx_l2_bus;

	Ethernet2TypeDecoder rx_decoder(
		.rx_clk(clk_ipstack),

		.our_mac_address(mac_address),
		.promisc_mode(promisc_mode),

		.mac_rx_bus(cdc_rx_bus),
		.rx_l2_bus(rx_l2_bus),

		.perf()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit buffer

	EthernetTxL2Bus	tx_l2_bus;

	EthernetTransmitElasticBuffer #(
		.LINK_SPEED_IS_10G(LINK_SPEED_IS_10G)
	) tx_buf(
		.our_mac_address(mac_address),

		.tx_l2_clk(clk_ipstack),
		.tx_l2_bus(tx_l2_bus),

		.mac_tx_clk(mac_tx_clk),
		.mac_tx_ready(mac_tx_ready),
		.mac_tx_bus(mac_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Arbitration for the transmit bus

	EthernetTxL2Bus	arp_tx_l2_bus;
	EthernetTxL2Bus	ipv4_tx_arp_bus;

	EthernetTransmitArbiter tx_arbiter(
		.clk(clk_ipstack),

		.ipv4_tx_l2_bus(ipv4_tx_arp_bus),
		.arp_tx_l2_bus(arp_tx_l2_bus),

		.tx_l2_bus(tx_l2_bus),

		.perf()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ARP

	wire		arp_learn_valid;
	wire[31:0]	arp_learn_ip;
	wire[47:0]	arp_learn_mac;

	wire		arp_query_en;
	wire[31:0]	arp_query_ip;

	ARPProtocol arp(
		.clk(clk_ipstack),

		.our_mac_address(mac_address),
		.our_ip_address(ip_config.address),

		.rx_l2_bus(rx_l2_bus),
		.tx_l2_bus(arp_tx_l2_bus),

		.learn_valid(arp_learn_valid),
		.learn_ip(arp_learn_ip),
		.learn_mac(arp_learn_mac),

		.query_en(arp_query_en),
		.query_ip(arp_query_ip)
	);

	EthernetTxArpBus	ipv4_tx_l2_bus;

	ARPManager #(
		.AGE_INTERVAL(CLK_IPSTACK_HZ),
		.NUM_WAYS(ARP_CACHE_WAYS),
		.LINES_PER_WAY(ARP_CACHE_LINES_PER_WAY),
		.MAX_AGE(3600)
	) arp_mgr (
		.clk(clk_ipstack),

		.ipv4_tx_l2_bus(ipv4_tx_l2_bus),
		.ipv4_tx_arp_bus(ipv4_tx_arp_bus),

		.learn_en(arp_learn_valid),
		.learn_ip(arp_learn_ip),
		.learn_mac(arp_learn_mac),

		.query_en(arp_query_en),
		.query_ip(arp_query_ip)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Layer 3 IPv4

	IPv4RxBus	ipv4_rx_l3_bus;
	IPv4TxBus	ipv4_tx_l3_bus;

	IPv4Protocol ipv4(
		.clk(clk_ipstack),

		.ip_config(ip_config),

		.rx_l2_bus(rx_l2_bus),
		.tx_l2_bus(ipv4_tx_l2_bus),

		//TODO: Arbiter between multiple layer-4 protocols
		//For now, hard-wire ICMP to the TX bus
		.rx_l3_bus(ipv4_rx_l3_bus),
		.tx_l3_bus(ipv4_tx_l3_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IPv4 transmit arbiter

	IPv4TxBus	icmp_ipv4_tx_l3_bus;
	IPv4TxBus	udp_ipv4_tx_l3_bus;
	IPv4TxBus	tcp_ipv4_tx_l3_bus;

	IPv4TransmitArbiter ip_arbiter(
		.clk(clk_ipstack),

		.icmp_bus(icmp_ipv4_tx_l3_bus),
		.tcp_bus(tcp_ipv4_tx_l3_bus),
		.udp_bus(udp_ipv4_tx_l3_bus),

		.ipv4_bus(ipv4_tx_l3_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Layer 4 UDP (for IPv4)

	UDPProtocol udp_ipv4(
		.clk(clk_ipstack),

		.rx_l3_bus(ipv4_rx_l3_bus),
		.rx_l4_bus(udpv4_rx_bus),

		.tx_l3_bus(udp_ipv4_tx_l3_bus),
		.tx_l4_bus(udpv4_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Layer 4 ICMP (for IPv4)

	ICMPv4Protocol icmp_ipv4(
		.clk(clk_ipstack),

		.rx_l3_bus(ipv4_rx_l3_bus),
		.tx_l3_bus(icmp_ipv4_tx_l3_bus),

		.perf()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Layer 4 TCP (for IPv4)

	TCPProtocol tcp_ipv4(
		.clk(clk_ipstack),

		.rx_l3_bus(ipv4_rx_l3_bus),
		.rx_l4_bus(tcpv4_rx_l4_bus),

		.tx_l3_bus(tcp_ipv4_tx_l3_bus),
		.tx_l4_bus(tcpv4_tx_l4_bus)
	);

endmodule
