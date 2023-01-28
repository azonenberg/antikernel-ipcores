`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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
`include "TCPIPPerformanceCounters.svh"

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	The entire TCP/IP stack
 */
module TCPIPStack #(
	parameter LINK_SPEED_IS_10G			= 0,				//true for 10G, false for 1G
	parameter CLK_IPSTACK_HZ			= 125000000,		//used for aging timers
	parameter ARP_CACHE_LINES_PER_WAY	= 128,
	parameter ARP_CACHE_WAYS			= 4,
	parameter TX_PACKET_DEPTH			= 4096,
	parameter TX_HEADER_DEPTH			= 256,
	parameter TCP_RAM_DEPTH				= 8192,
	localparam TCP_ADDR_BITS			= $clog2(TCP_RAM_DEPTH),
	parameter TCP_ENABLE				= 1,
	parameter UDP_ENABLE				= 1
) (

	//Core clock
	input wire						clk_ipstack,

	//Configuration
	input wire						link_up,					//mac_rx_clk domain
	input wire IPv4Config			ip_config,
	input wire[47:0]				mac_address,				//clk_ipstack domain
	input wire						promisc_mode,
	input wire						config_update,				//assert when ip_config/mac_addr/promisc_mode are changed

	//Link to the MAC
	input wire						mac_rx_clk,
	input wire EthernetRxBus		mac_rx_bus,
	input wire						mac_tx_clk,
	output EthernetTxBus			mac_tx_bus,
	input wire						mac_tx_ready,

	//UDP socket interface
	output UDPv4RxBus				udpv4_rx_bus,
	input UDPv4TxBus				udpv4_tx_bus,

	//TCP socket interface
	output TCPv4RxBus				tcpv4_rx_bus,
	input TCPv4TxBus				tcpv4_tx_bus,
	input wire						tcp_port_open_en,
	input wire						tcp_port_close_en,
	input wire portnum_t			tcp_port_num,

	//RAM interface for TCP transmit buffers
	input wire						ram_ready,
	output wire						ram_wr_en,
	output wire[TCP_ADDR_BITS-1:0]	ram_wr_addr,
	output wire[511:0]				ram_wr_data,
	input wire						ram_rd_en,
	input wire[TCP_ADDR_BITS-1:0]	ram_rd_addr,
	input wire[511:0]				ram_rd_data,
	input wire						ram_rd_valid,

	//Performance counters
	output TCPIPPerformanceCounters	perf
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize link-up state into the IP stack clock domain

	wire	link_up_sync;

	ThreeStageSynchronizer sync_link_up(
		.clk_in(mac_rx_clk),
		.din(link_up),
		.clk_out(clk_ipstack),
		.dout(link_up_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize MAC address to the TX clock domain for sending frames

	wire[47:0]	mac_addr_txclk;

	RegisterSynchronizer #(
		.WIDTH(48)
	) mac_sync (
		.clk_a(clk_ipstack),
		.en_a(config_update),
		.ack_a(),
		.reg_a(mac_address),
		.clk_b(mac_tx_clk),
		.updated_b(),
		.reg_b(mac_addr_txclk),
		.reset_b(1'b0)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC of incoming packets to the IP stack

	EthernetRxBus	cdc_rx_bus;

	EthernetRxClockCrossing rx_cdc(
		.gmii_rxc(mac_rx_clk),
		.sys_clk(clk_ipstack),

		.mac_rx_bus(mac_rx_bus),
		.cdc_rx_bus(cdc_rx_bus),

		.perf_rx_cdc_frames(perf.rx_cdc_frames)
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

		.perf(perf.rx_decoder)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit buffer

	EthernetTxL2Bus	tx_l2_bus;

	EthernetTransmitElasticBuffer #(
		.LINK_SPEED_IS_10G(LINK_SPEED_IS_10G),
		.HEADER_DEPTH(TX_HEADER_DEPTH),
		.PACKET_DEPTH(TX_PACKET_DEPTH)
	) tx_buf (
		.our_mac_address(mac_addr_txclk),

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

		.perf(perf.tx_arbiter)
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

		.link_up(link_up_sync),
		.config_update(config_update),
		.ip_config(ip_config),

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

	wire		ipv4_tx_busy;

	IPv4Protocol ipv4(
		.clk(clk_ipstack),

		.ip_config(ip_config),

		.rx_l2_bus(rx_l2_bus),
		.tx_l2_bus(ipv4_tx_l2_bus),

		.rx_l3_bus(ipv4_rx_l3_bus),
		.tx_l3_bus(ipv4_tx_l3_bus),

		.tx_busy(ipv4_tx_busy)
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

		.ipv4_bus(ipv4_tx_l3_bus),

		.tx_busy(ipv4_tx_busy)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Layer 4 UDP (for IPv4)

	if(UDP_ENABLE) begin

		UDPProtocol udp_ipv4(
			.clk(clk_ipstack),

			.rx_l3_bus(ipv4_rx_l3_bus),
			.rx_l4_bus(udpv4_rx_bus),

			.tx_l3_bus(udp_ipv4_tx_l3_bus),
			.tx_l4_bus(udpv4_tx_bus)
		);
	end

	else begin
		assign udpv4_rx_bus = 0;
		assign udp_ipv4_tx_l3_bus = 0;
	end

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

	if(TCP_ENABLE) begin
		TCPProtocol #(
			.AGE_INTERVAL(CLK_IPSTACK_HZ)
		) tcp_ipv4 (
			.clk(clk_ipstack),

			.ip_config(ip_config),

			.rx_l3_bus(ipv4_rx_l3_bus),
			.rx_l4_bus(tcpv4_rx_bus),

			.tx_l3_bus(tcp_ipv4_tx_l3_bus),
			.tx_l4_bus(tcpv4_tx_bus),

			.port_open_en(tcp_port_open_en),
			.port_close_en(tcp_port_close_en),
			.port_num(tcp_port_num)
		);

		TCPTransmitBufferManager #(
			.RAM_DEPTH(TCP_RAM_DEPTH)
		) tcp_bufmgr (
			.clk(clk_ipstack),

			.ram_ready(ram_ready),
			.ram_wr_en(ram_wr_en),
			.ram_wr_addr(ram_wr_addr),
			.ram_wr_data(ram_wr_data),
			.ram_rd_en(ram_rd_en),
			.ram_rd_addr(ram_rd_addr),
			.ram_rd_data(ram_rd_data),
			.ram_rd_valid(ram_rd_valid)
		);
	end

	else begin
		assign tcpv4_rx_bus = 0;
		assign tcp_ipv4_tx_l3_bus = 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Performance counters

endmodule
