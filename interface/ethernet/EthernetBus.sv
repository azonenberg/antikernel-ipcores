/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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
	@file EthernetBus.svh
	@author Andrew D. Zonenberg
	@brief Structure definitions for layer-2 Ethernet buses

	Conventions
		start is asserted before, not simultaneous with, first assertion of data_valid
		bytes_valid is always 4 until last word in the packet, at which point it may take any value
		commit is asserted after, not simultaneous with, last assertion of data_valid
 */

package EthernetBus;

`include "Ethertypes.svh"
`include "XGMII_CtlChars.svh"

typedef logic[11:0]	vlan_t;
typedef logic[47:0] macaddr_t;

//Inbound packet data bus
typedef struct packed
{
	logic		start;			//asserted for one cycle before a frame starts
	logic		data_valid;		//asserted when data is ready to be processed
	logic[2:0]	bytes_valid;	//when data_valid is set, indicated number of valid bytes in data
								//Valid bits are left aligned in data
								//1 = 31:24, 2 = 31:16, 3 = 31:8, 4 = 31:0
	logic[31:0]	data;			//actual packet content

	logic		commit;			//asserted for one cycle at end of packet if checksum was good
	logic		drop;			//asserted for one cycle to indicate packet is invalid and should be discarded
} EthernetRxBus;

//Bus from elastic buffer to MAC
typedef struct packed
{
	logic		start;			//asserted for one cycle before a frame starts
	logic		data_valid;		//asserted when data is ready to be processed
	logic[31:0]	data;			//actual packet content
								//only 7:0 meaningful in 1G mode
	logic[2:0]	bytes_valid;	//only meaningful in 10G mode
} EthernetTxBus;

//Bus from IP to ARP
typedef struct packed
{
	logic		start;			//asserted for one cycle before a frame starts
	logic		data_valid;		//asserted when data is ready to be processed
	logic[2:0]	bytes_valid;	//when data_valid is set, indicated number of valid bytes in data
								//Valid bits are left aligned in data
								//1 = 31:24, 2 = 31:16, 3 = 31:8, 4 = 31:0
	logic[31:0]	data;			//actual packet content
	logic[31:0]	dst_ip;			//NOTE: This is the NEXT HOP ip address, not the ultimate destination
								//(used for MAC address lookup).
								//If the destination is outside our local subnet this is the default router's IP.

	logic		commit;			//asserted for one cycle at end of packet if checksum was good
	logic		drop;			//asserted for one cycle to indicate packet is invalid and should be discarded
} EthernetTxArpBus;

//Bus from arbiter to elastic buffer
typedef struct packed
{
	logic		start;			//asserted for one cycle before a frame starts
	logic		data_valid;		//asserted when data is ready to be processed
	logic[2:0]	bytes_valid;	//when data_valid is set, indicated number of valid bytes in data
								//Valid bits are left aligned in data
								//1 = 31:24, 2 = 31:16, 3 = 31:8, 4 = 31:0
	logic[31:0]	data;			//actual packet content
	macaddr_t	dst_mac;		//destination MAC address (source implicit)
	ethertype_t	ethertype;

	logic		commit;			//asserted for one cycle at end of packet if checksum was good
	logic		drop;			//asserted for one cycle to indicate packet is invalid and should be discarded
} EthernetTxL2Bus;

//Bus from Ethernet decoder to layer-3 protocols
typedef struct packed
{
	logic		start;				//asserted for one cycle once all headers are valid
									//but before any data is ready
	logic		data_valid;			//asserted when data is ready to be processed
	logic[2:0]	bytes_valid;		//when data_valid is set, indicated number of valid bytes in data
									//Valid bits are left aligned in data
									//1 = 31:24, 2 = 31:16, 3 = 31:8, 4 = 31:0
	logic[31:0]	data;				//actual packet content

	logic		commit;				//asserted for one cycle at end of packet if checksum was good
	logic		drop;				//asserted for one cycle to indicate packet is invalid and should be discarded

	macaddr_t	dst_mac;
	macaddr_t	src_mac;
	ethertype_t	ethertype;
	logic		ethertype_is_ipv4;
	logic		ethertype_is_ipv6;
	logic		ethertype_is_arp;

	logic		has_vlan_tag;
	vlan_t		vlan_id;			//802.1q header fields
	logic[2:0]	qos_pri;
	logic		drop_eligible;

} EthernetRxL2Bus;

//Performance counters for EthernetPerformanceCounters
typedef struct packed
{
	//tx_bus domain
	logic[47:0]	tx_frames;		//Number of frames sent
	logic[47:0]	tx_bytes;		//Number of bytes sent

	//rx_bus domain
	logic[47:0]	rx_frames;		//Number of frames successfully received
	logic[47:0]	rx_crc_err;		//Number of frames dropped due to CRC or other errors
	logic[47:0]	rx_bytes;		//Number of bytes received
} EthernetMacPerformanceData;

//Performance counters for EthernetTransmitArbiter
typedef struct packed
{
	logic[63:0]	ipv4_sent;
	logic[63:0]	ipv4_dropped;
	logic[63:0]	arp_sent;
	logic[63:0]	arp_dropped;
} EthernetArbiterPerformanceCounters;

//Performance counters for Ethernet2TypeDecoder
typedef struct packed
{
	logic[47:0]	rx_total;
	logic[47:0]	rx_ipv4;
	logic[47:0]	rx_ipv6;
	logic[47:0]	rx_arp;
} EthernetDecoderPerformanceCounters;

typedef struct packed
{
	logic		dvalid;	//asserted alternate cycles during 10/100 mode
	logic		en;
	logic		er;
	logic[7:0]	data;
} GmiiBus;

typedef enum logic[1:0]
{
	LINK_SPEED_10G		= 3,
	LINK_SPEED_1000M	= 2,
	LINK_SPEED_100M		= 1,
	LINK_SPEED_10M		= 0
} lspeed_t;

typedef struct packed
{
	logic		valid;
	logic[3:0]	ctl;
	logic[31:0]	data;
} XgmiiBus;


endpackage
