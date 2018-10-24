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

/**
	@file IPv4Bus.svh
	@author Andrew D. Zonenberg
	@brief Structure definitions for layer-3 IPv4 buses

	Conventions
		start is asserted before, not simultaneous with, first assertion of data_valid
		bytes_valid is always 4 until last word in the packet, at which point it may take any value
		commit is asserted after, not simultaneous with, last assertion of data_valid
 */

`ifndef IPv4Bus_h
`define IPv4Bus_h

typedef struct packed
{
	logic[31:0]	address;
	logic[31:0]	mask;
	logic[31:0]	broadcast;
	logic[31:0]	gateway;
} IPv4Config;

typedef struct packed
{
	logic		start;			//asserted for one cycle before a frame starts
	logic		data_valid;		//asserted when data is ready to be processed
	logic[2:0]	bytes_valid;	//when data_valid is set, indicated number of valid bytes in data
								//Valid bits are left aligned in data
								//1 = 31:24, 2 = 31:16, 3 = 31:8, 4 = 31:0
	logic[31:0]	data;			//actual packet content

	logic[15:0]	payload_len;	//size of upper layer payload only
								//(not the IP datagram length)
	logic[7:0]	protocol;
	logic		protocol_is_icmp;
	logic		protocol_is_udp;
	logic		protocol_is_tcp;
	logic[31:0]	src_ip;
	logic[31:0]	dst_ip;
	logic		headers_valid;
	logic[15:0]	pseudo_header_csum;

	logic		commit;			//asserted for one cycle at end of packet if checksum was good
	logic		drop;			//asserted for one cycle to indicate packet is invalid and should be discarded
} IPv4RxBus;

typedef struct packed
{
	logic		start;			//asserted for one cycle before a frame starts
	logic		data_valid;		//asserted when data is ready to be processed
	logic[2:0]	bytes_valid;	//when data_valid is set, indicated number of valid bytes in data
								//Valid bits are left aligned in data
								//1 = 31:24, 2 = 31:16, 3 = 31:8, 4 = 31:0
	logic[31:0]	data;			//actual packet content

	logic[15:0]	payload_len;	//size of upper layer payload only
								//(not the IP datagram length)
	logic[7:0]	protocol;
	logic[31:0]	dst_ip;

	logic		commit;
	logic		drop;
} IPv4TxBus;

`endif
