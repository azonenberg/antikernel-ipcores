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
	@file TCPv4Bus.svh
	@author Andrew D. Zonenberg
	@brief Structure definitions for layer-4 TCPv4 buses
 */

`ifndef TCPv4Bus_h
`define TCPv4Bus_h

typedef logic[15:0]	portnum_t;

typedef struct packed
{
	logic		start;			//asserted for one cycle before a frame starts
	logic[31:0]	src_ip;			//IP the data came from (important if we're replying)
	portnum_t	src_port;
	portnum_t	dst_port;

	logic		data_valid;		//asserted when data is ready to be processed
	logic[2:0]	bytes_valid;	//when data_valid is set, indicated number of valid bytes in data
								//Valid bits are left aligned in data
								//1 = 31:24, 2 = 31:16, 3 = 31:8, 4 = 31:0
	logic[31:0]	data;			//actual packet content

	logic[15:0]	payload_len;	//size of upper layer payload only
								//(not the IP datagram length)
	logic[15:0]	sockid;			//ID of the socket that this segment came from

	logic		commit;			//asserted for one cycle at end of packet if checksum was good
	logic		drop;			//asserted for one cycle to indicate packet is invalid and should be discarded

	//Socket state notifications, asserted for one cycle when something happens to a socket
	logic		open;
	logic		close;
} TCPv4RxBus;

typedef struct packed
{
	logic		start;
	logic[15:0]	sockid;
	logic[31:0]	dst_ip;
	portnum_t	src_port;
	portnum_t	dst_port;

	logic		data_valid;
	logic[2:0]	bytes_valid;
	logic[31:0]	data;

	//payload length is calculated at commit time, since we can't send the packet without a checksum anyway
	//no need to make application layer precompute it

	logic		commit;
	logic		drop;

} TCPv4TxBus;

`endif
