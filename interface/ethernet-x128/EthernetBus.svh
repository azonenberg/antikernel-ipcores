`timescale 1ns / 1ps
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

/**
	@file EthernetBus.svh
	@author Andrew D. Zonenberg
	@brief Structure definitions for layer-2 Ethernet with a nominal 128-bit datapath width
 */

`ifndef EthernetBus_svh
`define EthernetBus_svh

/**
	@brief XLGMII control fields
 */
typedef enum logic[7:0]
{
	XLGMII_CTL_LPI 			= 8'h06,
	XLGMII_CTL_IDLE			= 8'h07,
	XLGMII_CTL_SEQUENCE		= 8'h9c,
	XLGMII_CTL_START		= 8'hfb,
	XLGMII_CTL_TERMINATE	= 8'hfd,
	XLGMII_CTL_ERROR		= 8'hfe
} xlgmii_ctl_t;

/**
	@brief XLGMII bus as specified in 802.3-2018 clause 81

	We use an INVERTED convention for lane numbering (lane 7 is first, not 0).
	This means that bytes show up in as-transmitted order in LA/sim traces, so things make a lot more sense.
 */
typedef struct packed
{
	logic[7:0]	ctl;
	logic[63:0]	data;
} xlgmii64;

/**
	@brief XLGMII-128 bus. Double width (128 bit) and half speed (312.5 MHz) compared to standard XLGMII.

	We use an INVERTED convention for lane numbering (lane 7 is first, not 0).
	This means that bytes show up in as-transmitted order in LA/sim traces, so things make a lot more sense.
 */
typedef struct packed
{
	logic[15:0]		ctl;
	logic[127:0]	data;
} xlgmii128;


//Data output from MAC
typedef struct packed
{
	logic			start;			//Asserted during the first cycle of a frame.
									//May be concurrent with, or before, first cycle with data_valid asserted.

	logic			data_valid;		//asserted when data is ready to be processed
	logic[4:0]		bytes_valid;	//when data_valid is set, indicated number of valid bytes in data
									//Valid bytes are left aligned in data

	logic[127:0]	data;			//actual packet content

	logic			commit;			//asserted for one cycle at end of packet if checksum was good
	logic			drop;			//asserted for one cycle to indicate packet is invalid and should be discarded
									//Both commit and drop may be asserted concurrent with last cycle of data_valid.
} EthernetRxBus;

`endif
