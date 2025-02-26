`timescale 1ns / 1ps
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

`ifndef AXIStreamTypes_sv
`define AXIStreamTypes_sv

/**
	@brief An AXI Stream bus, as specified in ARM IHI 0051B
 */
interface AXIStream #(
	parameter DATA_WIDTH	= 32,	//can be 8, 16, 32, 64, 128, 256, 512, or 1024
	parameter ID_WIDTH		= 0,	//transaction ID, recommended to be <= 8 bits
	parameter DEST_WIDTH	= 0,	//destination ID, recommended to be <= 8 bits
	parameter USER_WIDTH	= 0		//user defined sideband data, recommended to be integer multiple of byte count
);

	//Mandatory signals (all devices must implement)
	logic						aclk;
	logic						areset_n;
	logic						tvalid;
	logic						tready;
	logic[DATA_WIDTH-1:0]		tdata;
	logic						tlast;

	//Additional stuff needed in some cases
	logic[(DATA_WIDTH/8)-1:0]	tstrb;
	logic[(DATA_WIDTH/8)-1:0]	tkeep;
	logic						twakeup;

	//User defined request tagging
	logic[ID_WIDTH-1:0]			tid;
	logic[DEST_WIDTH-1:0]		tdest;
	logic[USER_WIDTH-1:0]		tuser;

	//Our convention is that ACLK and ARESET are sourced by the transmitter
	//(this can be forwarded from an input port if needed)
	modport transmitter(
		output aclk, areset_n,
		output tvalid,
		input tready,
		output tdata, tstrb, tkeep, tlast, tid, tdest, tuser, twakeup
	);

	modport receiver(
		input aclk, areset_n,
		input tvalid,
		output tready,
		input tdata, tstrb, tkeep, tlast, tid, tdest, tuser, twakeup
	);

endinterface

`endif
