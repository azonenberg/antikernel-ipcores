`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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

package APBTypes;

/**
	@brief An APB bus, as specified in ARM IHI 0024D
 */
interface APB #(
	parameter DATA_WIDTH = 8,	//can be 8, 16, or 32
	parameter ADDR_WIDTH = 16,	//can be 1-32
	parameter USER_WIDTH = 0	//can be up to 128 bits, application specific
);

	//Mandatory signals (all devices must implement)
	logic						pclk;
	logic						preset_n;
	logic[ADDR_WIDTH-1:0]		paddr;
	logic						psel;
	logic						penable;
	logic						pwrite;
	logic[DATA_WIDTH-1:0]		pwdata;
	logic						pready;
	logic[DATA_WIDTH-1:0]		prdata;

	//Additional stuff needed in some cases
	logic[2:0]					pprot;
	logic[(DATA_WIDTH/8)-1:0]	pstrb;
	logic						pslverr;
	logic						pwakeup;

	//User defined request tagging
	logic[USER_WIDTH-1:0]		pauser;
	logic[USER_WIDTH-1:0]		pwuser;
	logic[USER_WIDTH-1:0]		pruser;
	logic[USER_WIDTH-1:0]		pbuser;

	modport requester (
		output pclk, preset_n,
		output paddr, psel, penable, pwrite, pwdata,
		input pready, prdata,
		output pprot, pstrb,
		input pslverr,
		output pwakeup, pauser, pwuser,
		input pruser, pbuser
	);

	modport completer (
		input pclk, preset_n,
		input paddr, psel, penable, pwrite, pwdata,
		output pready, prdata,
		input pprot, pstrb,
		output pslverr,
		input pwakeup, pauser, pwuser,
		output pruser, pbuser
	);

endinterface

endpackage
