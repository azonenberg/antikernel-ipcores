`timescale 1ns / 1ps
`default_nettype none
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

`include "APBTypes.sv"

/**
	@brief A simple, parameterizable APBv5 interconnect bridge

	Connects a single upstream endpoint to one or more downstream peripherals,
	each with an equal block of address space allocated starting from the base.

	Entirely combinatorial, external register stages may be added if required.
 */
module APBBridge #(
	parameter BASE_ADDR			= 32'h4000_0000,		//base address of the bridge
	parameter BLOCK_SIZE		= 32'h0000_0800,		//how much address space to allocate to each peripheral
	parameter NUM_PORTS			= 2,
	parameter ERR_ON_INVALID	= 1
)(
	APB.completer	upstream,						//single upstream port
	APB.requester	downstream[NUM_PORTS-1:0]
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Extract some width-dependent configuration

	localparam BLOCK_BITS	= $clog2(BLOCK_SIZE);
	localparam DEVICE_BITS	= upstream.ADDR_WIDTH - BLOCK_BITS;
	localparam BASE_BLOCK	= BASE_ADDR[upstream.ADDR_WIDTH-1 : BLOCK_BITS];
	localparam INDEX_BITS	= $clog2(NUM_PORTS);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pass downstream control signals through unmodified to save LUTs

	for(genvar g=0; g<NUM_PORTS; g++) begin
		assign downstream[g].pclk		= upstream.pclk;
		assign downstream[g].preset_n	= upstream.preset_n;
		assign downstream[g].penable	= upstream.penable;
		assign downstream[g].paddr		= upstream.paddr[BLOCK_BITS-1:0];
		assign downstream[g].pwrite		= upstream.pwrite;
		assign downstream[g].pwdata		= upstream.pwdata;
		assign downstream[g].pauser		= upstream.pauser;
		assign downstream[g].pwuser		= upstream.pwuser;
		assign downstream[g].pprot		= upstream.pprot;
		assign downstream[g].pstrb		= upstream.pstrb;

		//TODO: is this correct? or should we be more targeted
		assign downstream[g].pwakeup	= upstream.pwakeup;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address decoding

	logic[DEVICE_BITS-1:0]	block_addr;
	logic					range_match;
	logic[INDEX_BITS-1:0]	devid;

	always_comb begin
		block_addr 			= upstream.paddr[upstream.ADDR_WIDTH-1 : BLOCK_BITS];
		if(DEVICE_BITS > INDEX_BITS)
			range_match 	= (block_addr[DEVICE_BITS-1:INDEX_BITS] == BASE_BLOCK[DEVICE_BITS-1:INDEX_BITS]);
		else
			range_match		= 1;
		devid 				= block_addr[INDEX_BITS-1:0];
	end

	for(genvar g=0; g<NUM_PORTS; g++)
		assign downstream[g].psel		= upstream.psel && range_match && (devid == g);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Extract readback data into local vectors

	//This us derpy but needed since Vivado doesn't let us use integer indexes on interfaces, only genvars
	//TODO: check LRM and see if this is a language issue or a tooling/Verific issue

	wire							pready[NUM_PORTS-1:0];
	wire							pslverr[NUM_PORTS-1:0];
	wire[upstream.DATA_WIDTH-1:0]	prdata[NUM_PORTS-1:0];
	wire[upstream.USER_WIDTH-1:0]	pruser[NUM_PORTS-1:0];
	wire[upstream.USER_WIDTH-1:0]	pbuser[NUM_PORTS-1:0];

	for(genvar g=0; g<NUM_PORTS; g++) begin
		assign pready[g]	= downstream[g].pready;
		assign prdata[g]	= downstream[g].prdata;
		assign pruser[g]	= downstream[g].pruser;
		assign pbuser[g]	= downstream[g].pbuser;
		assign pslverr[g]	= downstream[g].pslverr;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Mux reply data

	always_comb begin

		//tie off by default
		upstream.pready		= 0;
		upstream.prdata		= 0;
		upstream.pruser		= 0;
		upstream.pbuser		= 0;
		upstream.pslverr	= 0;

		//forward from selected device
		if(upstream.psel && range_match) begin
			upstream.pready		= pready[devid];
			upstream.prdata		= prdata[devid];
			upstream.pruser		= pruser[devid];
			upstream.pbuser		= pbuser[devid];
			upstream.pslverr	= pslverr[devid];
		end

		//If upstream psel and *no* range match, force a PSLVERR to terminate the transaction
		if(ERR_ON_INVALID && upstream.psel && !range_match) begin
			upstream.pready		= 1;
			upstream.pslverr 	= 1;
		end

	end

endmodule
