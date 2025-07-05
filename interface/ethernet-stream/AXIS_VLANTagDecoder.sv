`timescale 1ns / 1ps
`default_nettype none
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
	@file
	@author Andrew D. Zonenberg
	@brief AXI4-Stream VLAN tag decoding

	TDEST will be populated with the VLAN ID, derived from the port ID or the 802.1q tag as appropriate
 */
module AXIS_VLANTagDecoder(

	//AXI buses for packets
	AXIStream.receiver			axi_rx,
	AXIStream.transmitter		axi_tx,

	//VLAN control signals
	input wire[11:0]			port_vlan,
	input wire					drop_tagged,
	input wire					drop_untagged
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Validate bus width (this core only works for 32 bits wide)

	if(axi_rx.DATA_WIDTH != 32)
		axi_bus_width_inconsistent();
	if(axi_rx.DATA_WIDTH != 32)
		axi_bus_width_inconsistent();
	if(axi_tx.DEST_WIDTH != 12)
		axi_bus_width_inconsistent();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Forward AXI control signals

	assign axi_tx.aclk		= axi_rx.aclk;
	assign axi_tx.areset_n	= axi_rx.areset_n;
	assign axi_tx.twakeup	= axi_rx.twakeup;

	assign axi_rx.tready	= axi_tx.tready;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// VLAN tag decoding

	/*
		Word 0: mac dest
		Word 1: mac dest + src
		Word 2: mac src
		Word 3: 802.1q tag or gap
	 */

	logic[10:0] count = 0;

	logic		is_ethertype;
	logic		is_dot1q_tagged;
	logic		dropping	= 0;

	always_comb begin

		//Match position of ethertype field
		is_ethertype		= (count == 3);

		//Match 802.1q tag (only valid if is_ethertype)
		is_dot1q_tagged		= (axi_rx.tdata[7:0] == 8'h81) && (axi_rx.tdata[15:8] == 8'h00);

		//Forward everything by default
		axi_tx.tvalid		= axi_rx.tvalid;
		axi_tx.tdata		= axi_rx.tdata;
		axi_tx.tuser		= axi_rx.tuser;
		axi_tx.tlast		= axi_rx.tlast;
		axi_tx.tstrb		= axi_rx.tstrb;
		axi_tx.tkeep		= axi_rx.tkeep;

		//Don't forward ethertype on tagged frames
		if(is_ethertype && is_dot1q_tagged) begin
			axi_tx.tvalid	= 0;
			axi_tx.tstrb	= 0;
			axi_tx.tkeep	= 0;
		end

		//Drop on request
		if(axi_tx.tlast && dropping)
			axi_tx.tuser	= 1;

	end

	always_ff @(posedge axi_rx.aclk or negedge axi_rx.areset_n) begin
		if(!axi_rx.areset_n) begin
			count			<= 0;
			axi_tx.tdest	<= 0;
			dropping		<= 0;
		end

		else begin

			//increment every word
			if(axi_rx.tvalid && axi_rx.tready)
				count	<= count + 1;
			if(axi_rx.tlast) begin
				count		<= 0;
				dropping	<= 0;
			end

			//Look for VLAN tag
			if(is_ethertype && axi_rx.tvalid && axi_rx.tready) begin

				//Ethertype 0x8100?
				if(is_dot1q_tagged) begin

					//Discard PCP/DEI, just grab the VLAN ID
					axi_tx.tdest	<=  { axi_rx.tdata[19:16], axi_rx.tdata[31:24] };

					dropping		<= drop_tagged;

				end

				//Untagged frame, use the port VLAN ID
				else begin
					axi_tx.tdest	<= port_vlan;
					dropping		<= drop_untagged;
				end

			end

		end
	end

endmodule
