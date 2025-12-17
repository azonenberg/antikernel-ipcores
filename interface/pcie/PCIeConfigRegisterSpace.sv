`timescale 1ns/1ps
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
	@brief PCI Express configuration register space
 */
module PCIeConfigRegisterSpace #(
	parameter DEVICE_ID		= 16'hbeef,
	parameter VENDOR_ID		= 16'hdead,

	//Device type
	parameter BASE_CLASS	= 8'h05,	//memory device
	parameter SUB_CLASS		= 8'h00,	//RAM
	parameter PROG_IFACE	= 8'h00,	//always 0
	parameter REVISION_ID	= 8'h00
)(
	APB.completer	apb,

	input wire		dl_link_up
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 32 bit bus required, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[7:0]
	{
		//PCI base space
		REG_DEVICE_VENDOR	= 'h00,
		REG_STATUS_COMMAND	= 'h04,
		REG_CLASS_REV		= 'h08,
		REG_TYPE_INFO		= 'h0c,
		REG_BAR0			= 'h10,
		REG_BAR1			= 'h14,
		REG_BAR2			= 'h18,
		REG_BAR3			= 'h1c,
		REG_BAR4			= 'h20,
		REG_BAR5			= 'h24,
		REG_CARDBUS_CIS		= 'h28,
		REG_SUBSYS			= 'h2c,
		REG_ROM_BASE		= 'h30,
		REG_CAPS_PTR		= 'h34,
		REG_RSVD_38			= 'h38,
		REG_LAT_IRQ			= 'h3c,

		//PCIe extended caps space
		REG_PCIE_EXT_BASE	= 'h40,
		REG_PCIE_DEV_CAPS	= 'h44,
		REG_PCIE_DEV_CTL	= 'h48,
		REG_PCIE_LINK_CAPS	= 'h4c,
		REG_PCIE_LINK_CTL	= 'h50,
		REG_PCIE_SLOT_CAPS	= 'h54,
		REG_PCIE_SLOT_CTL	= 'h58,
		REG_PCIE_DEV_CAPS2	= 'h64,
		REG_PCIE_DEV_CTRLS2	= 'h68,
		REG_PCIE_LINK_CAPS2	= 'h6c

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register state

	logic[31:0]	bar0	= 0;
	logic[31:0]	bar1	= 0;
	logic[31:0]	bar2	= 0;
	logic[31:0]	bar3	= 0;
	logic[31:0]	bar4	= 0;
	logic[31:0]	bar5	= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Combinatorial reads

	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin
				case(apb.paddr)

					////////////////////////////////////////////////////////////////////////////////////////////////////
					// Base register space

					//VID/PID
					REG_DEVICE_VENDOR:	apb.prdata = { DEVICE_ID, VENDOR_ID };

					//Status / command
					REG_STATUS_COMMAND:	apb.prdata =
					{
						//Status register: mostly hardwired state
						1'b0,	//detected parity
						1'b0,	//signaled system error
						1'b0,	//rx master abort
						1'b0,	//rx target abort
						1'b0,	//signaled target abort
						2'b0,	//devsel timing, must be 0
						1'b0,	//master data parity error
						1'b0,	//fast b2b transaction, must be 0
						1'b0,	//reserved, must be 0
						1'b0,	//66 MHz capable, must be 0
						1'b1,	//extended capabilities list present
						1'b0,	//interrupt status
						3'b0,	//reserved, must be 0

						//Command register: nothing implemented yet, all zero
						16'h0
					};

					//Class / revision
					REG_CLASS_REV:		apb.prdata = { BASE_CLASS, SUB_CLASS, PROG_IFACE, REVISION_ID };

					//Header type and cache stuff we don't care about
					REG_TYPE_INFO:		apb.prdata = 32'h00000000;

					//TODO: type/bist

					//BARs
					REG_BAR0:			apb.prdata = bar0;
					REG_BAR1:			apb.prdata = bar1;
					REG_BAR2:			apb.prdata = bar2;
					REG_BAR3:			apb.prdata = bar3;
					REG_BAR4:			apb.prdata = bar4;
					REG_BAR5:			apb.prdata = bar5;

					//Cardbus CIS, not implemented
					REG_CARDBUS_CIS:	apb.prdata = 0;

					//TODO: subsystem

					//No expansion ROM
					REG_ROM_BASE:		apb.prdata = 0;

					//Pointer to extended capabilities space
					REG_CAPS_PTR:		apb.prdata = 32'h0000_0040;

					////////////////////////////////////////////////////////////////////////////////////////////////////
					// PCIe extended caps space

					//PCIe capability structure, no next pointer, version 2.
					//We're an endpoint.
					REG_PCIE_EXT_BASE:	apb.prdata = 32'h0002_0010;

					//May payload size 256 bytes, no phantom functions, no extended tag
					//leave all L0s / L1 latency numbers at base, we don't implement power mgmt yet
					//No function level reset
					REG_PCIE_DEV_CAPS:	apb.prdata = 32'h0000_0001;

					//256 byte max payload size, 256 byte max read request
					//also device status but we don't implement any of that yet
					REG_PCIE_DEV_CTL:	apb.prdata = 32'h0000_1020;

					//We support 2.5G only (for now), x1 only, no power management
					REG_PCIE_LINK_CAPS:	apb.prdata = 32'h0000_0011;

					//Link status/control
					REG_PCIE_LINK_CTL:	apb.prdata =
					{
						//Autonomous bandwidth - not implemented
						1'b0,

						//Bandwidth status - not implemented
						1'b0,

						//Data link layer up
						dl_link_up,

						//Slot clock config, not implemented
						1'b0,

						//Link training status, always 0
						1'b0,

						//undefined
						1'b0,

						//always linked in x1, we don't support more lanes
						5'h0,

						//always linked at 2.5G for now
						4'b0001,

						16'h0000
					};

					//Slot capabilities - not implemented
					REG_PCIE_SLOT_CAPS:	apb.prdata = 32'h0000_0000;

					//Slot control and status - not implemented
					REG_PCIE_SLOT_CTL:	apb.prdata = 32'h0000_0000;

					//Root control and caps - not implemented
					//Root status - not implemented

					//Device capabilities 2
					REG_PCIE_DEV_CAPS2: apb.prdata = 32'h0000_0000;

					//Device control / status 2
					REG_PCIE_DEV_CTRLS2: apb.prdata = 32'h0000_0000;

					//Link capabilities 2 - support 2.5G only
					REG_PCIE_LINK_CAPS2: apb.prdata = 32'h0000_0002;

					////////////////////////////////////////////////////////////////////////////////////////////////////
					// Other

					//return 0xff to any unmapped address
					default:			apb.prdata	= 32'hffffffff;
				endcase
			end

			//write
			else begin
				//for now, always succeed
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronous writes

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			bar0	<= 0;
			bar1	<= 0;
			bar2	<= 0;
			bar3	<= 0;
			bar4	<= 0;
			bar5	<= 0;
		end

		else begin

			if(apb.pready && apb.pwrite) begin

				case(apb.paddr)
					REG_BAR0:	bar0	<= apb.pwdata;
					REG_BAR1:	bar1	<= apb.pwdata;
					REG_BAR2:	bar2	<= apb.pwdata;
					REG_BAR3:	bar3	<= apb.pwdata;
					REG_BAR4:	bar4	<= apb.pwdata;
					REG_BAR5:	bar5	<= apb.pwdata;

					default: begin
					end
				endcase
			end

		end

	end

endmodule
