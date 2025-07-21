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
	@file
	@author	Andrew D. Zonenberg
	@brief	APB_MDIO alternative implementation with external tristate buffer for MDIO

	Write process:
	* verify STATUS is zero
	* write register / PHY ID to CMD_ADDR and set write bit
	* write register value to DATA

	Read process:
	* verify STATUS is zero
	* write register / PHY ID to CMD_ADDR
	* wait for STATUS to be zero
	* read register value from DATA

	Reads from DATA may return stale or garbage data if BUSY is set, but will not stall the bus.
 */
module APB_MDIO_ExternalTristate #(
	parameter CLK_DIV		= 16	//clock divider from PCLK to MDIO clock domain
) (

	//The APB bus
	APB.completer 			apb,

	//The MDIO bus
	output wire				mdio_tx_data,
	output wire				mdio_tx_en,
	input wire				mdio_rx_data,
	output wire				mdc
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16 or 32-bit APB, throw synthesis error for anything else

	if( (apb.DATA_WIDTH != 16) && (apb.DATA_WIDTH != 32) )
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	//Align all writable registers to 0x20 boundaries to work around STM32H7 OCTOSPI bugs
	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		CMD_ADDR	= 'h00,		//15	1=write, 0=read (always RAZ)
								//12:8	Register address
								//4:0	PHY address

		DATA		= 'h20,		//Read/write data

		STATUS		= 'h40,		//Busy flag

		STATUS2		= 'h60		//Duplicate of STATUS at 0x20, 32 bytes offset
								//(workaround for inability to disable prefetch cache on STM32H7)

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The MDIO transceiver itself

	logic[4:0]	phy_reg_addr;
	logic[15:0]	phy_wr_data;
	wire[15:0]	phy_rd_data;
	logic		phy_reg_wr;
	logic		phy_reg_rd;
	logic[4:0]	phy_md_addr;

	wire		mdio_busy;

	EthernetMDIOTransceiver #(
		.CLK_DIV(CLK_DIV)
	)  mdio_txvr (
		.clk(apb.pclk),
		.phy_md_addr(phy_md_addr),

		.mdio_tx_data(mdio_tx_data),
		.mdio_tx_en(mdio_tx_en),
		.mdio_rx_data(mdio_rx_data),
		.mdc(mdc),

		.mgmt_busy_fwd(mdio_busy),
		.phy_reg_addr(phy_reg_addr),
		.phy_wr_data(phy_wr_data),
		.phy_rd_data(phy_rd_data),
		.phy_reg_wr(phy_reg_wr),
		.phy_reg_rd(phy_reg_rd)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	always_comb begin

		//Combinatorially assert PREADY when selected
		apb.pready	= apb.psel && apb.penable;

		//Default to no errors and no read data
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//Write path
			if(apb.pwrite) begin

				case(apb.paddr)

					//Command and address
					CMD_ADDR: begin
					end

					DATA: begin
					end

					//invalid address
					default:	apb.pslverr = 1;

				endcase

			end

			//Read data muxing
			else begin
				case(apb.paddr)

					CMD_ADDR:	apb.prdata	= {4'h0, phy_reg_addr, 3'h0, phy_md_addr};
					DATA:		apb.prdata	= phy_rd_data;

					STATUS:		apb.prdata	= { 15'h0, mdio_busy };
					STATUS2:	apb.prdata	= { 15'h0, mdio_busy };

					//unmapped address, error
					default:	apb.pslverr	= 1;

				endcase

			end
		end

	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			phy_reg_wr		<= 0;
			phy_reg_rd		<= 0;
			phy_md_addr		<= 0;
			phy_reg_addr	<= 0;
		end

		//Normal path
		else begin

			//Clear MDIO control signals
			phy_reg_wr		<= 0;
			phy_reg_rd		<= 0;

			//Registered writes
			if(apb.pready && apb.pwrite) begin

				case(apb.paddr)

					//Command and address
					CMD_ADDR: begin

						//start read operation on write with MSB clear
						if(!apb.pwdata[15])
							phy_reg_rd	<= 1;

						//extract address
						phy_reg_addr	<= apb.pwdata[12:8];
						phy_md_addr		<= apb.pwdata[4:0];

					end

					DATA: begin

						//start write operation
						phy_reg_wr		<= 1;
						phy_wr_data		<= apb.pwdata;

					end

					default: begin
					end

				endcase
			end

		end

	end

endmodule
