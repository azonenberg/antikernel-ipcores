`timescale 1ns/1ps
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

/**
	@file
	@author Andrew D. Zonenberg
	@brief	APB wrapper providing an interface to a 7 series GTX (and possibly other) DRP
 */
module APB_SerdesDRP(
	APB.completer 		apb,

	input wire			drp_clk,

	output wire			drp_en,
	output wire			drp_we,
	output wire[8:0]	drp_addr,
	output wire[15:0]	drp_wdata,
	input wire[15:0]	drp_rdata,
	input wire			drp_rdy,

	input wire			rx_rst_done
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 16)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		REG_ADDR		= 'h00,	//8:0 DRP address
								//15 = 1 to write (must have written REG_DATA first), 0 to read
		REG_DATA		= 'h04,	//DRP read/write data
		REG_STATUS		= 'h08,	//0 = DRP busy
								//1 = RX reset done

		REG_STATUS_2	= 'h28	//mirror of REG_STATUS
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock domain crossing for the DRP

	logic		drp_en_int		= 0;
	logic		drp_we_int		= 0;
	logic[8:0]	drp_addr_int	= 0;
	logic[15:0]	drp_wdata_int	= 0;
	wire[15:0]	drp_rdata_int;
	wire		drp_done;

	DRPClockDomainShifting drp_cdc(
		.mgmt_clk(apb.pclk),
		.mgmt_en(drp_en_int),
		.mgmt_wr(drp_we_int),
		.mgmt_addr(drp_addr_int),
		.mgmt_wdata(drp_wdata_int),
		.mgmt_rdata(drp_rdata_int),
		.mgmt_done(drp_done),

		.drp_clk(drp_clk),
		.drp_en(drp_en),
		.drp_we(drp_we),
		.drp_addr(drp_addr),
		.drp_di(drp_wdata),
		.drp_do(drp_rdata),
		.drp_rdy(drp_rdy)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	logic		drp_busy	= 0;

	//Combinatorial readback and writes
	always_comb begin

		apb.pready		= apb.psel && apb.penable;
		apb.prdata		= 0;
		apb.pslverr		= 0;

		if(apb.pready) begin

			//write
			if(apb.pwrite) begin

				case(apb.paddr)

					REG_ADDR: begin
						//registered, nothing needed here
					end

					REG_DATA: begin
						//registered, nothing needed here
					end

					//illegal / unmapped address
					default:	apb.pslverr = 1;

				endcase

			end

			//read
			else begin

				case(apb.paddr)
					//no readback of address allowed
					REG_DATA:		apb.prdata	= drp_rdata_int;
					REG_STATUS:		apb.prdata	= { 14'h0, rx_rst_done, drp_busy };
					REG_STATUS_2:	apb.prdata	= { 14'h0, rx_rst_done, drp_busy };

					//illegal / unmapped address
					default:		apb.pslverr	= 1;
				endcase

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			drp_busy		<= 0;
			drp_wdata_int	<= 0;
		end

		//Normal path
		else begin

			drp_en_int		<= 0;

			//Track busy state
			if(drp_done)
				drp_busy	<= 0;
			if(drp_en_int)
				drp_busy	<= 1;

			if(apb.pready && apb.pwrite) begin

				case(apb.paddr)

					REG_DATA:	drp_wdata_int	<= apb.pwdata;

					REG_ADDR: begin
						drp_en_int		<= 1;
						drp_we_int		<= apb.pwdata[15];
						drp_addr_int	<= apb.pwdata[8:0];
					end

					default: begin
					end
				endcase
			end

		end

	end

endmodule
