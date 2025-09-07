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
	@brief Device information block for Trion FPGAs
 */
module APB_DeviceInfo_Trion #(
	parameter IDCODE = 32'h0	//no ICAP to read IDCODE at run time
)(
	APB.completer 	apb
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB

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
		REG_STATUS		= 8'h00,	//[0] = IDCODE valid
									//[1] = serial valid
		REG_IDCODE		= 8'h04,	//Device IDCODE
		REG_SERIAL_0	= 8'h08,	//Die serial bits 95:64
		REG_SERIAL_1	= 8'h0c,	//Die serial bits 63:32
		REG_SERIAL_2	= 8'h10,	//Die serial bits 31:0
		REG_USERCODE	= 8'h14,	//User ID
		REG_SCRATCH		= 8'h18		//Dummy register
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	logic[31:0] scratch = 32'h5555aaaa;

	always_ff @(posedge apb.pclk) begin
		if(apb.pready && apb.pwrite && (apb.paddr == REG_SCRATCH))
			scratch	<= apb.pwdata;
	end

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin
				case(apb.paddr)
					REG_STATUS:		apb.prdata = { 32'h00000003 };
					REG_IDCODE:		apb.prdata = IDCODE;
					REG_SERIAL_0:	apb.prdata = 32'h0;
					REG_SERIAL_1:	apb.prdata = 32'h0;
					REG_SERIAL_2:	apb.prdata = 32'h0;
					REG_USERCODE:	apb.prdata = 32'h0;
					REG_SCRATCH:	apb.prdata = scratch;
					default:		apb.pslverr	= 1;
				endcase
			end

			//write
			else begin
				if(apb.paddr != REG_SCRATCH)
					apb.pslverr	 = 1;
			end

		end
	end

endmodule
