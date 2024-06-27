`default_nettype none
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

/**
	@brief A timer, readable/writable in blocks over APB

	For now, only full width reads are supported (so no support for 32-bit counters on 16-bit APB)
 */
module APB_Timer #(
	parameter COUNT_WIDTH = 32
) (
	//The APB bus
	APB.completer 						apb
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support up to 32-bit APB due to register alignment, throw synthesis error for anything else

	if(apb.DATA_WIDTH > 32)
		apb_bus_width_is_invalid();

	if(COUNT_WIDTH > 32)
		count_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[3:0]
	{
		REG_COUNT		= 'h00,		//Raw count value

		REG_PREDIV		= 'h04,		//pre-divider from PCLK to counter

		REG_CTRL		= 'h08		//[0]	reset request

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	logic[COUNT_WIDTH-1:0]	count		= 0;
	logic[COUNT_WIDTH-1:0]	divcount	= 0;
	logic[COUNT_WIDTH-1:0]	prediv		= 1;

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin
				case(apb.paddr)
					REG_COUNT:		apb.prdata	= count;
					REG_PREDIV:		apb.prdata	= prediv;
					REG_CTRL:		apb.prdata	= 0;
					default:		apb.pslverr	= 1;
				endcase
			end

			//write
			else begin
				if(apb.paddr[1:0] || (apb.paddr > REG_CTRL) )
					apb.pslverr	 = 1;
			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			count			<= 0;
			divcount		<= 0;
			prediv			<= 1;
		end

		//Normal path
		else begin

			//Run the counter
			divcount	<= divcount + 1;
			if(divcount >= prediv) begin
				divcount	<= 0;
				count		<= count + 1;
			end


			if(apb.pready && apb.pwrite) begin

				case(apb.paddr)

					REG_COUNT:	count	<= apb.pwdata;
					REG_PREDIV:	prediv	<= apb.pwdata;

					REG_CTRL: begin
						if(apb.pwdata[0]) begin
							divcount	<= 0;
							count		<= 0;
						end
					end

					default: begin
					end
				endcase

			end

		end

	end

endmodule
