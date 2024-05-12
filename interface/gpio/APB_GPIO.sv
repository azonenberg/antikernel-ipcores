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

/**
	@brief A GPIO controller, readable/writable in blocks over APB
 */
module APB_GPIO #(
	parameter OUT_INIT		= 16'h0,
	parameter TRIS_INIT		= 16'h0
)(
	//The APB bus
	APB.completer 						apb,

	//The GPIO signals
	output logic[apb.DATA_WIDTH-1:0]	gpio_out	= OUT_INIT,
	output logic[apb.DATA_WIDTH-1:0]	gpio_tris	= TRIS_INIT,
	input wire[apb.DATA_WIDTH-1:0]		gpio_in
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support up to 32-bit APB due to register alignment, throw synthesis error for anything else

	if(apb.DATA_WIDTH > 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[3:0]
	{
		REG_GPIO_OUT	= 'h00,
		REG_GPIO_IN		= 'h04,
		REG_GPIO_TRIS	= 'h08
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin
				case(apb.paddr)
					REG_GPIO_OUT:	apb.prdata	= gpio_out;
					REG_GPIO_IN:	apb.prdata	= gpio_in;
					REG_GPIO_TRIS:	apb.prdata	= gpio_tris;
					default:		apb.pslverr	= 1;
				endcase
			end

			//write
			else begin
				if( (apb.paddr != REG_GPIO_OUT) &&
					(apb.paddr != REG_GPIO_TRIS) ) begin

					apb.pslverr	 = 1;

				end
			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			gpio_out	<= OUT_INIT;
			gpio_tris	<= TRIS_INIT;
		end

		//Normal path
		else begin

			if(apb.pready && apb.pwrite) begin

				case(apb.paddr)

					REG_GPIO_OUT:	gpio_out	<= apb.pwdata;
					REG_GPIO_TRIS:	gpio_tris	<= apb.pwdata;

					default: begin
					end
				endcase

			end

		end

	end

endmodule
