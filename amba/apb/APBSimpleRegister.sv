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

import APBTypes::*

/**
	@brief A single register of arbitrary size, which can be read or written bytewise over APB
 */
module APBSimpleRegister #(
	parameter REG_WIDTH 	= 32,
	parameter INIT			= 0
)(
	//The APB bus
	APB.completer apb,

	//Internal register access (in PCLK domain)
	output logic[REG_WIDTH-1:0]	regval_out = INIT,
	output logic 				updated = 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	//Combinatorially assert PREADY when selected
	always_comb begin
		apb.pready = apb.psel && apb.penable;
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			apb.prdata	<= 0;
			updated		<= 0;
			regval_out	<= INIT;
		end

		//Process bus transactions
		else if(apb.pready) begin

			if(apb.pwrite)
				regval_out[apb.paddr*8 +: 8]	<= apb.pwdata;

		end

	end

endmodule
