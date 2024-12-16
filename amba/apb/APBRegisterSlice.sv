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
	@brief An APBv5 pipeline register (parameterizable upstream, downstream, or both)
 */
module APBRegisterSlice #(
	parameter UP_REG	= 1,	//register upstream ports coming from the peripheral
	parameter DOWN_REG	= 1		//register downstream ports coming from the bridge
) (
	APB.completer	upstream,
	APB.requester	downstream
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Downstream traffic

	//Pass through clock/reset
	assign downstream.pclk = upstream.pclk;
	assign downstream.preset_n = upstream.preset_n;

	logic	done = 0;

	//Register path
	if(DOWN_REG) begin

		always_ff @(posedge upstream.pclk or negedge upstream.preset_n) begin

			//reset
			if(!upstream.preset_n) begin
				downstream.paddr	<=	0;
				downstream.psel		<=	0;
				downstream.penable	<=	0;
				downstream.pwrite	<=	0;
				downstream.pwdata	<=	0;
				downstream.pprot	<=	0;
				downstream.pstrb	<=	0;
				downstream.pwakeup	<=	0;
				downstream.pauser	<=	0;
				downstream.pwuser	<=	0;
				done				<=	0;
			end

			//normal path
			else begin
				downstream.paddr	<=	upstream.paddr;
				downstream.psel		<=	upstream.psel;
				downstream.penable	<=	upstream.penable;
				downstream.pwrite	<=	upstream.pwrite;
				downstream.pwdata	<=	upstream.pwdata;
				downstream.pprot	<=	upstream.pprot;
				downstream.pstrb	<=	upstream.pstrb;
				downstream.pwakeup	<=	upstream.pwakeup;
				downstream.pauser	<=	upstream.pauser;
				downstream.pwuser	<=	upstream.pwuser;

				//Special path needed to handle the case of PENABLE being asserted upstream
				//after PREADY is asserted downstream
				if(downstream.pready) begin
					done				<= 1;
					downstream.penable	<= 0;
					downstream.psel		<= 0;
				end

				if(done) begin
					downstream.penable	<= 0;
					downstream.psel		<= 0;
				end

				if(!upstream.penable) begin
					done				<= 0;
					downstream.psel		<= 0;
				end

			end

		end
	end

	//Combinatorial path
	else begin

		always_ff @(posedge upstream.pclk or negedge upstream.preset_n) begin
			if(!upstream.preset_n)
				done				<= 0;

			else begin

				if(downstream.pready)
					done			<= 1;
				if(!upstream.penable)
					done			<= 0;

			end

		end

		always_comb begin
			downstream.paddr		=	upstream.paddr;
			downstream.psel			=	upstream.psel;
			downstream.penable		=	upstream.penable;
			downstream.pwrite		=	upstream.pwrite;
			downstream.pwdata		=	upstream.pwdata;
			downstream.pprot		=	upstream.pprot;
			downstream.pstrb		=	upstream.pstrb;
			downstream.pwakeup		=	upstream.pwakeup;
			downstream.pauser		=	upstream.pauser;
			downstream.pwuser		=	upstream.pwuser;

			//Special path needed to handle the case of PENABLE being asserted upstream
			//after PREADY is asserted downstream
			if(done)
				downstream.penable	= 0;

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Upstream traffic

	//Register path
	if(UP_REG) begin

		always_ff @(posedge upstream.pclk or negedge upstream.preset_n) begin

			//reset
			if(!upstream.preset_n) begin
				upstream.pready		<= 0;
				upstream.prdata		<= 0;
				upstream.pslverr	<= 0;
				upstream.pruser		<= 0;
				upstream.pbuser		<= 0;
			end

			//normal path
			else begin
				upstream.pready		<= downstream.pready;
				upstream.prdata		<= downstream.prdata;
				upstream.pslverr	<= downstream.pslverr;
				upstream.pruser		<= downstream.pruser;
				upstream.pbuser		<= downstream.pbuser;
			end
		end

	end

	//Combinatorial path
	else begin
		always_comb begin
			upstream.pready			= downstream.pready;
			upstream.prdata			= downstream.prdata;
			upstream.pslverr		= downstream.pslverr;
			upstream.pruser			= downstream.pruser;
			upstream.pbuser			= downstream.pbuser;
		end
	end

endmodule
