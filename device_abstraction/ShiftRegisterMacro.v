`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2017 Andrew D. Zonenberg                                                                          *
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
	@brief A parameterizable width / depth (addressable up to 32 bits for now) shift register.

	dout_concat is a concatenated copy of the shift register elements for use in formal verification only.
	Do not use it for synthesis.

	dout_concat is LEFT aligned!
	MSB is most recent word pushed
 */

module ShiftRegisterMacro #(
	parameter WIDTH = 16,
	parameter DEPTH = 32,
	parameter ADDR_BITS = 5
) (
	input wire clk,
	input wire[ADDR_BITS-1 : 0] addr,
	input wire[WIDTH-1 : 0] din,
	input wire ce,
	output wire[WIDTH-1 : 0] dout

	`ifdef FORMAL
	, output reg[WIDTH*DEPTH - 1 : 0] dout_concat
	`endif
);

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Sanity checking

	generate
		initial begin
			if(DEPTH > 32) begin
				$display("ERROR - ShiftRegisterMacro only supports depth values less than 32 for now");
				$finish;
			end
		end
	endgenerate

	////////////////////////////////////////////////////////////////////////////////////////////////
	// The RAM itself

	`ifndef FORMAL
		genvar i;
		generate
			for(i=0; i<WIDTH; i = i+1) begin: shregblock
				ShiftRegisterPrimitiveWrapper #(.DEPTH(32), .ADDR_BITS(ADDR_BITS)) shregbit (
					.clk(clk),
					.addr(addr),
					.din(din[i]),
					.ce(ce),
					.dout(dout[i])
					);
			end
		endgenerate

	`else

		//More easily constrainable behavioral simulation model
		//We can easily constrain this with a .smtc file to match an inferred RAM, MemoryMacro, etc

		integer i;

		reg[WIDTH-1 : 0] storage[DEPTH-1 : 0];
		assign dout = storage[addr];

		initial begin
			for(i=0; i<DEPTH; i=i+1)
				storage[i] <= 0;
		end

		always @(posedge clk) begin
			if(ce) begin
				for(i=DEPTH-1; i>=1; i=i-1)
					storage[i] <= storage[i-1];
				storage[0] <= din;
			end
		end

	`endif

	////////////////////////////////////////////////////////////////////////////////////////////////
	// Dedicated output for formal verification only

	`ifdef FORMAL
		always @(*) begin
			for(i=0; i<DEPTH; i=i+1)
				dout_concat[i*WIDTH +: WIDTH] <= storage[DEPTH + i - 1];
		end
	`endif

endmodule

/**
	@brief Dumb wrapper around a single SRL* to use vector addresses.

	Parameterizable depth.
 */
module ShiftRegisterPrimitiveWrapper #(
	parameter DEPTH = 32,
	parameter ADDR_BITS = 5
)  (
	input wire clk,
	input wire[ADDR_BITS-1 : 0] addr,
	input wire din,
	input wire ce,
	output wire dout);

	generate
		if(DEPTH == 32) begin

			//instantiate the Xilinx primitive
			`ifndef FORMAL
				SRLC32E #(
					.INIT(32'h00000000)
				) shreg (
					.Q(dout),
					.Q31(),		//cascade output not used yet
					.A(addr),
					.CE(ce),
					.CLK(clk),
					.D(din)
				);

			//simple behavioral model
			`else
				reg[31:0] data = 0;
				assign dout = data[addr];
				always @(posedge clk) begin
					if(ce)
						data <= {data[30:0], din};
				end
			`endif

		end
		else begin
			initial begin
				$display("Invalid depth");
				$finish;
			end
		end
	endgenerate

endmodule
