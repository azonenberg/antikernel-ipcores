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
	@brief An abstract simulation model of a serial channel without worrying about any specifics of implementation
 */
module SerialChannelModel #(
	parameter WIDTH		= 32,	//Parallel bus width at the input or output of the simulated SERDES
	parameter DELAY		= 8,	//One-way channel latency, in clock cycles (must be >2 and, for now, <= 32)
	parameter BITSLIP	= 1		//Number of bits to shift from input to output
) (

	//TX side
	input wire				tx_clk,
	input wire[WIDTH-1:0]	tx_data,

	//RX side
	output wire				rx_clk,
	output wire[WIDTH-1:0]	rx_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Forward the clock

	assign rx_clk = tx_clk;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Bitslip

	/*
		Note, conventional serdes bit order is LSB first!
		So abc on the wire is cba at the transceiver ports
		Now consider abc def on the wire, this will be cba fed at the ports normally
		If we bitslip by 1, we will instead sample bcd which is dcb at the ports
		so this is { current[0], prev[2:1] }
	 */

	logic[WIDTH-1:0] tx_data_ff = 0;
	logic[WIDTH-1:0] bitslip_data;
	always_ff @(posedge tx_clk) begin
		tx_data_ff		<= tx_data;
		bitslip_data 	<= { tx_data[BITSLIP-1:0], tx_data_ff[WIDTH-1:BITSLIP] };
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Delay

	ShiftRegisterMacro #(
		.WIDTH(WIDTH),
		.DEPTH(32)
	) shreg (
		.clk(tx_clk),
		.addr(DELAY-3),
		.din(bitslip_data),
		.ce(1'b1),
		.dout(rx_data)
	);

	//TODO: error injection

endmodule
