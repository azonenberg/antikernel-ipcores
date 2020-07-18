`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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

`include "I2CTransceiver.svh"

/**
	@brief Arbiter between multiple I2C masters on a single transceiver

	Assert driver_request for one cycle to request access to a given port.
	When driver_ack goes high, you may send data to the transceiver.
	Assert driver_done for one cycle when finished.
 */
module I2CArbiter #(
	parameter NUM_PORTS	= 1
) (

	input wire							clk,

	//Multiple ports from driver
	input wire[NUM_PORTS-1:0]			driver_request,
	input wire[NUM_PORTS-1:0]			driver_done,
	output logic[NUM_PORTS-1:0]			driver_ack	= 0,
	input wire i2c_in_t[NUM_PORTS-1:0]	driver_cin,
	output i2c_out_t[NUM_PORTS-1:0]		driver_cout,

	//Single port to I2C transceiver
	output i2c_in_t						txvr_cin,
	input wire i2c_out_t				txvr_cout
);

	localparam 				PORT_BITS		= $clog2(NUM_PORTS);

	logic					port_active		= 0;
	logic[PORT_BITS-1:0]	selected_port	= 0;

	//Transmit side mux
	always_comb begin

		//clear unused ports
		for(integer i=0; i<NUM_PORTS; i++)
			driver_cout[i]	<= {$bits(i2c_out_t){1'b0}};
		txvr_cin			<= {$bits(i2c_in_t){1'b0}};

		if(port_active) begin
			txvr_cin					<= driver_cin[selected_port];
			driver_cout[selected_port]	<= txvr_cout;
		end
	end

	//Arbitration
	logic[NUM_PORTS-1:0]	pending	= 0;
	logic[PORT_BITS-1:0]	rr_port	= 0;
	always_ff @(posedge clk) begin

		//Clear flags
		driver_ack	<= 0;

		//Save new requests as they come in
		pending 	<= pending | driver_request;

		//Stop sending when we finish
		if(driver_done[selected_port]) begin
			port_active				= 0;
			pending[selected_port]	<= 0;

			//increment round robin counter mod N
			rr_port					<= rr_port + 1'h1;
			if(rr_port == NUM_PORTS-1)
				rr_port				<= 0;
		end

		//Transaction in progress? Sit back and wait
		else if(port_active) begin
		end

		//Arbitration step 1: if round robin winner wants to send, let them
		else if(pending[rr_port]) begin
			port_active			= 1;
			selected_port		<= rr_port;
			driver_ack[rr_port]	<= 1;
		end

		//Arbitration step 2: lowest numbered port wins
		else if(pending) begin
			for(integer j=0; j<NUM_PORTS; j++) begin

				//If we already activated another port, don't ACK another
				if(port_active) begin
				end

				else if(pending[j]) begin
					port_active		= 1;
					selected_port	<= j;
					driver_ack[j]	<= 1;
				end

			end
		end

	end

endmodule
