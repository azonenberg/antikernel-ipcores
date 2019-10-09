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

/**
	@brief Streaming HMAC-SHA-256 core (reference: RFC 4231)

	The key must be <= 512 bits in length, right-padded with zeroes if necessary. If the desired key is >512 bits long,
	pass SHA256(key) as the key.

	After asserting "start", wait until "ready" goes high before sending input to the HMAC core.
 */
module StreamingHMACSHA256(
	input wire			clk,

	input wire			key_update,
	input wire[511:0]	key,

	input wire			start,
	output logic		ready		= 0,
	input wire			update,
	input wire[31:0]	data_in,
	input wire[2:0]		bytes_valid,
	input wire			finalize,

	output logic		hash_valid	= 0,
	output logic[255:0]	hash		= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Key padding

	logic[511:0] o_key_pad	= 0;
	logic[511:0] i_key_pad	= 0;

	always_ff @(posedge clk) begin
		if(key_update) begin
			for(integer i=0; i<64; i++) begin
				o_key_pad[i*8 +: 8]	<= key[i*8 +: 8] ^ 8'h5c;
				i_key_pad[i*8 +: 8]	<= key[i*8 +: 8] ^ 8'h36;
			end
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual hash core

	/*
	logic		start 		= 0;
	logic		update 		= 0;
	logic[31:0]	data_in		= 0;
	logic[2:0]	bytes_valid	= 0;
	logic		finalize	= 0;

	wire		hash_valid;
	wire[255:0]	hash;

	StreamingSHA256 dut(
		.clk(clk),
		.start(start),
		.update(update),
		.data_in(data_in),
		.bytes_valid(bytes_valid),
		.finalize(finalize),
		.hash_valid(hash_valid),
		.hash(hash)
	);
	*/

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// HMAC wrapper logic

endmodule
