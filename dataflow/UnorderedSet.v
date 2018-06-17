`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
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
	@brief RTL equivalent of std::unordered_set (but with a fixed maximum size)

	Single cycle insert/remove. The internal data structure is essentially a fully associative cache with
	automatic de-duplication.

	This is meant for VERY SMALL depths, as critical paths will get massive if you have more than a handful of elements!
 */
module UnorderedSet #(
	parameter DEPTH			= 16,	//number of elements in the set
	parameter ITER_SIZE		= 4,	//must be clog2(DEPTH)

	parameter KEY_SIZE		= 16	//Size of a single key element in bits
) (
	input wire					clk,

	input wire					insert_en,		//assert to add a new element
	input wire[KEY_SIZE-1:0]	insert_key,		//the element to insert
	output reg					insert_ok = 0,	//goes high after 1 cycle on successful insert
												//stays low to indicate the set is full

	input wire					iter_inc,		//assert to increment the iterator at iter_in
	input wire					iter_begin,		//assert to look up the first element in the set, ignoring iter_in
	input wire[ITER_SIZE-1:0]	iter_in,		//previous value of the iterator
	output reg					iter_end = 0,	//true if we hit the end (iter_next not valid)
	output reg[ITER_SIZE-1:0]	iter_next = 0,	//next value of the iterator
	output reg[KEY_SIZE-1:0]	iter_key = 0,	//key associated with this iterator

	input wire					remove_en,		//assert to clear an item by iterator (remove by key not supported)
	input wire[ITER_SIZE-1:0]	remove_iter		//the iterator to remove
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual storage array

	integer i;

	reg					slot_valid[DEPTH-1:0];
	reg[KEY_SIZE-1:0]	slot_key[DEPTH-1:0];

	initial begin
		for(i=0; i<DEPTH; i=i+1) begin
			slot_valid[i]	<= 0;
			slot_key[i]		<= 0;
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Iterator management. This is combinatorial so external registers can be used.

	reg		hit = 0;

	always @(*) begin

		iter_end		<= 1;		//Default to "end of set, iterator not valid"
		iter_next		<= 0;

		hit				= 0;

		//When RESETTING the iterator: find the first valid element
		if(iter_begin) begin
			for(i=0; i<DEPTH; i=i+1) begin
				if(slot_valid[i] && !hit) begin
					hit			= 1;
					iter_end	<= 0;
					iter_next	<= i;
					iter_key	<= slot_key[i];
				end
			end
		end

		//When INCREMENTING the iterator: find the first valid element with index >= iter_in
		else if(iter_inc) begin
			for(i=0; i<DEPTH; i=i+1) begin
				if(slot_valid[i] && !hit && (i > iter_in) ) begin
					hit			= 1;
					iter_end	<= 0;
					iter_next	<= i;
					iter_key	<= slot_key[i];
				end
			end
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Insertion/removal

	always @(posedge clk) begin

		insert_ok		= 0;

		//Process inserts
		if(insert_en) begin

			//Check if the key is already in the set. If so, no action needed.
			for(i=0; i<DEPTH; i=i+1) begin
				if(slot_valid[i] && (slot_key[i] == insert_key) ) begin
					insert_ok		= 1;
				end
			end

			//If not in the set, insert it.
			for(i=0; i<DEPTH; i=i+1) begin
				if(!slot_valid[i] && !insert_ok) begin
					insert_ok		= 1;
					slot_valid[i]	<= 1;
					slot_key[i]		<= insert_key;
				end
			end

		end

		//Process removal
		if(remove_en)
			slot_valid[remove_iter]	<= 0;

	end


endmodule
