/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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

`include "EthernetBus.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief Ethernet packet generator based on PCAP file

	Reads frames from a PCAP and outputs the next one on an EthernetRxBus each time the "next packet" flag goes high.

	Note that this bypasses the MAC and outputs layer 2 frames with no preamble or FCS.
 */
module PcapPacketGenerator #(
	parameter FILENAME = ""
)(
	input wire				clk,
	input wire				next,

	output EthernetRxBus	bus
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read the packet data

	integer hfile;

	logic[31:0]	blocktype;
	logic[31:0]	blocklen;

	initial begin
		$display("[%m] Opening pcap file %s", FILENAME);

		hfile = $fopen(FILENAME, "rb");

		//Read first block type (should be SHB)
		blocktype = ReadNative32(hfile);
		if(blocktype != 32'h0a0d0d0a) begin
			$display("Invalid Section Header Block");
			$finish;
		end

		//Read block length
		blocklen = ReadNative32(hfile);
		$display("[%m]     SHB is %0d bytes long", blocklen);

		//Process it
		ReadSHB(hfile);

		//Read and ignore trailing block length
		blocklen = ReadNative32(hfile);

		//Read and process blocks until we get to packet data
		while(1) begin

			//Read type and length of next block
			blocktype = ReadNative32(hfile);
			blocklen = ReadNative32(hfile);

			case(blocktype)

				//Interface Definition Block
				1: begin
				end

				default: begin
					$display("Unknown block type %d\n", blocktype);
				end

			endcase

			//for now, stop after first one
			break;
		end

	end

endmodule

/**
	@brief Read and process a section header block
 */
function ReadSHB(integer hfile);

	logic[31:0]	byteorder;
	logic[15:0] major;
	logic[15:0] minor;
	logic[63:0] section;

	logic[15:0] optid;

	//Read byte order
	//For now, assume file is little endian and reject anything else
	byteorder = ReadNative32(hfile);
	if(byteorder != 32'h1a2b3c4d) begin
		$display("[ReadSHB] Invalid byte order mark");
		$finish;
	end

	//Read major and minor versions
	major = ReadNative16(hfile);
	minor = ReadNative16(hfile);
	$display("[ReadSHB]         pcapng file version %0d.%0d", major, minor);

	//Read and discard section length
	$fread(section, hfile);

	//Read SHB options
	while(1) begin

		//Get option type and print it
		optid = ReadNative16(hfile);
		ReadSHBOption(hfile, optid);

		//Stop when we get an opt_endopt
		if(optid == 0)
			break;

	end	//end SHB option loop

endfunction

/**
	@brief Process a single SHB option
 */
function ReadSHBOption(integer hfile, logic[15:0] optid);

	logic[15:0] optlen;
	logic[7:0] tmp;

	case(optid)

		//opt_endopt
		0: begin
			$display("[ReadSHBOption]         opt_endopt");
		end

		//shb_hardware
		2: begin
			optlen = ReadNative16(hfile);

			$write("[ReadSHBOption]         shb_hardware = ");
			for(integer i=0; i<optlen; i=i+1) begin
				if(!$fread(tmp, hfile))
					break;

				$write("%c", tmp);
			end
			$write("\n");

		end

		//shb_os
		3: begin
			optlen = ReadNative16(hfile);

			$write("[ReadSHBOption]         shb_os = ");
			for(integer i=0; i<optlen; i=i+1) begin
				if(!$fread(tmp, hfile))
					break;
				$write("%c", tmp);
			end
			$write("\n");

		end

		//shb_userappl
		4: begin
			optlen = ReadNative16(hfile);

			$write("[ReadSHBOption]         shb_userappl = ");
			for(integer i=0; i<optlen; i=i+1) begin
				if(!$fread(tmp, hfile))
					break;
				$write("%c", tmp);
			end
			$write("\n");

		end

		//TODO: probably shouldn't be fatal
		default: begin
			$display("[ReadSHBOption]     Unknown SHB option %0d\n", optid);
			$finish;
		end

	endcase

	//read padding until we're 32-bit aligned
	while($ftell(hfile) & 3) begin
		if(!$fread(tmp, hfile))
			break;
	end

endfunction

/**
	@brief Reads a 32-bit native-endian value
 */
function[31:0] ReadNative32(integer hfile);

	logic[31:0] tmp;
	if(4 != $fread(tmp, hfile)) begin
		$display("unexpected EOF in ReadNative32");
		$finish;
	end

	ReadNative32 = EndianSwap32(tmp);

endfunction

/**
	@brief Reads a 16-bit native-endian value
 */
function[15:0] ReadNative16(integer hfile);

	logic[15:0] tmp;
	if(2 != $fread(tmp, hfile)) begin
		$display("unexpected EOF in ReadNative16");
		$finish;
	end

	ReadNative16 = EndianSwap16(tmp);

endfunction

function[31:0] EndianSwap32(logic[31:0] din);
	EndianSwap32 = {din[7:0], din[15:8], din[23:16], din[31:24]};
endfunction

function[31:0] EndianSwap16(logic[15:0] din);
	EndianSwap16 = {din[7:0], din[15:8]};
endfunction
