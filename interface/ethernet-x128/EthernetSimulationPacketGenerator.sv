`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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
	@brief Packet generator that creates XLGMII-64 packets from a PCAP file. Timestamps are ignored for now.
 */
module EthernetSimulationPacketGenerator #(
	parameter PCAP_FILE 			= "",
	parameter PACKET_INTERVAL		= 0			//Number of clocks (in the 128-bit domain) between packets.
												//Standard IFG for 40GbE is 12 bytes, but can be as low as one at RX.
)(
	input wire			clk_625mhz,
	output xlgmii64		bus = 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Endianness manipulation

	logic		need_swap = 0;

	function[31:0] FixEndianness32(input logic[31:0] data, input logic swap);
		begin
			if(swap)
				return { data[7:0], data[15:8], data[23:16], data[31:24] };
			else
				return data;
		end
	endfunction

	function[15:0] FixEndianness16(input logic[15:0] data, input logic swap);
		begin
			if(swap)
				return { data[7:0], data[15:8] };
			else
				return data;
		end
	endfunction

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CRC calculation

	function[31:0] UpdateCRC(input logic[63:0] data, input logic[31:0] oldcrc, input logic[3:0] len);
		logic[7:0]	current_byte;
		logic		lsb;
		logic[31:0]	crc;

		begin
			crc = oldcrc;

			for(integer nbyte=0; nbyte<8; nbyte++) begin
				if(nbyte < len) begin
					current_byte = data[(7-nbyte)*8 +: 8];

					for(integer nbit=0; nbit<8; nbit++) begin
						lsb 	= current_byte[nbit] ^ crc[31];
						crc		= { crc[30:0], lsb };

						crc[1]	= lsb ^ crc[1];
						crc[2]	= lsb ^ crc[2];
						crc[4]	= lsb ^ crc[4];
						crc[5]	= lsb ^ crc[5];
						crc[7]	= lsb ^ crc[7];
						crc[8]	= lsb ^ crc[8];
						crc[10] = lsb ^ crc[10];
						crc[11] = lsb ^ crc[11];
						crc[12] = lsb ^ crc[12];
						crc[16] = lsb ^ crc[16];
						crc[22] = lsb ^ crc[22];
						crc[23] = lsb ^ crc[23];
						crc[26] = lsb ^ crc[26];
					end
				end
			end

			return crc;
		end

	endfunction

	function[31:0] FlipCRCBits(input logic[31:0] crc);
		return
		{
			~crc[24], ~crc[25], ~crc[26], ~crc[27], ~crc[28], ~crc[29], ~crc[30], ~crc[31],
			~crc[16], ~crc[17], ~crc[18], ~crc[19], ~crc[20], ~crc[21], ~crc[22], ~crc[23],
			~crc[8],  ~crc[9],  ~crc[10], ~crc[11], ~crc[12], ~crc[13], ~crc[14], ~crc[15],
			~crc[0],  ~crc[1],  ~crc[2],  ~crc[3],  ~crc[4],  ~crc[5],  ~crc[6],  ~crc[7]
		};
	endfunction

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	logic[31:0] magic;
	logic[15:0] major;
	logic[15:0] minor;
	logic[31:0] reserved1;
	logic[31:0] reserved2;
	logic[31:0] maxlen;
	logic[31:0] linktype;

	logic[31:0] ts_sec;
	logic[31:0] ts_usec;
	logic[31:0] orig_len;
	logic[31:0] incl_len;

	integer bytes_left;
	integer bytes_read;
	integer padding_bytes_left = 0;

	logic[63:0]	read_buf;

	logic[31:0] crc;
	logic[31:0] crc_debug_flipped;

	localparam CRC_POLY = 32'h04c11db7;

	integer fp;
	initial begin

		//Link is idle on reset
		for(integer i=0; i<8; i++) begin
			bus.ctl[i]			= 1;
			bus.data[i*8 +: 8]	= XLGMII_CTL_IDLE;
		end

		$display("Initializing PCAP signal source...");

		fp = $fopen(PCAP_FILE, "rb");
		if(!fp)
			$fatal(0, "Couldn't open pcap file");

		//Read file header
		if(4 != $fread(magic, fp))
			$fatal("Failed to read header");
		if(magic == 32'ha1b2c3d4) begin
			need_swap = 0;
			$display("    File is native endianness");
		end
		else if(magic == 32'hd4c3b2a1) begin
			need_swap = 1;
			$display("    File needs endianness swap");
		end
		else
			$fatal("Bad magic number");

		//Sanity check version numbers
		if(2 != $fread(major, fp))
			$fatal(0, "Failed to read major version");
		if(2 != $fread(minor, fp))
			$fatal(0, "Failed to read minor version");
		major = FixEndianness16(major, need_swap);
		minor = FixEndianness16(minor, need_swap);
		$display("    PCAP file is version %0d.%0d", major, minor);

		//Ignore reserved fields
		if(4 != $fread(reserved1, fp))
			$fatal(0, "Failed to read reserved1");
		if(4 != $fread(reserved2, fp))
			$fatal(0, "Failed to read reserved2");

		//Max packet length
		if(4 != $fread(maxlen, fp))
			$fatal(0, "Failed to read maxlen");
		maxlen = FixEndianness32(maxlen, need_swap);
		$display("    Max packet length is %0d", maxlen);

		//Link type
		if(4 != $fread(linktype, fp))
			$fatal(0, "Failed to read linktype");
		linktype = FixEndianness32(linktype, need_swap);
		if(linktype != 1)
			$fatal(0, "Expected LINKTYPE_ETHERNET (32'h00000001), got %8x instead", linktype);

		//Done reading file headers. Wait 50 clocks for stuff to initialize.
		//TODO: make this parameterizable
		repeat(50)
			@(posedge clk_625mhz);

		//Read all of the packets in the file
		while(!$feof(fp)) begin

			//Read the pcaprec_hdr_t
			if(4 != $fread(ts_sec, fp))
				break;
			if(4 != $fread(ts_usec, fp))
				$fatal(0, "Failed to read ts_usec");
			if(4 != $fread(incl_len, fp))
				$fatal(0, "Failed to read incl_len");
			if(4 != $fread(orig_len, fp))
				$fatal(0, "Failed to read orig_len");
			ts_sec = FixEndianness32(ts_sec, need_swap);
			ts_usec = FixEndianness32(ts_usec, need_swap);
			incl_len = FixEndianness32(incl_len, need_swap);
			orig_len = FixEndianness32(orig_len, need_swap);

			if(orig_len != incl_len)
				$fatal(0, "Truncated packets not supported");

			//Reset FCS
			crc = 32'hffffffff;

			//Generate the start sequence
			bus.ctl[7]				= 1;
			bus.data[63:56]			= XLGMII_CTL_START;
			for(integer i=1; i<7; i++) begin
				bus.ctl[i]			= 0;
				bus.data[i*8 +: 8]	= 8'h55;
			end
			bus.ctl[0]				= 0;
			bus.data[7:0]			= 8'hd5;
			@(posedge clk_625mhz);

			//Read all full data blocks in the packet
			for(integer i=0; i<orig_len; i += 8) begin

				bytes_left = orig_len - i;

				if(bytes_left >= 8) begin

					if(8 != $fread(read_buf, fp))
						$fatal(0, "Failed to read packet data");

					crc = UpdateCRC(read_buf, crc, 8);
					crc_debug_flipped = FlipCRCBits(crc);

					bus.ctl = 0;
					bus.data = read_buf;
					@(posedge clk_625mhz);

				end

			end

			//Read the remaining data.
			//This is messier than it needs to be, because Vivado fails to handle part selects in $fread.
			//(At least 2019.2 and 2020.1 are known to be affected, but most likely all released versions to date.)
			//https://forums.xilinx.com/t5/Simulation-and-Verification/SystemVerilog-fread-into-bit-select-gives-incorrect-results/td-p/1173984
			bytes_left = orig_len % 8;
			case(bytes_left)
				0: begin
					bytes_read = 0;
					read_buf = 64'h0;
				end
				1: begin
					bytes_read = $fread(read_buf[7:0], fp);
					read_buf = { read_buf[7:0], 56'h0 };
				end
				2: begin
					bytes_read = $fread(read_buf[15:0], fp);
					read_buf = { read_buf[15:0], 48'h0 };
				end
				3: begin
					bytes_read = $fread(read_buf[23:0], fp);
					read_buf = { read_buf[23:0], 40'h0 };
				end
				4: begin
					bytes_read = $fread(read_buf[31:0], fp);
					read_buf = { read_buf[31:0], 32'h0 };
				end
				5: begin
					bytes_read = $fread(read_buf[39:0], fp);
					read_buf = { read_buf[39:0], 24'h0 };
				end
				6: begin
					bytes_read = $fread(read_buf[47:0], fp);
					read_buf = { read_buf[47:0], 16'h0 };
				end
				7: begin
					bytes_read = $fread(read_buf[55:0], fp);
					read_buf = { read_buf[55:0], 8'h0 };
				end
				//can never read all 64 bits
			endcase

			//Make sure we actually got it
			if(bytes_read != bytes_left)
				$fatal(0, "Failed to read end of packet data");

			//Now comes the fun part! If the packet is less than 60 bytes long (plus FCS), we have to add padding.
			if(orig_len < 60) begin

				padding_bytes_left = 60 - orig_len;
				//$display("Packet is %0d bytes long, minimum is 60 plus FCS. Need to add %0d padding bytes",
				//	orig_len, padding_bytes_left);

				//If we're adding a relatively small amount of padding (just finishing this block) it's easy.
				//Just declare the zeroes at the end of this block to be data and move on.
				if( (padding_bytes_left + bytes_read) <= 8)
					bytes_read = bytes_read + padding_bytes_left;

				//Nope, adding at least one block of padding.
				else begin

					//Current block is padded. CRC it and send it out.
					crc = UpdateCRC(read_buf, crc, 8);
					crc_debug_flipped = FlipCRCBits(crc);
					padding_bytes_left -= (8 - bytes_read);

					bus.ctl = 0;
					bus.data = read_buf;
					@(posedge clk_625mhz);

					//Clock out blocks of zeroes until we hit the last block
					while(padding_bytes_left >= 8) begin
						crc = UpdateCRC(64'h0, crc, 8);
						crc_debug_flipped = FlipCRCBits(crc);
						padding_bytes_left -= 8;

						bus.ctl = 0;
						bus.data = 64'h0;
						@(posedge clk_625mhz);

					end

					//Done, add any partial padding in the last block
					read_buf = 0;
					bytes_read = padding_bytes_left;
					bytes_left = bytes_read;

				end

			end

			//Calculate final CRC
			crc = UpdateCRC(read_buf, crc, bytes_read);
			crc_debug_flipped = FlipCRCBits(crc);
			crc = FlipCRCBits(crc);

			//Merge the CRC into the final read data and clock it out
			case(bytes_left)

				//Clean end of packet: CRC then terminate and idles
				0: begin
					bus.ctl = 8'h0f;
					bus.data = { crc, XLGMII_CTL_TERMINATE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE };
				end

				//Still a few bytes of data left, but we can fit the CRC and termination
				1: begin
					bus.ctl = 8'h07;
					bus.data = { read_buf[63:56], crc, XLGMII_CTL_TERMINATE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE };
				end
				2: begin
					bus.ctl = 8'h03;
					bus.data = { read_buf[63:48], crc, XLGMII_CTL_TERMINATE, XLGMII_CTL_IDLE };
				end
				3: begin
					bus.ctl = 8'h01;
					bus.data = { read_buf[63:40], crc, XLGMII_CTL_TERMINATE };
				end

				//Need to start trimming stuff (so we need another data word)
				4: begin
					bus.ctl = 8'h0;
					bus.data = { read_buf[63:32], crc};
				end
				5: begin
					bus.ctl = 8'h0;
					bus.data = { read_buf[63:24], crc[31:8]};
				end
				6: begin
					bus.ctl = 8'h0;
					bus.data = { read_buf[63:16], crc[31:16]};
				end
				7: begin
					bus.ctl = 8'h0;
					bus.data = { read_buf[63:8], crc[31:24]};
				end

			endcase
			@(posedge clk_625mhz);

			//If we have additional data to send, send it
			if(bytes_left >= 4) begin

				case(bytes_left)
					4: begin
						bus.ctl = 8'hff;
						bus.data =
						{
							XLGMII_CTL_TERMINATE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE,
							XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE
						};
					end

					5: begin
						bus.ctl = 8'h7f;
						bus.data =
						{
							crc[7:0], XLGMII_CTL_TERMINATE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE,
							XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE
						};
					end

					6: begin
						bus.ctl = 8'h3f;
						bus.data =
						{
							crc[15:0], XLGMII_CTL_TERMINATE, XLGMII_CTL_IDLE,
							XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE
						};
					end

					7: begin
						bus.ctl = 8'h1f;
						bus.data =
						{
							crc[23:0], XLGMII_CTL_TERMINATE,
							XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE, XLGMII_CTL_IDLE
						};
					end

				endcase

				@(posedge clk_625mhz);
			end

			//If we were a real 40GbE transmitter, we'd need to send a minimum of twelve idles.
			//But in simulation, assume worst case RX behavior with min-sized IFG and don't send anything.

		end

		$fclose(fp);

	end

endmodule
