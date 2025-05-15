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
	@file
	@author Andrew D. Zonenberg
	@brief Ethernet packet generator based on PCAP file

	Reads frames from a PCAP and outputs the next one on an AXI4-Stream bus each time the "next packet" flag goes high.

	Note that this bypasses the MAC and outputs layer 2 frames with no preamble or FCS.
 */
module AXIS_PcapngPacketGenerator #(
	parameter FILENAME = ""
)(
	input wire				clk,
	input wire				next,

	AXIStream.transmitter	axi_tx
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Hook up AXI control signals

	assign axi_tx.aclk		= clk;
	assign axi_tx.twakeup	= 1;
	assign axi_tx.tuser		= 1;	//we never have bad packets in the pcap
									//TODO: where should we inject bad FCSes to test?

	initial begin
		axi_tx.areset_n		= 0;
		axi_tx.tdata		= 0;
		axi_tx.tkeep		= 0;
		axi_tx.tstrb		= 0;
		axi_tx.tvalid		= 0;
		axi_tx.tlast		= 0;
	end

	always_ff @(posedge clk) begin
		axi_tx.areset_n		<= 1;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read the pcap header

	integer hfile;

	logic[31:0]	blocktype;
	logic[31:0]	blockstart;
	logic[31:0]	blocklen;
	logic[31:0]	rblocklen;

	initial begin
		$display("[%m] Opening pcapng file %s", FILENAME);

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

		//Verify trailing redundant block length
		rblocklen = ReadNative32(hfile);
		if(rblocklen != blocklen) begin
			$display("Invalid redundant block length (malformed file or parser bug)");
			$finish;
		end

		//Read and process blocks until we get to packet data
		while(1) begin

			//Read type and length of next block
			blockstart = $ftell(hfile);
			blocktype = ReadNative32(hfile);
			blocklen = ReadNative32(hfile);

			case(blocktype)

				//Interface Definition Block
				1: begin
					$display("[%m]     IDB is %0d bytes long", blocklen);
					ReadIDB(hfile);
				end

				//Enhanced Packet Block
				6: begin
				end

				default: begin
					$display("Unknown block type %d", blocktype);
					$finish;
				end

			endcase

			//If we got to an EPB, stop - we're ready to read the packet
			if(blocktype == 6)
				break;

			//Verify trailing redundant block length
			rblocklen = ReadNative32(hfile);
			if(rblocklen != blocklen) begin
				$display("Invalid redundant block length (malformed file or parser bug)");
				$finish;
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main packet generation state machine

	enum logic[2:0]
	{
		STATE_FIRST,
		STATE_IDLE,
		STATE_DATA,
		STATE_FINISH,
		STATE_EOF
	} state = STATE_FIRST;

	logic[31:0]	iid 			= 0;
	logic[31:0]	ts_hi 			= 0;
	logic[31:0]	ts_lo 			= 0;
	logic[31:0]	pack_len 		= 0;
	logic[31:0] original_len 	= 0;
	logic[31:0]	count			= 0;
	logic[31:0] rdbuf			= 0;

	logic[31:0] epbstart		= 0;
	logic[31:0]	epblen			= 0;
	logic[31:0]	epbrlen			= 0;
	logic[31:0]	epbtype			= 0;

	logic[15:0]	optname			= 0;
	logic[15:0] optlen			= 0;

	always_ff @(posedge clk) begin

		//If an existing transaction is still active, keep it open
		if(axi_tx.tvalid && !axi_tx.tready) begin
		end

		//Nope, nothing active. Clear the bus
		else begin
			axi_tx.tvalid	<= 0;
			axi_tx.tdata	<= 0;
			axi_tx.tlast	<= 0;
			axi_tx.tstrb	<= 0;
			axi_tx.tkeep	<= 0;
		end

		case(state)

			STATE_FIRST: begin
				epbstart			= blockstart;
				epblen				= blocklen;
				state				<= STATE_IDLE;
			end

			STATE_IDLE: begin

				//Ready to start a new packet!
				//Read EPB headers
				if(next) begin

					//Interface ID (ignored for now since we only have one output interface)
					iid				= ReadNative32(hfile);

					//Timestamp (ignored since we replay packets on cue rather than real time)
					ts_hi			= ReadNative32(hfile);
					ts_lo			= ReadNative32(hfile);

					//Packet length in the file (this is what we're replaying)
					pack_len		= ReadNative32(hfile);

					//Original packet length (same unless truncated by short snap len)
					//If these mismatch, print a warning since we're replaying an incomplete packet
					original_len	= ReadNative32(hfile);
					if(pack_len != original_len) begin
						$display("[%m] WARNING: replaying truncated packet (%0d bytes captured, %0d bytes on wire)",
						pack_len, original_len);
					end

					//Starting a new packet
					count			<= 0;

					//Reading frame data next cycle
					state			<= STATE_DATA;

				end

			end	//STATE_IDLE

			STATE_DATA: begin

				//RX not ready? Stall
				if(axi_tx.tvalid && !axi_tx.tready) begin

				end

				//Do we have a full 32 bits of data to send? If so, output it all
				else if( (count + 4) <= pack_len) begin

					if(4 != $fread(rdbuf, hfile)) begin
						$display("[%m] unexpected EOF");
						state		<= STATE_EOF;
					end

					axi_tx.tvalid	<= 1;
					axi_tx.tkeep	<= 4'b1111;
					axi_tx.tstrb	<= 4'b1111;
					axi_tx.tdata	<= { rdbuf[7:0], rdbuf[15:8], rdbuf[23:16], rdbuf[31:24] };

					count			<= count + 4;
				end

				//No, send whatever is left
				else if(count < pack_len) begin

					axi_tx.tvalid	<= 1;
					axi_tx.tkeep	<= 4'b1111;

					case(pack_len - count)

						1: begin
							if(1 != $fread(rdbuf[7:0], hfile)) begin
								$display("[%m] unexpected EOF");
								state		<= STATE_EOF;
							end

							axi_tx.tstrb	<= 4'b0001;
							axi_tx.tdata	<= {24'h0, rdbuf[7:0]};
						end

						2: begin
							if(2 != $fread(rdbuf[15:0], hfile)) begin
								$display("[%m] unexpected EOF");
								state		<= STATE_EOF;
							end

							axi_tx.tstrb	<= 4'b0011;
							axi_tx.tdata	<= {16'h0, rdbuf[7:0], rdbuf[15:8]};
						end

						3: begin
							if(3 != $fread(rdbuf[23:0], hfile)) begin
								$display("[%m] unexpected EOF");
								state		<= STATE_EOF;
							end

							axi_tx.tstrb	<= 4'b0111;
							axi_tx.tdata	<= {8'h0, rdbuf[7:0], rdbuf[15:8], rdbuf[23:16]};
						end

					endcase

					state			<= STATE_FINISH;

				end

				//Last byte
				else
					state			<= STATE_FINISH;

			end	//STATE_DATA

			STATE_FINISH: begin

				//RX not ready? Stall
				if(axi_tx.tvalid && !axi_tx.tready) begin
				end

				else begin
					axi_tx.tvalid	<= 1;
					axi_tx.tlast	<= 1;

					//read padding until we're 32-bit aligned
					while($ftell(hfile) & 3) begin
						if(!$fread(rdbuf[7:0], hfile))
							break;
					end

					//Read options if we had space for any
					if( ($ftell(hfile) + 4) < (epbstart + epblen) ) begin

						while(1) begin

							if($feof(hfile)) begin
								state	<= STATE_EOF;
								break;
							end

							optname = ReadNative16(hfile);
							$display("got EPB option %0d", optname);

							if(optname != 0) begin
								optlen = ReadNative16(hfile);
								$display("length is %0d", optlen);
							end

							//discard block content
							for(integer i=0; i<optlen; i=i+1) begin
								if(!$fread(rdbuf[7:0], hfile))
									break;
							end

							//read padding until we're 32-bit aligned
							while($ftell(hfile) & 3) begin
								if(!$fread(rdbuf[7:0], hfile))
									break;
							end

							if(optname == 0)
								break;
						end

					end

					//Verify trailing redundant block length
					epbrlen = ReadNative32(hfile);
					if(epbrlen != epblen) begin
						$display("Invalid redundant EPB length (malformed file or parser bug)");
						$finish;
					end

					//read padding until we're 32-bit aligned
					while($ftell(hfile) & 3) begin
						if(!$fread(rdbuf[7:0], hfile))
							break;
					end

					if($feof(hfile)) begin
						$display("[%m] Normal end of pcap file");
						state		<= STATE_EOF;
					end

					else begin

						//Read the next EPB header
						epbstart = $ftell(hfile);
						epbtype = ReadNative32NoEOFAbort(hfile);

						if($feof(hfile)) begin
							$display("[%m] Normal end of pcap file");
							state		<= STATE_EOF;
						end

						else begin
							epblen = ReadNative32(hfile);
							if(epbtype != 6) begin
								$display("[%m] Unrecognized block type %d, stopping packet generation", epbtype);
								state	<= STATE_EOF;
							end

							else
								state	<= STATE_IDLE;
						end

					end
				end

			end	//STATE_FINISH

			STATE_EOF: begin
			end	//STATE_EOF

		endcase

	end

endmodule

/**
	@brief Read and process an interface definition block
 */
function void ReadIDB(integer hfile);

	logic[15:0] linktype;
	logic[15:0] reserved;
	logic[31:0] snaplen;

	logic[15:0]	optid;

	//Read link type
	linktype = ReadNative16(hfile);
	if(linktype == 16'h0001)
		$display("[ReadIDB]     Link type is Ethernet");
	else begin
		$display("[ReadIDB] Link type is not Ethernet, cannot replay this pcapng");
		$finish;
	end

	//Skip two reserved bytes
	reserved = ReadNative16(hfile);

	//Snap length
	snaplen = ReadNative32(hfile);
	$display("[ReadIDB]     Snap length is %d bytes", snaplen);

	//Read IDB options
	while(1) begin

		//Get option type and print it
		optid = ReadNative16(hfile);
		ReadIDBOption(hfile, optid);

		//Stop when we get an opt_endopt
		if(optid == 0)
			break;

	end

endfunction

/**
	@brief Process a single IDB option
 */
function void ReadIDBOption(integer hfile, logic[15:0] optid);

	logic[15:0] optlen;
	logic[7:0] tmp;


	case(optid)

		//opt_endopt
		0: begin
			//$display("[ReadIDBOption]         opt_endopt");
		end

		//if_name
		2: begin
			optlen = ReadNative16(hfile);

			$write("[ReadIDBOption]         if_name = ");
			for(integer i=0; i<optlen; i=i+1) begin
				if(!$fread(tmp, hfile))
					break;

				$write("%c", tmp);
			end
			$write("\n");

		end

		//if_description
		3: begin
			optlen = ReadNative16(hfile);

			$write("[ReadIDBOption]         if_description = ");
			for(integer i=0; i<optlen; i=i+1) begin
				if(!$fread(tmp, hfile))
					break;
				$write("%c", tmp);
			end
			$write("\n");

		end

		//if_tsresol
		9: begin
			optlen = ReadNative16(hfile);

			if(!$fread(tmp, hfile)) begin
				$display("[ReadIDBOption] unexpected eof");
				$finish;
			end

			$display("[ReadIDBOption]         if_tsresol = %d", tmp);

		end

		//if_filter
		16'h0b: begin
			optlen = ReadNative16(hfile);

			$write("[ReadIDBOption]         if_filter = ");
			for(integer i=0; i<optlen; i=i+1) begin
				if(!$fread(tmp, hfile))
					break;
				$write("%c", tmp);
			end
			$write("\n");
		end

		//if_os
		16'h0c: begin
			optlen = ReadNative16(hfile);

			$write("[ReadIDBOption]         if_os = ");
			for(integer i=0; i<optlen; i=i+1) begin
				if(!$fread(tmp, hfile))
					break;
				$write("%c", tmp);
			end
			$write("\n");

		end

		//TODO: probably shouldn't be fatal
		default: begin
			$display("[ReadIDBOption]     Unknown IDB option %0d\n", optid);
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
	@brief Read and process a section header block
 */
function void ReadSHB(integer hfile);

	logic[31:0]	byteorder;
	logic[15:0] major;
	logic[15:0] minor;
	logic[63:0] section;

	logic[15:0] optid;

	integer unused;

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
	unused = $fread(section, hfile);

	//Read SHB options
	while(1) begin

		//Get option type and print it
		optid = ReadNative16(hfile);
		ReadSHBOption(hfile, optid);

		//Stop when we get an opt_endopt
		if(optid == 0)
			break;

	end

endfunction

/**
	@brief Process a single SHB option
 */
function void ReadSHBOption(integer hfile, logic[15:0] optid);

	logic[15:0] optlen;
	logic[7:0] tmp;

	case(optid)

		//opt_endopt
		0: begin
			//$display("[ReadSHBOption]         opt_endopt");
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
	@brief Reads a 32-bit native-endian value but don't abort if we hit end of file
 */
function[31:0] ReadNative32NoEOFAbort(integer hfile);

	logic[31:0] tmp;
	if(4 != $fread(tmp, hfile)) begin
		ReadNative32NoEOFAbort = 0;
	end
	else
		ReadNative32NoEOFAbort = EndianSwap32(tmp);

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
