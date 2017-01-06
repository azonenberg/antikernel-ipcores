`default_nettype none
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
	@brief Transceiver for RPC network, protocol version 3

	Messages are fixed size 128-bit datagrams.

	Flow control:
		Transmitter waits for rpc_rx_ready to be asserted
		Transmitter asserts rpc_tx_en and sends the first DATA_WIDTH bits of the message
		Transmitter sends remaining data, if any, on consecutive clock edges.

	No inter-frame gap is required, the next message may be sent immediately if rpc_rx_ready is still asserted.

	The packet consists of eight 16-bit words. When DATA_WIDTH=16 the message is sent one word per clock.
	Large DATA_WIDTH values send messages two, four, or eight words at a time.
		Word 0: tx_dst_addr
		Word 1: NODE_ADDR (automatically added)
		Word 2: {tx_callnum, tx_type, tx_d0[20:16]}
		Word 3: tx_d0[15:0]
		Word 4: tx_d1[31:16]
		Word 5: tx_d1[15:0]
		Word 6: tx_d2[31:16]
		Word 7: tx_d2[15:0]

	To transmit a message:
		Load rpc_fab_tx_* with the message to be sent
		Assert rpc_fab_tx_en for one cycle
		rpc_fab_tx_done goes high combinatorially for one cycle as the last word of the message is sent.
		Do not change rpc_fab_tx_* until rpc_fab_tx_done goes low.
		rpc_fab_tx_busy is high during the entire period of a send, including the "wait for rx_ready" phase.

		Note that if DATA_WIDTH=128 and rpc_tx_ready is set, rpc_fab_tx_done will be asserted combinatorially.

	To receive a message:
		Assert rpc_fab_rx_ready for one cycle
		Wait for rpc_fab_rx_en to go high and process rpc_fab_rx_*
		If still ready to receive another message, keep rpc_fab_rx_ready high.

		Note that two messages can arrive on consecutive clocks if DATA_WIDTH=128 and rpc_fab_rx_ready is held high,
		so any node doing this must have a 2+ message receive FIFO.

	RESOURCE USAGE (XST A7 post synthesis estimate, TX side)
		Quiet
			Width			FF			LUT
			16				5			56
			32				3			56
			64				4			68
			128				2			115

		Noisy
			Width			FF			LUT
			16				5			54
			32				3			39
			64				3			68
			128				1			3

		RPCv2 (for comparison)
			32				7			73

	RESOURCE USAGE (XST A7 post synthesis estimate, RX side)
		RPCv3
			Width			FF			LUT
			16				135			136
			32				134			136
			64				132			67
			128				130			2

		RPCv2 (for comparison)
			32				135			135
 */
module RPCv3Transceiver
#(
	//Data width (must be one of 16, 32, 64, 128).
	parameter DATA_WIDTH = 32,

	//When zero, queued data is allowed to show up on rpc_tx_data in between packets.
	//This can cause extra switching power consumption and clutters LA traces, but uses less LUTs.
	//When nonzero, force rpc_tx_data to zero when not transmitting.
	parameter QUIET_WHEN_IDLE = 1,

	//This transceiver is always a leaf (node) port, no exceptions.
	//We always send from this address.
	parameter NODE_ADDR = 16'h8000
) (
	//Clock for this link
	input wire					clk,

	//Network interface, outbound side
	output reg					rpc_tx_en,
	output reg[DATA_WIDTH-1:0]	rpc_tx_data,
	input wire					rpc_tx_ready,

	//Network interface, inbound side
	input wire					rpc_rx_en,
	input wire[DATA_WIDTH-1:0]	rpc_rx_data,
	output reg					rpc_rx_ready,

	//Fabric interface, outbound side
	input wire					rpc_fab_tx_en,
	output reg					rpc_fab_tx_busy,
	input wire[15:0]			rpc_fab_tx_dst_addr,
	input wire[7:0]				rpc_fab_tx_callnum,
	input wire[2:0]				rpc_fab_tx_type,
	input wire[20:0]			rpc_fab_tx_d0,
	input wire[31:0]			rpc_fab_tx_d1,
	input wire[31:0]			rpc_fab_tx_d2,
	output reg					rpc_fab_tx_done,

	//Fabric interface, inbound side
	input wire					rpc_fab_rx_ready,
	output reg 					rpc_fab_rx_busy,
	output reg					rpc_fab_rx_en		= 0,
	output reg[15:0]			rpc_fab_rx_src_addr	= 0,
	output reg[15:0]			rpc_fab_rx_dst_addr	= 0,
	output reg[7:0]				rpc_fab_rx_callnum	= 0,
	output reg[2:0]				rpc_fab_rx_type		= 0,
	output reg[20:0]			rpc_fab_rx_d0		= 0,
	output reg[31:0]			rpc_fab_rx_d1		= 0,
	output reg[31:0]			rpc_fab_rx_d2		= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synthesis-time sanity checking

	initial begin
		case(DATA_WIDTH)
			16: begin
			end

			32: begin
			end

			64: begin
			end

			128: begin
			end

			default: begin
				$display("ERROR: RPCv3Transceiver DATA_WIDTH must be 16/32/64/128");
				$finish;
			end

		endcase
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Compute some useful values

	//Number of clocks it takes to send a message
	localparam MESSAGE_CYCLES = 128 / DATA_WIDTH;

	//Number of bits we need in the cycle counter
	`include "../../synth_helpers/clog2.vh"
	localparam CYCLE_BITS = clog2(MESSAGE_CYCLES);
	localparam CYCLE_MAX = CYCLE_BITS ? CYCLE_BITS-1 : 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit path

	//Words of data to send
	wire[15:0]	tx_d0				= rpc_fab_tx_dst_addr;
	wire[15:0]	tx_d1				= NODE_ADDR;
	wire[15:0]	tx_d2				= {rpc_fab_tx_callnum, rpc_fab_tx_type, rpc_fab_tx_d0[20:16]};
	wire[15:0]	tx_d3				= rpc_fab_tx_d0[15:0];
	wire[15:0]	tx_d4				= rpc_fab_tx_d1[31:16];
	wire[15:0]	tx_d5				= rpc_fab_tx_d1[15:0];
	wire[15:0]	tx_d6				= rpc_fab_tx_d2[31:16];
	wire[15:0]	tx_d7				= rpc_fab_tx_d2[15:0];

	//If we couldn't send the first cycle, remember that we have a send queued up
	reg			tx_pending			= 0;

	//True if we want to send (either current or queued transmit request)
	wire		tx_request			= tx_pending || rpc_fab_tx_en;

	//True if we're starting a transmit this cycle (whether queued or fresh)
	wire		tx_starting			= tx_request && rpc_tx_ready;

	//Position within the message (in DATA_WIDTH-bit units)
	reg[CYCLE_MAX:0] tx_count	= 0;

	//True if a transmit is in progress
	wire		tx_active			= (tx_count != 0) || tx_starting;

	generate

		//128-bit datapath is stupidly simple, special-case it
		if(DATA_WIDTH == 128) begin

			//All transmit logic is combinatorial
			always @(*) begin

				//Busy if we're sending or waiting to send
				rpc_fab_tx_busy <= (tx_pending || (tx_count != 0) || tx_starting );

				//Send the message if we're starting to send
				rpc_tx_en		<= tx_starting;

				//We finish sending the same cycle we send
				rpc_fab_tx_done	<= tx_starting;

				//Optionally squash output when not sending
				if(QUIET_WHEN_IDLE && !tx_active)
					rpc_tx_data	<= {DATA_WIDTH{1'b0}};

				//Nope, send it
				else
					rpc_tx_data <= {tx_d0, tx_d1, tx_d2, tx_d3, tx_d4, tx_d5, tx_d6, tx_d7};

			end

			//One little bit of stateful logic, though :)
			always @(posedge clk) begin

				//Clear pending messages once sent
				if(tx_starting)
					tx_pending	<= 0;

				//If we try to send when rx isn't ready, save it until they are
				if(rpc_fab_tx_en && !rpc_tx_ready)
					tx_pending	<= 1;

			end

		end

		//All other datapaths take >1 cycle to send
		else begin

			//Combinatorial transmit
			always @(*) begin

				//We're busy if we want to send, or are sending.
				rpc_fab_tx_busy <= (tx_pending || (tx_count != 0) || tx_starting );

				//Send the message if we're starting to send
				rpc_tx_en		<= tx_starting && (tx_count == 0);

				//Optionally squash output when not sending
				if(QUIET_WHEN_IDLE && !tx_active)
					rpc_tx_data	<= {DATA_WIDTH{1'b0}};

				//Nope, send it and assert the done flag on the last clock
				else if(DATA_WIDTH == 64) begin
					case(tx_count)
						1:			rpc_tx_data <= { tx_d4, tx_d5, tx_d6, tx_d7 };
						default:	rpc_tx_data <= { tx_d0, tx_d1, tx_d2, tx_d3 };
					endcase
				end
				else if(DATA_WIDTH == 32) begin
					case(tx_count)
						3:			rpc_tx_data <= { tx_d6, tx_d7 };
						2:			rpc_tx_data <= { tx_d4, tx_d5 };
						1:			rpc_tx_data <= { tx_d2, tx_d3 };
						default:	rpc_tx_data <= { tx_d0, tx_d1 };
					endcase
				end
				else begin
					case(tx_count)
						7:			rpc_tx_data <= tx_d7;
						6:			rpc_tx_data <= tx_d6;
						5:			rpc_tx_data <= tx_d5;
						4:			rpc_tx_data <= tx_d4;
						3:			rpc_tx_data <= tx_d3;
						2:			rpc_tx_data <= tx_d2;
						1:			rpc_tx_data <= tx_d1;
						default:	rpc_tx_data <= tx_d0;
					endcase
				end

				//Set done flag at end of message
				rpc_fab_tx_done		<= (tx_count == {CYCLE_BITS{1'b1}});

			end

			//More state logic needed to keep track of our phase etc
			always @(posedge clk) begin

				//Clear pending messages once sent
				if(tx_active)
					tx_pending	<= 0;

				//If we try to send when rx isn't ready, save it until they are
				if(rpc_fab_tx_en && !rpc_tx_ready)
					tx_pending	<= 1;

				//Increment word counter
				if(tx_active)
					tx_count	<= tx_count + 1'h1;

			end

		end

	endgenerate

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive path

	//True if we are in the first cycle of an incoming message
	wire					rx_starting		= (rpc_rx_en && rpc_rx_ready);

	//Position within the message (in DATA_WIDTH-bit units)
	reg[CYCLE_MAX:0]		rx_count		= 0;

	//True if a receive is in progress
	wire					rx_active;

	generate

		if(DATA_WIDTH == 128)
			assign rx_active = rx_starting;

		else
			assign rx_active = (rx_count != 0) || rx_starting;

		//Set busy flag if we're doing stuff (mostly used by formal stuff)
		always @(*) begin
			rpc_fab_rx_busy		<= (rx_count != 0);
		end

		always @(posedge clk) begin

			rpc_fab_rx_en		<= 0;

			//Process incoming data words
			if(rx_active) begin

				//Whole message is on the wire right now
				if(DATA_WIDTH == 128) begin
					rpc_fab_rx_dst_addr	<= rpc_rx_data[127:112];
					rpc_fab_rx_src_addr	<= rpc_rx_data[111:96];
					rpc_fab_rx_callnum	<= rpc_rx_data[95:88];
					rpc_fab_rx_type		<= rpc_rx_data[87:85];
					rpc_fab_rx_d0		<= rpc_rx_data[84:64];
					rpc_fab_rx_d1		<= rpc_rx_data[63:32];
					rpc_fab_rx_d2		<= rpc_rx_data[31:0];

					//end of message
					rpc_fab_rx_en		<= 1;
				end

				else begin

					//Update word count as we move through the message
					if(rx_starting)
						rx_count		<= 1;
					else
						rx_count		<= rx_count + 1;

					//Grab whatever fields are currently on the wire
					case(rx_count)

						0: begin

							case(DATA_WIDTH)

								16:	rpc_fab_rx_dst_addr	<= rpc_rx_data[15:0];

								32: begin
									rpc_fab_rx_dst_addr	<= rpc_rx_data[31:16];
									rpc_fab_rx_src_addr	<= rpc_rx_data[15:0];
								end

								64: begin
									rpc_fab_rx_dst_addr	<= rpc_rx_data[63:48];
									rpc_fab_rx_src_addr	<= rpc_rx_data[47:32];
									rpc_fab_rx_callnum	<= rpc_rx_data[31:24];
									rpc_fab_rx_type		<= rpc_rx_data[23:21];
									rpc_fab_rx_d0		<= rpc_rx_data[20:0];
								end

							endcase

						end

						1: begin
							case(DATA_WIDTH)
								16:	begin
									rpc_fab_rx_src_addr	<= rpc_rx_data;
								end

								32: begin
									rpc_fab_rx_callnum	<= rpc_rx_data[31:24];
									rpc_fab_rx_type		<= rpc_rx_data[23:21];
									rpc_fab_rx_d0		<= rpc_rx_data[20:0];
								end

								64: begin
									rpc_fab_rx_d1		<= rpc_rx_data[63:32];
									rpc_fab_rx_d2		<= rpc_rx_data[31:0];

									//end of message
									rpc_fab_rx_en		<= 1;
								end

							endcase
						end

						2: begin
							if(DATA_WIDTH == 32)
								rpc_fab_rx_d1			<= rpc_rx_data;
							else begin
								rpc_fab_rx_callnum		<= rpc_rx_data[15:8];
								rpc_fab_rx_type			<= rpc_rx_data[7:5];
								rpc_fab_rx_d0[20:16]	<= rpc_rx_data[4:0];
							end
						end

						3: begin
							if(DATA_WIDTH == 32) begin
								rpc_fab_rx_d2			<= rpc_rx_data;

								//end of message
								rpc_fab_rx_en		<= 1;
							end
							else
								rpc_fab_rx_d0[15:0]		<= rpc_rx_data;
						end

						//If we get here we're a 16-bit message, so no more conditionals needed
						4: rpc_fab_rx_d1[31:16]	<= rpc_rx_data;
						5: rpc_fab_rx_d1[15:0]	<= rpc_rx_data;
						6: rpc_fab_rx_d2[31:16]	<= rpc_rx_data;
						7: begin
							rpc_fab_rx_d2[15:0]	<= rpc_rx_data;

							//end of message
							rpc_fab_rx_en		<= 1;
						end

					endcase
				end
			end

		end

		//Ready to receive if the fabric side is ready.
		//Once we go ready, go un-ready when a message comes in unless we keep ready high.
		reg		rpc_rx_ready_ff	= 0;
		always @(posedge clk) begin
			if(rpc_rx_en)
				rpc_rx_ready_ff		<= 0;
			if(rpc_fab_rx_ready)
				rpc_rx_ready_ff		<= 1;
		end

		always @(*) begin
			rpc_rx_ready		<= rpc_rx_ready_ff;
			if(rpc_rx_en)
				rpc_rx_ready	<= 0;
			if(rpc_fab_rx_ready)
				rpc_rx_ready	<= 1;
		end

	endgenerate

endmodule
