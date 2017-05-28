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
	@brief Arbiter for deciding who gets to send where and when

	Arbiter is attached to a destination port
 */
module RPCv3RouterArbiter #(
	parameter TOTAL_PORTS 		= 4,

	//True if we have one child port, and it's a trunk
	parameter CHILD_IS_TRUNK	= 1'b0,

	//Index of the destination port this arbiter is attached to
	parameter THIS_PORT			= 0,

	//Coordinates of this router in the grid.
	//The base address of this router is {X_POS, Y_POS, 8'h00}.
	parameter X_POS 						= 4'h0,
	parameter Y_POS 						= 4'h0
) (

	input wire		clk,

	//Indicates the corresponding dst_addr is OK to look at
	input wire[TOTAL_PORTS-1 : 0]		rx_dst_valid,

	//The address each source port wants to send to
	input wire[16*TOTAL_PORTS - 1 : 0]	rx_dst_addr,

	//Indicates src_port is OK to look at
	output reg							tx_src_valid	= 0,

	//The addressthis output port should get its data from
	output reg[8:0]						tx_src_port		= 0,

	//Set high for one cycle at end of a transmit to indicate we should bump the counter
	input wire							tx_done
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Round robin counter

	reg[8:0] 	rr_count	= 0;
	wire[8:0]	rr_count_inc	= rr_count + 1'h1;
	always @(posedge clk) begin

		if(tx_done) begin
			if(rr_count_inc >= TOTAL_PORTS)
				rr_count	<= 0;
			else
				rr_count	<= rr_count_inc;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// See who wants to send to us

	//Our base address
	localparam BASE_ADDR				= {X_POS, Y_POS, 8'h00};

	//Index of our port within the children (if we're a child)
	localparam CHILD_PORT 				= THIS_PORT - 4;

	//True if the port's destination is pointing to us (may not be valid, though)
	reg[TOTAL_PORTS-1:0]	port_match	= 0;

	genvar i;
	generate

		for(i=0; i<TOTAL_PORTS; i=i+1) begin : sendloop

			//Pull out some useful address bits
			wire[15:0]	rx_dst	= rx_dst_addr[i*16 +: 16];
			wire[3:0]	rx_xpos	= rx_dst[15:12];
			wire[3:0]	rx_ypos	= rx_dst[11:8];

			//Pseudorandom value that is static for each destination (to ensure deterministic routing)
			//We cannot use THIS_PORT in addrxor to ensure deterministic matching!
			//in particular, it's important that ``random" have the same value for all four neighbor ports
			//given the same destination.
			wire[15:0]	addrxor	= rx_dst ^ BASE_ADDR[15:8];
			wire		random	= ^addrxor;

			always @(*) begin

				//Clean up
				port_match[i]	<= 0;

				case(THIS_PORT)

					//north
					0: begin

						//If destination is not to our north (dest Y > ours), we're not interested
						if(rx_ypos <= Y_POS)
							port_match[i]	<= 0;

						//It's north, but might also be east/west
						else begin

							//If destination is due north, we obviously route north
							if(rx_xpos == X_POS)
								port_match[i]	<= 1;

							//Destination is northeast or northwest.
							//Route north randomly for 50% of destinations
							else if(random)
								port_match[i]	<= 1;

							//Nope, don't match
							else
								port_match[i]	<= 0;
						end

					end

					//south
					1: begin

						//If destination is not to our south (dest Y < ours), we're not interested
						if(rx_ypos >= Y_POS)
							port_match[i]	<= 0;

						//It's south, but might also be east/west
						else begin

							//If destination is due south, we obviously route south
							if(rx_xpos == X_POS)
								port_match[i]	<= 1;

							//Destination is southeast or southwest.
							//Route south randomly for 50% of destinations
							else if(random)
								port_match[i]	<= 1;

							//Nope, don't match
							else
								port_match[i]	<= 0;
						end

					end

					//east
					2: begin

						//If destination is not to our east (dest X > ours), we're not interested
						if(rx_xpos <= X_POS)
							port_match[i]	<= 0;

						//It's east, but might also be north/south
						else begin

							//If destination is due east, we obviously route east
							if(rx_ypos == Y_POS)
								port_match[i]	<= 1;

							//Destination is northeast or southeast
							//Route east randomly for 50% of destinations
							//(those not matched by random north/south)
							else if(!random)
								port_match[i]	<= 1;

							//Nope, don't match
							else
								port_match[i]	<= 0;
						end

					end

					//west
					3: begin

						//If destination is not to our west (dest X < ours), we're not interested
						if(rx_xpos >= X_POS)
							port_match[i]	<= 0;

						//It's west, but might also be north/south
						else begin

							//If destination is due west, we obviously route west
							if(rx_ypos == Y_POS)
								port_match[i]	<= 1;

							//Destination is northwest or southwest
							//Route west randomly for 50% of destinations
							//(those not matched by random north/south)
							else if(!random)
								port_match[i]	<= 1;

							//Nope, don't match
							else
								port_match[i]	<= 0;
						end

					end

					//Child interface
					default: begin

						//We're a trunk port. Match anything in our subnet.
						if(CHILD_IS_TRUNK)
							port_match[i]	<= (rx_dst[15:8] == BASE_ADDR[15:8]);

						//We're a child port. Match exact child address
						else
							port_match[i]	<= (rx_dst == {BASE_ADDR[15:8], CHILD_PORT[7:0]});

					end

				endcase

			end

		end

	endgenerate

	//True if the port's destination is pointing to us AND they're valid
	wire[TOTAL_PORTS-1:0]	port_sending_to_us	= port_match & rx_dst_valid;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decide where to send from

	always @(posedge clk) begin
		//tx_src_valid	<= 0;

		//

	end

endmodule
