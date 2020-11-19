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
	@brief Parses Ethernet frame headers and filters by target MAC address
 */
module EthernetFrameParser(
	input wire					clk,
	input wire EthernetRxBus	mac_bus,

	input wire					promisc_mode,
	input wire[47:0]			our_mac_address,

	output EthernetHeaders		headers		= 0,
	output EthernetRxBus		frame_bus	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline incoming data

	EthernetRxBus	mac_bus_ff = 0;

	always_ff @(posedge clk) begin
		mac_bus_ff	<= mac_bus;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Frame parsing

	enum logic[2:0]
	{
		STATE_IDLE,
		STATE_SECOND,
		STATE_THIRD,
		STATE_TAGGED_BODY,
		STATE_UNTAGGED_BODY
	} state = STATE_IDLE;

	logic	send_untagged_last	= 0;
	logic	send_tagged_last	= 0;

	always_ff @(posedge clk) begin

		frame_bus			<= 0;
		send_untagged_last	<= 0;
		send_tagged_last	<= 0;

		case(state)

			//Wait for a frame to start
			STATE_IDLE: begin

				//Send the last 1-2 bytes of an untagged frame
				if(send_untagged_last) begin
					frame_bus.data_valid		<= 1;
					frame_bus.data				<= { mac_bus_ff.data[15:0], 112'h0 };
					frame_bus.bytes_valid		<= mac_bus_ff.bytes_valid - 14;
					frame_bus.commit			<= 1;
				end

				//Send the last 1-2 bytes of an untagged frame
				else if(send_tagged_last) begin
					frame_bus.data_valid		<= 1;
					frame_bus.data				<= { mac_bus_ff.data[111:0], 16'h0 };
					frame_bus.bytes_valid		<= mac_bus_ff.bytes_valid - 2;
					frame_bus.commit			<= 1;
				end

				//Start a new frame
				if(mac_bus.start) begin
					headers.dst_mac				<= mac_bus.data[127:80];
					headers.src_mac				<= mac_bus.data[79:32];

					headers.ethertype			<= ethertype_t'(mac_bus.data[31:16]);

					if(mac_bus.data[31:16] == ETHERTYPE_DOT1Q) begin
						headers.has_vlan_tag	<= 1;
						headers.qos_pri			<= mac_bus.data[15:13];
						headers.drop_eligible	<= mac_bus.data[12];
						headers.vlan_id			<= mac_bus.data[11:0];
					end
					else begin
						headers.has_vlan_tag	<= 0;
						headers.qos_pri			<= 0;
						headers.drop_eligible	<= 0;
						headers.vlan_id			<= 0;
					end

					//Address filter
					if(promisc_mode || (mac_bus.data[127:80] == our_mac_address) )
						state	<= STATE_SECOND;
				end

			end	//end STATE_IDLE

			//Second word of a frame
			STATE_SECOND: begin

				//If we had a VLAN tag, ethertype is the start of this word, and we have only 14 data bytes.
				if(headers.has_vlan_tag) begin
					headers.ethertype		<= ethertype_t'(mac_bus.data[127:112]);
					state					<= STATE_THIRD;
				end

				//If no VLAN tag, we're ready to start the frame.
				else begin
					frame_bus.start			<= 1;
					frame_bus.data_valid	<= 1;
					frame_bus.bytes_valid	<= 16;
					frame_bus.data			<= { mac_bus_ff.data[15:0], mac_bus.data[127:16] };
					state					<= STATE_UNTAGGED_BODY;
				end

			end	//end STATE_SECOND

			//Third word of a frame with a VLAN tag
			STATE_THIRD: begin
				frame_bus.start				<= 1;
				frame_bus.data_valid		<= 1;
				frame_bus.bytes_valid		<= 16;
				frame_bus.data				<= { mac_bus_ff.data[111:0], mac_bus.data[127:112] };
				state						<= STATE_TAGGED_BODY;
			end	//end STATE_THIRD

			//Body of an untagged frame
			STATE_UNTAGGED_BODY: begin

				frame_bus.data_valid		<= 1;
				frame_bus.data				<= { mac_bus_ff.data[15:0], mac_bus.data[127:16] };

				if(mac_bus.commit) begin

					state					<= STATE_IDLE;

					//End of frame. Did everything fit this cycle?
					if(mac_bus.bytes_valid <= 14) begin
						frame_bus.bytes_valid	<= mac_bus.bytes_valid + 2;
						frame_bus.commit		<= 1;
					end

					//No, gotta send it next time
					else begin
						frame_bus.bytes_valid	<= 16;
						send_untagged_last		<= 1;
					end

				end

				else
					frame_bus.bytes_valid	<= 16;

			end	//end STATE_UNTAGGED_BODY

			//Body of a tagged frame
			STATE_TAGGED_BODY: begin

				frame_bus.data_valid		<= 1;
				frame_bus.data				<= { mac_bus_ff.data[111:0], mac_bus.data[127:112] };

				if(mac_bus.commit) begin

					state					<= STATE_IDLE;

					//End of frame. Did everything fit this cycle?
					if(mac_bus.bytes_valid <= 2) begin
						frame_bus.bytes_valid	<= mac_bus.bytes_valid + 14;
						frame_bus.commit		<= 1;
					end

					//No, gotta send it next time
					else begin
						frame_bus.bytes_valid	<= 16;
						send_untagged_last		<= 1;
					end

				end

				else
					frame_bus.bytes_valid	<= 16;

			end	//end STATE_TAGGED_BODY

		endcase

		//If incoming frame requests a drop, discard the frame in progress and reset
		if(mac_bus.drop) begin
			frame_bus.drop		<= 1;
			frame_bus.commit	<= 0;
			state				<= STATE_IDLE;
		end

	end

endmodule
