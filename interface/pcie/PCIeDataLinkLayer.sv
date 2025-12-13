`timescale 1ns/1ps
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
	@brief PCI Express data link layer
 */
module PCIeDataLinkLayer(
	input wire			clk,
	input wire			rst_n,

	input wire			link_up,

	input wire[15:0]	rx_data,
	input wire[1:0]		rx_charisk,
	input wire[1:0]		rx_err,

	output logic[15:0]	tx_data			= 0,
	output logic[1:0]	tx_charisk		= 0,

	input wire			tx_skip_req,
	output logic		tx_skip_ack		= 0,
	input wire			tx_skip_done

	//todo: data link specific ports
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Descramble incoming data

	logic[15:0]	rx_data_descrambled		= 0;
	logic[1:0]	rx_charisk_ff			= 0;
	logic[1:0]	rx_err_ff				= 0;

	logic[15:0]	scrambler_state			= 0;
	logic[15:0]	scrambler_state_next;

	logic[15:0]	scrambler_out;
	always_comb begin
		scrambler_state_next			= scrambler_state;
		scrambler_out					= 0;

		for(integer i=0; i<16; i++) begin
			scrambler_out[i]			= scrambler_state_next[15];

			if(scrambler_out[i])
				scrambler_state_next	= ((scrambler_state_next ^ 16'h1c) << 1) | 1;
			else
				scrambler_state_next	= (scrambler_state_next << 1);
		end
	end

	always_ff @(posedge clk or negedge rst_n) begin
		if(!rst_n) begin
			scrambler_state		<= 16'hffff;
			rx_data_descrambled	<= 0;
			rx_charisk_ff		<= 0;
			rx_err_ff			<= 0;
		end

		else begin
			rx_charisk_ff		<= rx_charisk;
			rx_err_ff			<= rx_err;

			//Descramble data
			if(rx_charisk[0])
				rx_data_descrambled[7:0]	<= rx_data[7:0];
			else
				rx_data_descrambled[7:0]	<= rx_data[7:0] ^ scrambler_out[7:0];

			if(rx_charisk[1])
				rx_data_descrambled[15:8]	<= rx_data[15:8];
			else
				rx_data_descrambled[15:8]	<= rx_data[15:8] ^ scrambler_out[15:8];

			//Don't run the scrambler during a skip set
			//TODO: handle odd numbers of skips?
			if( (rx_charisk == 2'b11) && (rx_data == 16'h1c1c) ) begin
			end
			else
				scrambler_state		<= scrambler_state_next;

			//Commas reset the scrambler
			if( (rx_data[7:0] == 8'hbc) && (rx_charisk[0]) ) begin

				//If this was a skip, don't advance
				if( (rx_data[15:8] == 8'h1c) && rx_charisk[1])
					scrambler_state	<= 16'hffff;

				//otherwise advance by one stage
				else
					scrambler_state	<= 16'he817;
			end

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parsing of incoming DLLPs (and eventually TLPs)

	enum logic[3:0]
	{
		RX_STATE_IDLE			= 0,
		RX_STATE_DLLP_1			= 1,
		RX_STATE_DLLP_2			= 2,
		RX_STATE_DLLP_CRC_END	= 3
	} rx_state = RX_STATE_IDLE;

	typedef enum logic[7:0]
	{
		DLLP_TYPE_ACK			= 8'b0000_0000,
		DLLP_TYPE_NAK			= 8'b0001_0000,
		DLLP_TYPE_PM_ENTER_L1	= 8'b0010_0000,
		DLLP_TYPE_PM_ENTER_L23	= 8'b0010_0001,
		DLLP_TYPE_PM_ACTIVE_L1	= 8'b0010_0011,
		DLLP_TYPE_PM_REQ_ACK	= 8'b0010_0100,
		DLLP_TYPE_VENDOR		= 8'b0011_0000,
		DLLP_TYPE_INITFC1_P		= 8'b0100_0000,	//low 3 bits are VC
		DLLP_TYPE_INITFC1_NP	= 8'b0101_0000,	//low 3 bits are VC
		DLLP_TYPE_INITFC1_CPL	= 8'b0110_0000,	//low 3 bits are VC
		DLLP_TYPE_INITFC2_P		= 8'b1100_0000,	//low 3 bits are VC
		DLLP_TYPE_INITFC2_NP	= 8'b1101_0000,	//low 3 bits are VC
		DLLP_TYPE_INITFC2_CPL	= 8'b1110_0000,	//low 3 bits are VC
		DLLP_TYPE_UPDATEFC_P	= 8'b1000_0000,	//low 3 bits are VC
		DLLP_TYPE_UPDATEFC_NP	= 8'b1001_0000,	//low 3 bits are VC
		DLLP_TYPE_UPDATEFC_CPL	= 8'b1010_0000	//low 3 bits are VC
	} dllp_type_t;

	logic		rx_dllp_valid			= 0;
	logic[7:0]	rx_dllp_type			= 0;	//Type of DLLP
	logic[23:0]	rx_dllp_payload			= 0;	//Payload (type dependent)

	logic[7:0]	rx_dllp_crc_hi			= 0;
	logic[15:0]	rx_dllp_crc_comb;
	logic[15:0]	rx_dllp_crc_expected	= 0;

	//Format the RX CRC input

	logic[15:0]	rx_dllp_crc_din;
	always_comb begin

		//Start a new CRC
		if(rx_state == RX_STATE_DLLP_1) begin
			rx_dllp_crc_din		= { rx_data_descrambled[7:0], rx_dllp_type };
			rx_dllp_crc_comb	= 16'hffff;
		end

		//Continue an existing CRC
		else begin
			rx_dllp_crc_din		= { rx_data_descrambled[7:0], rx_dllp_payload[15:8] };
			rx_dllp_crc_comb	= rx_dllp_crc_expected;
		end

		for(integer i=0; i<2; i++) begin
			for(integer j=0; j<8; j++) begin
				if(rx_dllp_crc_comb[0] ^ rx_dllp_crc_din[8*i + j])
					rx_dllp_crc_comb = rx_dllp_crc_comb[15:1] ^ 16'hd008;
				else
					rx_dllp_crc_comb = rx_dllp_crc_comb[15:1];
			end
		end

		//Output inversion and byte swapping
		if(rx_state != RX_STATE_DLLP_1)
			rx_dllp_crc_comb = {~rx_dllp_crc_comb[7:0], ~rx_dllp_crc_comb[15:8] };

	end

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			rx_state		<= RX_STATE_IDLE;
			rx_dllp_valid	<= 0;
		end

		else begin
			rx_dllp_valid	<= 0;

			case(rx_state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Wait for a TLP or DLLP to start

				RX_STATE_IDLE: begin

					//K28.2 (k.5c) is start of DLLP
					if( (rx_charisk_ff == 2'b01) && (rx_data_descrambled[7:0] == 8'h5c) ) begin

						//First byte of the DLLP is the type
						rx_dllp_type	<= rx_data_descrambled[15:8];
						rx_state		<= RX_STATE_DLLP_1;

					end

				end	//end RX_STATE_IDLE

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// DLLP path

				//First two bytes of the DLLP
				RX_STATE_DLLP_1: begin

					//Register CRC output
					rx_dllp_crc_expected		<= rx_dllp_crc_comb;

					//should not have any k chars
					if(rx_charisk_ff == 2'b00) begin
						rx_dllp_payload[23:8]	<= { rx_data_descrambled[7:0], rx_data_descrambled[15:8] };
						rx_state				<= RX_STATE_DLLP_2;
					end

					//otherwise drop it
					else
						rx_state				<= RX_STATE_IDLE;

				end	//end RX_STATE_DLLP_1

				//Last byte of the DLLP, high half of the CRC
				RX_STATE_DLLP_2: begin

					//Register CRC output
					rx_dllp_crc_expected		<= rx_dllp_crc_comb;

					//should not have any k chars
					if(rx_charisk_ff == 2'b00) begin
						rx_dllp_payload[7:0]	<= rx_data_descrambled[7:0];
						rx_dllp_crc_hi			<= rx_data_descrambled[15:8];
						rx_state				<= RX_STATE_DLLP_CRC_END;

					end

					//otherwise drop it
					else
						rx_state				<= RX_STATE_IDLE;

				end	//end RX_STATE_DLLP_2

				RX_STATE_DLLP_CRC_END: begin

					//should be one byte of CRC then the end symbol (k.fd)
					if( (rx_charisk_ff == 2'b10) && (rx_data_descrambled[15:8] == 8'hfd) ) begin
						//7:0 is the rest of the CRC

						//Set valid flag if they match
						if(rx_dllp_crc_expected == {rx_dllp_crc_hi, rx_data_descrambled[7:0] })
							rx_dllp_valid	<= 1;

					end

					//No matter what happens we're back in the idle state after this
					rx_state				<= RX_STATE_IDLE;

				end	//end RX_STATE_DLLP_CRC_END

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// TLP path

			endcase

			//Any symbol error, or dropping the link, clears us back to the idle state for now
			//TODO: send a nak or something
			if(rx_err_ff || !link_up)
				rx_state	<= RX_STATE_IDLE;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main logic

	logic	skip_pending	= 0;

	always_ff @(posedge clk) begin

		tx_skip_ack		<= 0;

		//If we have a pending skip request, ack it
		if(tx_skip_req) begin
			tx_skip_ack			<= 1;
			skip_pending		<= 1;
		end

		//wait for skip to finish
		if(skip_pending) begin
			if(tx_skip_done) begin
				skip_pending	<= 0;
			end
		end

		//Normal mode
		else begin
			tx_data				<= 0;
			tx_charisk			<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_0 ila(
		.clk(clk),

		.probe0(rx_data),
		.probe1(rx_charisk),
		.probe2(rx_err),
		.probe3(tx_data),
		.probe4(tx_charisk),
		.probe5(tx_skip_req),
		.probe6(tx_skip_ack),
		.probe7(skip_pending),
		.probe8(rx_state),
		.probe9(rx_dllp_type),
		.probe10(rx_dllp_payload),
		.probe11(rx_dllp_valid),
		.probe12(rx_dllp_crc_hi),
		.probe13(rx_dllp_crc_expected),
		.probe14(link_up),
		.probe15(rx_data_descrambled),
		.probe16(rx_charisk_ff),
		.probe17(rx_err_ff),
		.probe18(rx_dllp_crc_din)
	);

endmodule
