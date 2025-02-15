`timescale 1ns / 1ps
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
	@brief Serial Chip to Chip Bus link layer

	Valid configurations for SYMBOL_WIDTH:
		* TODO: support 2
		* 4
		* TODO: support 8?

	All control characters other than K28.6 end-of-frame must occur in the lane 0 position.
	This means that both sides of a link must have the same SYMBOL_WIDTH setting and thus use the same amount of
	padding.

	Overall frame format:
		IDLE: K28.5 D16.2 (padded with D0.0 to SYMBOL_WIDTH if necessary)
		START: K character
			K30.7: LL ACK or error
			K28.6: end of frame (not legal outside a frame)
			All other K characters forwarded up to transaction layer
		SEQUENCE: 8 bit unsigned, wraps around
		DATA: transaction layer dependent
		CRC: CRC-8-CCITT/ATM (x^8 + x^2 + x + 1) of upper layer data only
		END: K28.6
		PADDING: Zero or more D0.0 bytes appended after CRC to fill out a full SERDES word

	TX SIDE INTERFACE
		Wait for link_up to go high
		Assert start
			valid = SYMBOL_WIDTH
			data[7:0] = k char for start of frame
			data[DATA_WIDTH-1:8] = data chars for frame payload
		Next cycle
			valid = SYMBOL_WIDTH
			data[DATA_WIDTH-1:0] = payload
		Last cycle
			Valid = 0...SYMBOL_WIDTH-1
			data[valid*8-1 : 0] = payload
 */
module SCCB_LinkLayer #(

	parameter SYMBOL_WIDTH 	= 4,	//Typical config: 40 bit bus width for 7 series GTP or U+ GTY
									//(4x 8b10b symbols per block)
									//Note, on GTP this requires a 20 bit internal width and TXUSRCLK at 2x the rate
									//of TXUSRCLK2

	localparam DATA_WIDTH	= 8 * SYMBOL_WIDTH,		//Width of the internal data bus in bits

	localparam SYMBOL_BITS	= $clog2(SYMBOL_WIDTH),

	//Control characters
	localparam IDLE_CODE	= 8'hbc,	//K28.5
	localparam STOP_CODE	= 8'hdc		//K28.6
) (
	//SERDES ports
	input wire						rx_clk,
	input wire[SYMBOL_WIDTH-1:0]	rx_kchar,
	input wire[DATA_WIDTH-1:0]		rx_data,
	input wire						rx_data_valid,

	input wire						tx_clk,
	output logic[SYMBOL_WIDTH-1:0]	tx_kchar		= 0,
	output logic[DATA_WIDTH-1:0]	tx_data			= 0,

	//Link layer data outputs (rx_clk domain) to transaction layer
	output logic					rx_ll_link_up	= 0,
	output logic					rx_ll_start		= 0,	//start of an APB or other upper layer transaction
	output logic					rx_ll_valid		= 0,	//true if data is valid
	output logic[SYMBOL_BITS:0]		rx_ll_nvalid	= 0,	//number of valid bytes in rx_ll_data
	output logic[DATA_WIDTH-1:0]	rx_ll_data		= 0,	//upper layer data (left aligned)
															//if rx_ll_start is set, the first word is a K character
	output logic					rx_ll_commit	= 0,	//frame received and is valid
	output logic					rx_ll_drop		= 0,	//frame received but checksum was bad, dropped

	//Link layer inputs (tx_clk domain) from transaction layer
	output wire						tx_ll_link_up,
	input wire						tx_ll_start,
	input wire[SYMBOL_BITS:0]		tx_ll_valid,
	input wire[DATA_WIDTH-1:0]		tx_ll_data
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX CRC block

	logic		rx_crc_reset	= 0;
	logic[2:0]	rx_crc_din_len	= 0;
	logic[31:0]	rx_crc_din		= 0;
	wire[7:0]	rx_crc_dout;

	CRC8_ATM_x32_variable  #(
		.LEFT_ALIGN(0)
	) rx_crc (
		.clk(rx_clk),
		.reset(rx_crc_reset),
		.din_len(rx_crc_din_len),
		.din(rx_crc_din),
		.crc_out(rx_crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side

	logic		rx_active				= 0;

	logic		rx_is_idle				= 0;
	logic[7:0]	rx_idle_count			= 0;
	logic[7:0]	rx_error_count			= 0;

	logic[7:0]	rx_checksum_expected	= 0;
	logic		rx_checksum_next		= 0;
	logic		rx_done_next			= 0;
	logic		rx_done					= 0;

	//Commit if checksum is valid
	always_comb begin
		rx_ll_commit = rx_done && (rx_crc_dout == rx_checksum_expected);
	end

	always_ff @(posedge rx_clk) begin

		rx_ll_start			<= 0;
		rx_ll_valid			<= 0;
		rx_ll_nvalid		<= 0;
		rx_ll_data			<= 0;
		rx_ll_drop			<= 0;

		rx_crc_reset		<= 0;
		rx_crc_din_len		<= 0;
		rx_crc_din			<= rx_data;

		rx_checksum_next	<= 0;
		rx_done_next		<= 0;
		rx_done				<= rx_done_next;

		//Send drop flag if checksum is bad
		if(rx_done && (rx_crc_dout != rx_checksum_expected) )
			rx_ll_drop	<= 1;

		//Gate all processing on data valid
		if(rx_data_valid) begin

			//Look for idles (K28.5 D21.5 with D0.0 in remaining slots if any)
			rx_is_idle	<= (rx_data == 'hb5_bc) && (rx_kchar == 1);

			//Expecting checksum?
			if(rx_checksum_next) begin
				rx_checksum_expected	<= rx_data[7:0];
				rx_done_next			<= 1;
			end

			//In a frame? If so, look for EOF
			else if(rx_active) begin

				//Look for end code or error
				if(rx_kchar) begin

					//We're no longer in a frame, no matter what
					rx_active	<= 0;

					//End in lane 0?
					if(rx_kchar[0]) begin

						if(rx_data[7:0] == STOP_CODE) begin
							//Previous cycle's frame was done
							//TODO: can we verify checksum a cycle ahead of time?
							rx_checksum_expected	<= rx_data[15:8];
							rx_done_next			<= 1;
						end

						//Malformed frame (didn't end with stop symbol)
						else
							rx_ll_drop	<= 1;

					end

					//End in lane 1?
					else if(rx_kchar[1]) begin

						if(rx_data[15:8] == STOP_CODE) begin
							rx_checksum_expected	<= rx_data[23:16];
							rx_done_next			<= 1;

							//Still one byte of data before the stop symbol
							rx_crc_din_len			<= 1;
							rx_ll_data				<= { 24'h0, rx_data[7:0] };
							rx_ll_valid				<= 1;
							rx_ll_nvalid			<= 1;
						end

						//Malformed frame (didn't end with stop symbol)
						else
							rx_ll_drop	<= 1;

					end

					//End in lane 2?
					else if(rx_kchar[2]) begin

						if(rx_data[23:16] == STOP_CODE) begin
							rx_checksum_expected	<= rx_data[31:24];
							rx_done_next			<= 1;

							//Still two bytes of data before the stop symbol
							rx_crc_din_len			<= 2;
							rx_ll_data				<= { 16'h0, rx_data[15:0] };
							rx_ll_valid				<= 1;
							rx_ll_nvalid			<= 2;
						end

						//Malformed frame (didn't end with stop symbol)
						else
							rx_ll_drop	<= 1;

					end

					//End in lane 3?
					else /* if(rx_kchar[3]) */ begin

						if(rx_data[31:24] == STOP_CODE) begin

							//We don't have the checksum yet
							rx_checksum_next	<= 1;

							//Still three bytes of data before the stop symbol
							rx_crc_din_len		<= 3;
							rx_ll_data				<= { 8'h0, rx_data[23:0] };
							rx_ll_valid				<= 1;
							rx_ll_nvalid			<= 3;
						end

						//Malformed frame (didn't end with stop symbol)
						else
							rx_ll_drop	<= 1;

					end

				end

				//Nope, still doing data
				else begin
					rx_crc_din_len	<= 4;

					rx_ll_valid		<= 1;
					rx_ll_nvalid	<= 4;
					rx_ll_data		<= rx_data;
				end

			end

			//Link up but not in a frame?
			else if(rx_ll_link_up) begin

				//Expect a control character at position 0, and nowhere else
				//(LLPs other than ACKs are all >2 bytes long so cannot start and end in same word)
				if( (rx_kchar[SYMBOL_WIDTH-1:1] == 0) && (rx_kchar[0] == 1) ) begin

					//Idle? No action required
					if(rx_data[7:0] == IDLE_CODE) begin
					end

					//K30.7? LL control packet, TODO handle these
					else if(rx_data[7:0] == 8'hfe) begin
					end

					//K28.6? End of frame, we should never see this outside a frame.
					//Call this an error
					else if(rx_data[7:0] == STOP_CODE) begin
						rx_error_count	<= rx_error_count + 1;
					end

					//Anything else? start of an upper layer frame
					//There's always going to be header + seq + 2 data bytes (no frame is allowed to be < 2 bytes long)
					else begin
						rx_ll_start		<= 1;
						rx_ll_valid		<= 1;
						rx_ll_nvalid	<= 4;
						rx_ll_data		<= rx_data;

						rx_active		<= 1;

						rx_crc_reset	<= 1;
						rx_crc_din		<= { 16'h0, rx_data[31:16] };
						rx_crc_din_len	<= 2;
					end

				end

				//Handle single word ACK packets (start, sequence, ack state, done), with CRC next cycle
				else if(rx_kchar == 4'b1001) begin

					rx_ll_start			<= 1;
					rx_ll_valid			<= 1;
					rx_ll_nvalid		<= 3;
					rx_ll_data			<= rx_data;

					//frame is already over
					rx_active			<= 0;

					rx_crc_reset		<= 1;
					rx_crc_din			<= { 24'h0, rx_data[23:16] };
					rx_crc_din_len		<= 1;

					rx_checksum_next	<= 1;

				end

				//If we do not see a control character in lane 0, call this an error
				else begin
					rx_error_count	<= rx_error_count + 1;
				end

			end

			//Link down? Wait until we get a bunch of idles in a row
			else begin

				//Idle? Bump the count and declare link up if we hit a reasonable threshold
				if(rx_is_idle) begin
					rx_idle_count	<= rx_idle_count + 1;

					if(rx_idle_count == 8'hff) begin
						rx_ll_link_up	<= 1;
						rx_error_count	<= 0;
					end
				end

				//Not an idle? Reset count
				else
					rx_idle_count	<= 0;

			end

			//If we hit enough errors between idles, reset the link
			if(rx_error_count == 64)
				rx_ll_link_up		<= 0;

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX to TX control flow

	//TODO: when committing or dropping a frame, send layer 2 ACK/NAK

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX CRC block

	logic		tx_crc_reset	= 0;
	logic[2:0]	tx_crc_din_len	= 0;
	logic[31:0]	tx_crc_din		= 0;
	wire[7:0]	tx_crc_dout;

	CRC8_ATM_x32_variable #(
		.LEFT_ALIGN(0)
	) tx_crc (
		.clk(tx_clk),
		.reset(tx_crc_reset),
		.din_len(tx_crc_din_len),
		.din(tx_crc_din),
		.crc_out(tx_crc_dout)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side

	//TODO: all of this stuff here assumes 4 byte datapath and needs to not assume that...

	logic		tx_active		= 0;
	logic		tx_crc_last		= 0;
	logic		tx_crc_last_adv	= 0;
	logic[1:0]	tx_crc_pos		= 0;

	logic[7:0]	tx_seq			= 0;

	//Pipeline TX data so we have time to calculate the CRC before inserting it
	logic[SYMBOL_WIDTH-1:0]	tx_kchar_adv	= 0;
	logic[DATA_WIDTH-1:0]	tx_data_adv		= 0;

	//Send data to TX CRC
	always_comb begin
		tx_crc_reset	= tx_ll_start;
		tx_crc_din_len	= tx_ll_valid;
		tx_crc_din		= tx_ll_data;

		//Starting a frame? Only checksum the data
		if(tx_ll_start)
			tx_crc_din		= { 16'h0, tx_ll_data[31:16] };
	end

	//Inject TX CRC into outbound data
	always_comb begin
		tx_kchar		= tx_kchar_adv;
		tx_data			= tx_data_adv;

		if(tx_crc_last) begin
			case(tx_crc_pos)
				0:	begin
					tx_data		= { 24'h0, tx_crc_dout };
					tx_kchar[0]	= 0;
				end
				1:	tx_data = { 16'h0, tx_crc_dout, tx_data_adv[7:0] };
				2:	tx_data = { 8'h0, tx_crc_dout, tx_data_adv[15:0] };
				3:	tx_data = { tx_crc_dout, tx_data_adv[23:0] };
			endcase
		end

	end

	//Main transmit path muxing
	always_ff @(posedge tx_clk) begin

		tx_crc_last_adv	<= 0;
		tx_crc_last		<= tx_crc_last_adv;
		tx_crc_pos		<= 0;

		//Continue or end a frame
		if(tx_active) begin

			case(tx_ll_valid)

				//Full word
				4: begin
					tx_kchar_adv	<= 4'b0000;
					tx_data_adv		<= tx_ll_data;
				end

				//3 data bytes + stop code
				3: begin
					tx_kchar_adv	<= 4'b1000;
					tx_data_adv		<= { STOP_CODE, tx_ll_data[23:0] };
					tx_active		<= 0;

					tx_crc_last_adv	<= 1;	//append crc *next* word
					tx_crc_pos		<= 0;
				end

				//2 data bytes + stop code + checksum
				2: begin
					tx_kchar_adv	<= 4'b0100;
					tx_data_adv		<= { 8'h0, STOP_CODE, tx_ll_data[15:0] };
					tx_active		<= 0;

					tx_crc_last		<= 1;	//append crc *this* word
					tx_crc_pos		<= 3;
				end

				//1 data byte + stop code + checksum + padding
				1: begin
					tx_kchar_adv	<= 4'b0010;
					tx_data_adv		<= { 16'h0, STOP_CODE, tx_ll_data[7:0] };
					tx_active		<= 0;

					tx_crc_last		<= 1;	//append crc *this* word
					tx_crc_pos		<= 2;
				end

				//Last cycle had full data
				//Send stop code + checksum + padding
				default: begin
					tx_kchar_adv	<= 4'b0001;
					tx_data_adv		<= { 23'h0, STOP_CODE };
					tx_active		<= 0;

					tx_crc_last		<= 1;	//append crc *this* word
					tx_crc_pos		<= 1;
				end

			endcase

		end

		//Start a frame or send idles
		else begin

			//Always sending a control character in the first lane
			tx_kchar_adv			<= 4'b0001;

			//Start a frame
			if(tx_ll_start) begin
				tx_data_adv			<= tx_ll_data;
				tx_active			<= 1;

				//Inject sequence number
				tx_data_adv[15:8]	<= tx_seq;
				tx_seq				<= tx_seq + 1;

				//If the frame has only a single byte of payload, add the end symbol in lane 3
				if(tx_ll_valid == 1) begin
					tx_kchar_adv		<= 4'b1001;
					tx_data_adv[31:24]	<= STOP_CODE;

					tx_crc_last_adv		<= 1; //append crc *next* word
					tx_crc_pos			<= 0;
					tx_active			<= 0;
				end

			end

			//Default: send idles
			else begin
				tx_data_adv			<= {DATA_WIDTH{1'b0}};
				tx_data_adv[15:0]	<= { 8'hb5, IDLE_CODE };
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC from TX to RX domain

	ThreeStageSynchronizer sync_rx_to_tx_ll_link_up(
		.clk_in(rx_clk),
		.din(rx_ll_link_up),
		.clk_out(tx_clk),
		.dout(tx_ll_link_up));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC from RX to TX domain

endmodule
