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
	input wire				clk,
	input wire				rst_n,

	//Link training state from LTSSM
	input wire				link_up,

	//RX data from SERDES
	input wire[15:0]		rx_data,
	input wire[1:0]			rx_charisk,
	input wire[1:0]			rx_err,

	//Transmit data to SERDES
	output logic[15:0]		tx_data			= 0,
	output logic[1:0]		tx_charisk		= 0,

	//Skip generation
	input wire				tx_skip_req,
	output logic			tx_skip_ack		= 0,
	input wire				tx_skip_done,

	//Data link layer status
	output logic			dl_link_up		= 0,

	//AXI stream to the transaction layer
	//TUSER[0] is "CRC bad" flag, only valid when tlast=1
	AXIStream.transmitter	axi_tlp_rx,

	//AXI stream from the transaction layer
	AXIStream.receiver		axi_tlp_tx
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// AXI stream tieoffs

	//for now always ready
	assign axi_tlp_tx.tready	= 1;

	assign axi_tlp_rx.aclk		= clk;
	assign axi_tlp_rx.areset_n	= rst_n;
	assign axi_tlp_rx.tid 		= 0;
	assign axi_tlp_rx.tdest		= 0;
	assign axi_tlp_rx.twakeup	= 0;

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
	// RX DLLP CRC engine

	logic		rx_dllp_valid			= 0;
	logic[7:0]	rx_dllp_type			= 0;	//Type of DLLP
	logic[23:0]	rx_dllp_payload			= 0;	//Payload (type dependent)

	wire[15:0]	rx_dllp_crc_expected;

	enum logic[3:0]
	{
		RX_STATE_IDLE			= 0,

		RX_STATE_DLLP_1			= 1,
		RX_STATE_DLLP_2			= 2,
		RX_STATE_DLLP_CRC_END	= 3,

		RX_STATE_TLP_1			= 4,
		RX_STATE_TLP_2			= 5
	} rx_state = RX_STATE_IDLE;

	PCIeDataLinkChecksum rx_dllp_checksum(
		.clk(clk),
		.rst_n(rst_n),

		.first_phase(rx_state == RX_STATE_DLLP_1),
		.second_phase(rx_state == RX_STATE_DLLP_2),
		.data_in_first({ rx_data_descrambled[7:0], rx_dllp_type }),
		.data_in_second({ rx_data_descrambled[7:0], rx_dllp_payload[15:8] }),

		.crc_out(rx_dllp_crc_expected)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX TLP CRC engine

	wire[31:0]	rx_tlp_crc_expected;
	logic[55:0]	rx_tlp_data				= 0;
	logic[2:0]	rx_tlp_bytecount		= 0;
	logic[11:0]	rx_tlp_seq				= 0;

	logic[31:0]	rx_tlp_crc_din;
	logic		rx_tlp_crc_reset		= 0;
	logic		rx_tlp_crc_ce		= 0;
	logic		rx_tlp_crc_half			= 0;

	//TLP checksum includes the sequence number!!
	CRC32_Ethernet_x32_variable_halfsel rx_tlp_checksum(
		.clk(clk),
		.reset(rx_tlp_crc_reset),
		.ce(rx_tlp_crc_ce),
		.din(rx_tlp_crc_din),
		.halfwidth(rx_tlp_crc_half),
		.crc_out(rx_tlp_crc_expected)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parsing of incoming DLLPs and TLPs

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

	logic[7:0]	rx_dllp_crc_hi			= 0;
	logic		rx_tlp_ending			= 0;
	logic[31:0]	rx_tlp_crc_actual		= 0;

	//RX acknowledgement
	logic[11:0]	next_rcv_seq			= 0;
	logic		nak_scheduled			= 0;
	logic[7:0]	acknak_latency_timer	= 0;

	//FIFO of DLLPs the RX logic wants to send
	logic		tlp_dllp_wr_en			= 0;
	logic[7:0]	tlp_dllp_type			= 0;
	logic[23:0]	tlp_dllp_data			= 0;
	logic		tlp_dllp_rd				= 0;
	wire[7:0]	tlp_dllp_fifo_type;
	wire[23:0]	tlp_dllp_fifo_data;
	wire		tlp_dllp_fifo_empty;

	SingleClockFifo #(
		.WIDTH(32),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(0)
	) dllp_tx_fifo (
		.clk(clk),
		.wr(tlp_dllp_wr_en),
		.din({tlp_dllp_type, tlp_dllp_data}),
		.rd(tlp_dllp_rd),
		.dout({tlp_dllp_fifo_type, tlp_dllp_fifo_data}),
		.empty(tlp_dllp_fifo_empty),
		.full(),
		.overflow(),
		.underflow(),
		.rsize(),
		.wsize(),
		.reset(!dl_link_up)
	);

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			rx_state				<= RX_STATE_IDLE;
			rx_dllp_valid			<= 0;
			axi_tlp_rx.tvalid		<= 0;
			rx_tlp_crc_actual		<= 0;
			next_rcv_seq			<= 0;
			nak_scheduled			<= 0;
			acknak_latency_timer	<= 0;
			tlp_dllp_wr_en			<= 0;
			rx_tlp_crc_reset		<= 0;
			rx_tlp_crc_ce			<= 0;
			rx_tlp_crc_din			<= 0;
		end

		else begin

			//Clear single cycle flags
			rx_dllp_valid			<= 0;
			axi_tlp_rx.tvalid		<= 0;
			rx_tlp_ending			<= 0;
			tlp_dllp_wr_en			<= 0;
			rx_tlp_crc_reset		<= 0;
			rx_tlp_crc_ce			<= 0;
			rx_tlp_crc_half			<= 0;
			axi_tlp_rx.tlast		<= 0;

			//Clear flow control if the data link layer is not up
			if(!dl_link_up) begin
				next_rcv_seq			<= 12'h0;
				nak_scheduled			<= 0;
				acknak_latency_timer	<= 0;
			end

			//Validate TLP checksum and report the end of the TLP
			if(rx_tlp_ending) begin
				axi_tlp_rx.tvalid	<= 1;
				axi_tlp_rx.tlast	<= 1;
				axi_tlp_rx.tuser	<= rx_tlp_crc_actual != rx_tlp_crc_expected;

				//TODO: validate sequence number, ack duplicates, nak anything else (see page 205)

				//Prepare to ack/nak the TLP if needed
				tlp_dllp_data		<= rx_tlp_seq;

				//All happy
				if(rx_tlp_crc_actual == rx_tlp_crc_expected) begin
					next_rcv_seq	<= next_rcv_seq + 1;
					tlp_dllp_type	<= DLLP_TYPE_ACK;

					//DEBUG: send immediate ack
					tlp_dllp_wr_en	<= 1;
				end

				//No, something went wrong. Send an immediate nak
				else begin
					tlp_dllp_wr_en	<= 1;
					tlp_dllp_type	<= DLLP_TYPE_NAK;
					nak_scheduled	<= 1;
				end

			end

			case(rx_state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Wait for a TLP or DLLP to start

				RX_STATE_IDLE: begin

					//K28.2 (K.5c) is start of DLLP
					if( (rx_charisk_ff == 2'b01) && (rx_data_descrambled[7:0] == 8'h5c) ) begin

						//First byte of the DLLP is the type
						rx_dllp_type	<= rx_data_descrambled[15:8];
						rx_state		<= RX_STATE_DLLP_1;

					end

					//K27.7 (K.fb) is start of TLP
					else if( (rx_charisk_ff == 2'b01) && (rx_data_descrambled[7:0] == 8'hfb) ) begin

						//First byte of the TLP is the high bit of the sequence number
						rx_tlp_seq[11:8]	<= rx_data_descrambled[11:8];
						rx_state			<= RX_STATE_TLP_1;

						//Reset the CRC engine
						rx_tlp_crc_reset	<= 1;
						rx_tlp_crc_ce	<= 1;

					end

				end	//end RX_STATE_IDLE

				////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// DLLP path

				//First two bytes of the DLLP
				RX_STATE_DLLP_1: begin

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

				//First cycle of TLP is special: one sequence byte, then one data byte
				RX_STATE_TLP_1: begin

					//should not have any k chars
					if(rx_charisk_ff == 2'b00) begin

						//First byte is the rest of the TLP sequence number
						rx_tlp_seq[7:0]		<= rx_data_descrambled[7:0];

						//Second byte is the first byte of the TLP header
						rx_tlp_data[7:0]	<= rx_data_descrambled[15:8];
						rx_tlp_bytecount	<= 1;

						//CRC the sequence number
						rx_tlp_crc_ce	<= 1;
						rx_tlp_crc_half		<= 1;
						rx_tlp_crc_din		<= { 4'b0, rx_tlp_seq[11:8], rx_data_descrambled[7:0], 16'h0 };

						rx_state			<= RX_STATE_TLP_2;
					end

					else
						rx_state			<= RX_STATE_IDLE;

				end	//end RX_STATE_TLP_1

				//Rest of the TLP is just data until we get to the end character (K29.7 / k.fd)
				RX_STATE_TLP_2: begin

					//End character in first lane: odd length TLP (invalid, drop it)
					//TODO: send NAK?
					if(rx_charisk_ff[0] && (rx_data_descrambled[7:0] == 8'hfd) ) begin
						axi_tlp_rx.tlast	<= 1;
						axi_tlp_rx.tuser	<= 1;
						axi_tlp_rx.tvalid	<= 1;
						axi_tlp_rx.tkeep	<= 0;
						axi_tlp_rx.tdata	<= 0;
						rx_state			<= RX_STATE_IDLE;
					end

					//End character in second lane: even length TLP, end + one byte of CRC
					//The previous 3 bytes are also the CRC
					else if(rx_charisk_ff[1] && (rx_data_descrambled[15:8] == 8'hfd) ) begin

						//we should always have 7 valid bytes so CRC is in the same position
						rx_tlp_crc_actual	<=
						{
							rx_tlp_data[39:32],
							rx_tlp_data[47:40],
							rx_tlp_data[55:48],
							rx_data_descrambled[7:0]
						};
						axi_tlp_rx.tuser	<= 0;

						//Fill the last block of the TLP and wait one cycle for the checksum before marking it valid
						axi_tlp_rx.tkeep	<= 4'b1111;
						axi_tlp_rx.tdata	<= rx_tlp_data[31:0];
						rx_tlp_ending		<= 1;

						//No more data in the scratch buffer
						rx_tlp_bytecount	<= 0;

						//Done
						rx_state			<= RX_STATE_IDLE;

					end

					//No end character, just normal TLP payload
					else if(!rx_charisk_ff) begin

						case(rx_tlp_bytecount)

							//We have one byte in 7:0, add the new 2 bytes to the holding buffer
							1: begin
								rx_tlp_data[23:8]	<= rx_data_descrambled;
								rx_tlp_bytecount	<= 3;
							end

							//We have 3 bytes in the holding buffer already and 2 new ones. Append them.
							3: begin
								rx_tlp_data[39:24]	<= rx_data_descrambled;
								rx_tlp_bytecount	<= 5;
							end

							//We have 5 bytes in the holding buffer and 2 new ones. Append them
							5: begin
								rx_tlp_data[55:40]	<= rx_data_descrambled;
								rx_tlp_crc_ce	<= 1;
								rx_tlp_crc_din		<=
								{
									rx_tlp_data[7:0],
									rx_tlp_data[15:8],
									rx_tlp_data[23:16],
									rx_tlp_data[31:24]
								};
								rx_tlp_bytecount	<= 7;
							end

							//Holding buffer is full and we're adding two new bytes for a total of 9
							//Push the first 4 to the AXI stream and shuffle the rest
							7: begin
								axi_tlp_rx.tvalid	<= 1;
								axi_tlp_rx.tuser	<= 0;
								axi_tlp_rx.tkeep	<= 4'b1111;
								axi_tlp_rx.tdata	<= rx_tlp_data[31:0];

								rx_tlp_data[23:0]	<= rx_tlp_data[55:32];
								rx_tlp_data[39:24]	<= rx_data_descrambled;
								rx_tlp_bytecount	<= 5;

								//debug
								rx_tlp_data[55:40]	<= 0;
							end

							//invalid, ignore
							default: begin
							end

						endcase

					end

					//Invalid control character, drop the TLP
					//TODO: send NAK?
					else begin
						axi_tlp_rx.tlast	<= 1;
						axi_tlp_rx.tuser	<= 1;
						axi_tlp_rx.tvalid	<= 1;
						axi_tlp_rx.tkeep	<= 0;
						axi_tlp_rx.tdata	<= 0;
						rx_state			<= RX_STATE_IDLE;
					end

				end //end RX_STATE_TLP_2

			endcase

			//Any symbol error, or dropping the link, clears us back to the idle state for now
			//TODO: if we have a symbol error while receiving a TLP, send a NAK for the expected sequence number
			if(rx_err_ff || !link_up)
				rx_state	<= RX_STATE_IDLE;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Flow control state for our single virtual circuit

	logic		partner_credit_valid_p		= 0;
	logic[7:0]	partner_credit_header_p		= 0;
	logic[11:0]	partner_credit_data_p		= 0;

	logic		partner_credit_valid_np		= 0;
	logic[7:0]	partner_credit_header_np	= 0;
	logic[11:0]	partner_credit_data_np		= 0;

	logic		partner_credit_valid_cpl	= 0;
	logic[7:0]	partner_credit_header_cpl	= 0;
	logic[11:0]	partner_credit_data_cpl		= 0;

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			partner_credit_valid_p		<= 0;
			partner_credit_valid_np		<= 0;
			partner_credit_valid_cpl	<= 0;
		end

		else begin

			//Every time a DLLP shows up, see if it's flow control related
			if(rx_dllp_valid) begin

				//Low 3 bits of flow control DLLP type are virtual circuit
				//We only support VC0, ignore anything for any other VC
				case(rx_dllp_type)

					//HdrFC is header credit count
					//DataFC is data credit count
					DLLP_TYPE_INITFC1_P: begin
						if(!partner_credit_valid_p) begin
							partner_credit_valid_p		<= 1;
							partner_credit_header_p		<= rx_dllp_payload[21:14];
							partner_credit_data_p		<= rx_dllp_payload[11:0];
						end
					end

					DLLP_TYPE_INITFC1_NP: begin
						if(!partner_credit_valid_np) begin
							partner_credit_valid_np		<= 1;
							partner_credit_header_np	<= rx_dllp_payload[21:14];
							partner_credit_data_np		<= rx_dllp_payload[11:0];
						end
					end

					DLLP_TYPE_INITFC1_CPL: begin
						if(!partner_credit_valid_cpl) begin
							partner_credit_valid_cpl	<= 1;
							partner_credit_header_cpl	<= rx_dllp_payload[21:14];
							partner_credit_data_cpl		<= rx_dllp_payload[11:0];
						end
					end

				endcase

			end

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX CRC engine

	logic[7:0]	tx_dllp_type	= 0;
	logic[23:0]	tx_dllp_payload	= 0;

	wire[15:0]	tx_dllp_crc;

	enum logic[2:0]
	{
		TX_STATE_IDLE		= 0,
		TX_STATE_SKIP		= 1,
		TX_STATE_DLLP_1		= 2,
		TX_STATE_DLLP_2		= 3,
		TX_STATE_DLLP_3		= 4
	} tx_state = TX_STATE_IDLE;

	PCIeDataLinkChecksum tx_checksum(
		.clk(clk),
		.rst_n(rst_n),

		.first_phase( (tx_state == TX_STATE_IDLE) && tx_dllp_req && !tx_dllp_ack && !tx_skip_req),
		.second_phase(tx_state == TX_STATE_DLLP_1),
		.data_in_first({ tx_dllp_payload[23:16], tx_dllp_type }),
		.data_in_second({ tx_dllp_payload[7:0], tx_dllp_payload[15:8] }),

		.crc_out(tx_dllp_crc)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit logic

	//Request sending a DLLP
	logic		tx_dllp_req		= 0;
	logic		tx_dllp_ack		= 0;

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			tx_state	<= TX_STATE_IDLE;
		end

		else begin

			//Clear single cycle flags
			tx_skip_ack		<= 0;
			tx_dllp_ack		<= 0;

			case(tx_state)

				TX_STATE_IDLE: begin

					//Send idles by default
					tx_data				<= 0;
					tx_charisk			<= 0;

					//If we have a pending skip request, ack it (takes precedence over starting a DLLP)
					if(tx_skip_req) begin
						tx_skip_ack		<= 1;
						tx_state		<= TX_STATE_SKIP;
					end

					//If we have a pending DLLP request, start sending it
					//K28.2 (k.5c) is start of DLLP
					else if(tx_dllp_req && !tx_dllp_ack) begin
						tx_data			<= { tx_dllp_type, 8'h5c };
						tx_charisk		<= 2'b01;
						tx_state		<= TX_STATE_DLLP_1;
					end

				end	//end TX_STATE_IDLE

				TX_STATE_SKIP: begin

					//Wait until we're done
					if(tx_skip_done) begin
						tx_state		<= TX_STATE_IDLE;
						tx_data			<= 0;
						tx_charisk		<= 0;
					end

				end	//end TX_STATE_SKIP

				TX_STATE_DLLP_1: begin
					tx_data			<= { tx_dllp_payload[15:8], tx_dllp_payload[23:16] };
					tx_charisk		<= 0;
					tx_state		<= TX_STATE_DLLP_2;
				end	//end TX_STATE_DLLP_1

				TX_STATE_DLLP_2: begin
					tx_data			<= { tx_dllp_crc[15:8], tx_dllp_payload[7:0] };
					tx_charisk		<= 0;
					tx_state		<= TX_STATE_DLLP_3;
					tx_dllp_ack		<= 1;
				end	//end TX_STATE_DLLP_2

				//K29.7 (k.fd) is end of packet
				TX_STATE_DLLP_3: begin
					tx_data			<= { 8'hfd, tx_dllp_crc[7:0] };
					tx_charisk		<= 2'b10;
					tx_state		<= TX_STATE_IDLE;
				end	//end TX_STATE_DLLP_3

			endcase

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Capacity limits for flow control

	localparam MAX_POSTED_HEADER		= 32;
	localparam MAX_POSTED_DATA			= 2047;

	localparam MAX_NONPOSTED_HEADER		= 32;
	localparam MAX_NONPOSTED_DATA		= 2047;

	//report unlimited completions
	localparam MAX_COMPLETION_HEADER	= 0;
	localparam MAX_COMPLETION_DATA		= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Top level state machine

	enum logic[1:0]
	{
		DL_INACTIVE		= 0,
		DL_INIT			= 1,
		DL_ACTIVE		= 2
	} dl_state = DL_INACTIVE;

	enum logic[2:0]
	{
		FC_INIT1_P		= 0,
		FC_INIT1_NP		= 1,
		FC_INIT1_CPL	= 2,

		FC_INIT2_P		= 3,
		FC_INIT2_NP		= 4,
		FC_INIT2_CPL	= 5,

		FC_DONE			= 6	//hang for now

	} fc_init_substate = FC_INIT1_P;

	logic[2:0]	rx_idle_count	= 0;
	logic		flag_fi2		= 0;

	//Send flow control updates every 30us nominally
	//At 2.5 GT/s we have a 400ps UI so this is 75000 UIs, 7500 symbols, or 3250 clocks with a 2-symbol datapath
	//At 5 GT/s we have a 200ps UI so this doubles to 150K UIs, 15K symbols, and 7500 clocks.
	localparam FC_TIMER_MAX_2G5	= 3249;
	localparam FC_TIMER_MAX_5G0	= 7499;
	logic[12:0]	fc_timer		= 0;
	logic[12:0]	fc_timer_max	= FC_TIMER_MAX_2G5;

	logic		next_fc_type_p	= 1;

	always_ff @(posedge clk or negedge rst_n) begin

		if(!rst_n) begin
			dl_state			<= DL_INACTIVE;
			dl_link_up			<= 0;
			tx_dllp_req			<= 0;
			fc_init_substate	<= FC_INIT1_P;
			rx_idle_count		<= 0;
			flag_fi2			<= 0;
			fc_timer_max		<= FC_TIMER_MAX_2G5;
			fc_timer			<= 0;
			next_fc_type_p		<= 1;
			tlp_dllp_rd			<= 0;
		end

		else begin

			tlp_dllp_rd			<= 0;

			//Set FI2 if we see any InitFC2 or UpdateFC
			if(rx_dllp_valid) begin
				case(rx_dllp_type)
					DLLP_TYPE_INITFC2_P:	flag_fi2 <= 1;
					DLLP_TYPE_INITFC2_NP:	flag_fi2 <= 1;
					DLLP_TYPE_INITFC2_CPL:	flag_fi2 <= 1;
					DLLP_TYPE_UPDATEFC_P:	flag_fi2 <= 1;
					DLLP_TYPE_UPDATEFC_NP:	flag_fi2 <= 1;
					DLLP_TYPE_UPDATEFC_CPL:	flag_fi2 <= 1;
					default: begin
					end
				endcase
			end

			//Set FI2 if we see any TLP (K27.7)
			if(rx_charisk_ff[0] && !rx_err_ff && (rx_data_descrambled[7:0] == 8'hfb) )
				flag_fi2	<= 1;

			case(dl_state)

				DL_INACTIVE: begin
					if(link_up) begin

						//Wait for 8 consecutive idle symbols, sending idles of our own, before we start sending DLLPs
						if( (rx_data_descrambled == 16'h00) && !rx_err_ff && !rx_charisk_ff)
							rx_idle_count	<= rx_idle_count + 1;
						else
							rx_idle_count	<= 0;

						if(rx_idle_count == 7)
							dl_state		<= DL_INIT;
					end
					else
						rx_idle_count = 0;
				end

				DL_INIT: begin

					//start sending flow control init DLLPs, return to inactive if link drops
					case(fc_init_substate)

						////////////////////////////////////////////////////////////////////////////////////////////////
						// InitFC-1

						//Send InitFC1-P
						FC_INIT1_P: begin

							tx_dllp_req				<= 1;
							tx_dllp_type			<= DLLP_TYPE_INITFC1_P;
							tx_dllp_payload			<= 0;
							tx_dllp_payload[21:14]	<= MAX_POSTED_HEADER;
							tx_dllp_payload[11:0]	<= MAX_POSTED_DATA;

							if(tx_dllp_ack) begin
								fc_init_substate	<= FC_INIT1_NP;

								//Start sending InitFC1-NP
								tx_dllp_req				<= 1;
								tx_dllp_type			<= DLLP_TYPE_INITFC1_NP;
								tx_dllp_payload			<= 0;
								tx_dllp_payload[21:14]	<= MAX_NONPOSTED_HEADER;
								tx_dllp_payload[11:0]	<= MAX_NONPOSTED_DATA;
							end

						end	//end FC_INIT1_P

						FC_INIT1_NP: begin

							if(tx_dllp_ack) begin
								fc_init_substate	<= FC_INIT1_CPL;

								//Start sending InitFC1-CPL
								tx_dllp_req				<= 1;
								tx_dllp_type			<= DLLP_TYPE_INITFC1_CPL;
								tx_dllp_payload			<= 0;
								tx_dllp_payload[21:14]	<= MAX_COMPLETION_HEADER;
								tx_dllp_payload[11:0]	<= MAX_COMPLETION_DATA;
							end

						end	//end FC_INIT1_NP

						FC_INIT1_CPL: begin

							if(tx_dllp_ack) begin

								tx_dllp_req				<= 1;
								tx_dllp_type			<= DLLP_TYPE_INITFC1_P;
								tx_dllp_payload			<= 0;
								tx_dllp_payload[21:14]	<= MAX_POSTED_HEADER;
								tx_dllp_payload[11:0]	<= MAX_POSTED_DATA;

								//If we have seen FC info from each VC, start sending InitFC2 instead
								if(partner_credit_valid_p && partner_credit_valid_np && partner_credit_valid_cpl) begin
									fc_init_substate		<= FC_INIT2_P;
									flag_fi2				<= 0;
								end
								else
									fc_init_substate		<= FC_INIT1_P;

							end

						end	//end FC_INIT1_CPL

						////////////////////////////////////////////////////////////////////////////////////////////////
						// InitFC-2

						//Send InitFC1-P
						FC_INIT2_P: begin

							tx_dllp_req				<= 1;
							tx_dllp_type			<= DLLP_TYPE_INITFC2_P;
							tx_dllp_payload			<= 0;
							tx_dllp_payload[21:14]	<= MAX_POSTED_HEADER;
							tx_dllp_payload[11:0]	<= MAX_POSTED_DATA;

							if(tx_dllp_ack) begin
								fc_init_substate	<= FC_INIT2_NP;

								//Start sending InitFC1-NP
								tx_dllp_req				<= 1;
								tx_dllp_type			<= DLLP_TYPE_INITFC2_NP;
								tx_dllp_payload			<= 0;
								tx_dllp_payload[21:14]	<= MAX_NONPOSTED_HEADER;
								tx_dllp_payload[11:0]	<= MAX_NONPOSTED_DATA;
							end

						end	//end FC_INIT2_P

						FC_INIT2_NP: begin

							if(tx_dllp_ack) begin
								fc_init_substate	<= FC_INIT2_CPL;

								//Start sending InitFC1-CPL
								tx_dllp_req				<= 1;
								tx_dllp_type			<= DLLP_TYPE_INITFC2_CPL;
								tx_dllp_payload			<= 0;
								tx_dllp_payload[21:14]	<= MAX_COMPLETION_HEADER;
								tx_dllp_payload[11:0]	<= MAX_COMPLETION_DATA;
							end

						end	//end FC_INIT2_NP

						FC_INIT2_CPL: begin

							if(tx_dllp_ack) begin

								tx_dllp_req				<= 1;
								tx_dllp_type			<= DLLP_TYPE_INITFC2_P;
								tx_dllp_payload			<= 0;
								tx_dllp_payload[21:14]	<= MAX_POSTED_HEADER;
								tx_dllp_payload[11:0]	<= MAX_POSTED_DATA;

								//Done?
								if(flag_fi2) begin
									fc_init_substate	<= FC_DONE;
									dl_state			<= DL_ACTIVE;
									dl_link_up			<= 1;
									fc_timer			<= 0;
									next_fc_type_p		<= 1;

									//TODO: speed dependent
									fc_timer_max		<= FC_TIMER_MAX_2G5;
								end
								else
									fc_init_substate	<= FC_INIT2_P;

							end

						end	//end FC_INIT2_CPL

						////////////////////////////////////////////////////////////////////////////////////////////////
						// Idle

						FC_DONE: begin
						end	//end FC_DONE

					endcase

					if(!link_up) begin
						dl_state	<= DL_INACTIVE;
						dl_link_up	<= 0;
					end
				end

				DL_ACTIVE: begin

					//Clear DLLP request when we get an ack
					if(tx_dllp_ack)
						tx_dllp_req	<= 0;

					//Timer to send UpdateFC DLLPs every X time
					fc_timer		<= fc_timer + 1;

					if(!tx_dllp_req) begin

						//If TLP engine has a pending DLLP, that's first priority
						if(!tlp_dllp_fifo_empty) begin
							tx_dllp_req				<= 1;
							tx_dllp_type			<= tlp_dllp_fifo_type;
							tx_dllp_payload			<= tlp_dllp_fifo_data;
							tlp_dllp_rd				<= 1;
						end

						//When timer wraps, send an UpdateFC-P the next chance we get
						else if(fc_timer >= fc_timer_max) begin
							next_fc_type_p			<= 0;
							fc_timer				<= 0;

							//TODO: use actual flow control credits
							//rather than always reporting max capacity
							tx_dllp_req				<= 1;
							tx_dllp_type			<= DLLP_TYPE_UPDATEFC_P;
							tx_dllp_payload			<= 0;
							tx_dllp_payload[21:14]	<= MAX_POSTED_HEADER;
							tx_dllp_payload[11:0]	<= MAX_POSTED_DATA;
						end

						//If we just sent an UpdateFC-P, send an UpdateFC-NP to follow it
						else if(!next_fc_type_p) begin
							next_fc_type_p			<= 1;

							//TODO: use actual flow control credits
							//rather than always reporting max capacity
							tx_dllp_req				<= 1;
							tx_dllp_type			<= DLLP_TYPE_UPDATEFC_NP;
							tx_dllp_payload			<= 0;
							tx_dllp_payload[21:14]	<= MAX_POSTED_HEADER;
							tx_dllp_payload[11:0]	<= MAX_POSTED_DATA;
						end

						//Don't need to control completions, we have infinite flow control capacity for those
					end

					//ready to go unless link drops
					if(!link_up) begin
						dl_state	<= DL_INACTIVE;
						dl_link_up	<= 0;
					end
				end

			endcase

		end

	end

endmodule
