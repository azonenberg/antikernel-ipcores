`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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

`include "GmiiBus.svh"
`include "EthernetBus.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief A simplified RMII MAC designed for talking to an MCU

	This MAC is intended to bridge from an FPGA design (perhaps using an external RGMII PHY) to an MCU which only has
	an RMII interface.

	As a result, the FPGA-side interface is the same gated GMII used elsewhere in antikernel-ipcores. The gate signal
	must be asserted with 1/16 duty cycle since we're serializing 32 to 2 bits.

	For now, insertion of padding is not implemented so frames must be padded to 64+ bytes externally.
 */
module SimpleRMIIMAC(

	//System synchronous clock used by the RMII link
	input wire					clk_50mhz,

	//FPGA-side interface
	input wire EthernetTxBus	mac_tx_bus,
	output logic				mac_tx_ready	= 1,
	output EthernetRxBus		mac_rx_bus,

	//MCU-side interface
	output logic				rmii_rx_en	= 0,
	output logic[1:0]			rmii_rxd	= 0,
	input wire					rmii_tx_en,
	input wire[1:0]				rmii_txd
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit side (MAC TX -> MCU RX)

	wire[31:0] tx_crc;
	CRC32_Ethernet_x32_variable tx_crc_calc(
		.clk(clk_50mhz),
		.reset(mac_tx_bus.start),
		.din_len(mac_tx_bus.bytes_valid),
		.din(mac_tx_bus.data),
		.crc_out(tx_crc)
	);

	enum logic[3:0]
	{
		TX_STATE_IDLE,
		TX_STATE_CARRIER_1,
		TX_STATE_CARRIER_2,
		TX_STATE_PREAMBLE_1,
		TX_STATE_PREAMBLE_2,
		TX_STATE_DATA,
		TX_STATE_DATA_LAST1,
		TX_STATE_DATA_LAST2,
		TX_STATE_CRC
	} tx_state = TX_STATE_IDLE;

	logic[3:0]	tx_count	= 0;

	logic[2:0]	tx_valid_ff	= 0;
	logic[31:0]	tx_data_ff	= 0;

	logic[2:0]	tx_valid_ff2	= 0;
	logic[31:0]	tx_data_ff2		= 0;

	logic[2:0]	tx_valid_ff3	= 0;
	logic[31:0]	tx_data_ff3		= 0;

	logic[2:0]	tx_valid_ff4	= 0;
	logic[31:0]	tx_data_ff4		= 0;

	always_ff @(posedge clk_50mhz) begin

		//Save incoming data
		if(mac_tx_bus.data_valid) begin
			tx_valid_ff		<= mac_tx_bus.bytes_valid;
			tx_data_ff		<= mac_tx_bus.data;
			tx_valid_ff2	<= tx_valid_ff;
			tx_data_ff2		<= tx_data_ff;
			tx_valid_ff3	<= tx_valid_ff2;
			tx_data_ff3		<= tx_data_ff2;
			tx_valid_ff4	<= tx_valid_ff3;
			tx_data_ff4		<= tx_data_ff3;
		end

		case(tx_state)

			//Wait for a frame to start
			TX_STATE_IDLE: begin

				if(rmii_rx_en) begin
					rmii_rx_en	<= 0;
					rmii_rxd	<= 0;
				end

				//When we get a start flag, we're no longer ready
				if(mac_tx_bus.start)
					mac_tx_ready	<= 0;

				//Data valid? Start sending a preamble
				if(mac_tx_bus.data_valid) begin
					tx_count	<= 0;
					tx_state	<= TX_STATE_PREAMBLE_1;
				end

			end	//end TX_STATE_IDLE

			//Send 4 bytes (16 cycles) of 0x55
			TX_STATE_PREAMBLE_1: begin
				rmii_rx_en		<= 1;
				rmii_rxd		<= 2'b01;

				tx_count		<= tx_count + 1;

				rmii_rx_en		<= 1;

				if(tx_count == 'd15)
					tx_state	<= TX_STATE_PREAMBLE_2;

			end	//end TX_STATE_PREAMBLE_1

			//Send 3 bytes (12 cycles) of 0x55 plus one byte (4 cycles) of 0xd5
			//Since RMII sends bit pairs low to high, the 2'b11 is the last of the 16 cycles in the word.
			TX_STATE_PREAMBLE_2: begin

				tx_count		<= tx_count + 1;

				rmii_rx_en		<= 1;

				if(tx_count == 'd15) begin
					rmii_rxd	<= 2'b11;
					tx_state	<= TX_STATE_DATA;
				end

			end	//end TX_STATE_PREAMBLE_2

			//Send 4 bytes (16 cycles) of data
			//Confusing ordering because EthernetTxBus sends most significant *byte* first, but RMII
			//sends least significant *bit* pair first
			TX_STATE_DATA: begin

				tx_count			<= tx_count + 1;

				rmii_rx_en			<= 1;
				case(tx_count[3:2])
					0:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 24) +: 2];
					1:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 16) +: 2];
					2:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 8) +: 2];
					3:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 0) +: 2];
				endcase

				//At end of cycle, but no new data coming in? Last word.
				//Cycle the shift register manually, because there's no input to do it for us
				if( (tx_count == 15) && !mac_tx_bus.data_valid) begin
					tx_state			<= TX_STATE_DATA_LAST1;

					tx_valid_ff2	<= tx_valid_ff;
					tx_data_ff2		<= tx_data_ff;
					tx_valid_ff3	<= tx_valid_ff2;
					tx_data_ff3		<= tx_data_ff2;
				end

			end	//end TX_STATE_DATA

			//Send the last guaranteed-full word of packet content
			TX_STATE_DATA_LAST1: begin

				tx_count			<= tx_count + 1;

				rmii_rx_en			<= 1;
				case(tx_count[3:2])
					0:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 24) +: 2];
					1:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 16) +: 2];
					2:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 8) +: 2];
					3:	rmii_rxd	<= tx_data_ff3[(tx_count[1:0]*2 + 0) +: 2];
				endcase

				//Cycle the shift register for the final time
				if(tx_count == 15) begin
					tx_state		<= TX_STATE_DATA_LAST2;
					tx_valid_ff3	<= tx_valid_ff2;
					tx_data_ff3		<= tx_data_ff2;
				end

			end	//end TX_STATE_DATA_LAST1

			//Send the very last word of packet content (might be partial)
			TX_STATE_DATA_LAST2: begin

				tx_count				<= tx_count + 1;

				rmii_rx_en				<= 1;

				case(tx_count[3:2])
					0:	begin
						rmii_rxd		<= tx_data_ff3[(tx_count[1:0]*2 + 24) +: 2];

						if( (tx_count[1:0] == 2'b11) && (tx_valid_ff3 == 1) ) begin
							tx_state	<= TX_STATE_CRC;
							tx_count		<= 0;
						end
					end

					1:	begin
						rmii_rxd		<= tx_data_ff3[(tx_count[1:0]*2 + 16) +: 2];

						if( (tx_count[1:0] == 2'b11) && (tx_valid_ff3 == 2) ) begin
							tx_state	<= TX_STATE_CRC;
							tx_count		<= 0;
						end
					end

					2:	begin
						rmii_rxd		<= tx_data_ff2[(tx_count[1:0]*2 + 8) +: 2];

						if( (tx_count[1:0] == 2'b11) && (tx_valid_ff3 == 3) ) begin
							tx_state	<= TX_STATE_CRC;
							tx_count		<= 0;
						end
					end

					3:	begin
						rmii_rxd		<= tx_data_ff2[(tx_count[1:0]*2 + 0) +: 2];

						if(tx_count[1:0] == 2'b11) begin
							tx_state		<= TX_STATE_CRC;
							tx_count		<= 0;
						end
					end

				endcase

			end	//end TX_STATE_DATA_LAST2

			//And finally, send the CRC32
			TX_STATE_CRC: begin

				tx_count			<= tx_count + 1;

				rmii_rx_en			<= 1;
				case(tx_count[3:2])
					0:	rmii_rxd	<= tx_crc[(tx_count[1:0]*2 + 24) +: 2];
					1:	rmii_rxd	<= tx_crc[(tx_count[1:0]*2 + 16) +: 2];
					2:	rmii_rxd	<= tx_crc[(tx_count[1:0]*2 + 8) +: 2];
					3:	rmii_rxd	<= tx_crc[(tx_count[1:0]*2 + 0) +: 2];
				endcase

				//Done?
				if(tx_count == 15) begin
					tx_state		<= TX_STATE_IDLE;
					mac_tx_ready	<= 1;
				end

			end	//end TX_STATE_CRC

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive side (MCU TX -> MAC RX)

	enum logic[3:0]
	{
		RX_STATE_IDLE,
		RX_STATE_CARRIER,
		RX_STATE_PREAMBLE,
		RX_STATE_DATA,
		RX_STATE_CRC_0,
		RX_STATE_CRC_1
	} rx_state = RX_STATE_IDLE;

	logic[2:0]	rx_bitcount			= 0;
	logic[7:0]	rx_byte				= 0;
	logic		rx_byte_valid		= 0;

	logic		rx_data_valid_adv	= 0;
	logic[2:0]	rx_bytes_valid_adv	= 0;
	logic[31:0]	rx_data_adv			= 0;

	logic		rx_data_valid_adv2	= 0;
	logic[31:0]	rx_data_adv2		= 0;

	wire[31:0]	rx_crc_calculated;
	CRC32_Ethernet rx_crc_calc(
		.clk(clk_50mhz),
		.reset(mac_rx_bus.start),
		.update(rx_byte_valid),
		.din(rx_byte),
		.crc_flipped(rx_crc_calculated)
	);

	logic[31:0]	rx_crc_expected		= 0;

	logic		rx_byte_valid_ff	= 0;
	logic[31:0]	rx_crc_calculated_ff	= 0;
	logic[31:0]	rx_crc_calculated_ff2	= 0;
	logic[31:0]	rx_crc_calculated_ff3	= 0;
	logic[31:0]	rx_crc_calculated_ff4	= 0;
	logic[31:0]	rx_crc_calculated_ff5	= 0;

	always_ff @(posedge clk_50mhz) begin

		mac_rx_bus.start		<= 0;
		mac_rx_bus.data_valid	<= 0;
		mac_rx_bus.commit		<= 0;
		mac_rx_bus.drop			<= 0;

		rx_data_valid_adv		<= 0;
		rx_byte_valid			<= 0;

		//Convert bytes to words
		if(rx_byte_valid) begin

			case(rx_bytes_valid_adv)
				0:	rx_data_adv[31:24]	<= rx_byte;
				1:	rx_data_adv[23:16]	<= rx_byte;
				2:	rx_data_adv[15:8]	<= rx_byte;
				3: begin
					rx_data_adv[7:0]	<= rx_byte;
					rx_data_valid_adv	<= 1;
				end
			endcase

			rx_crc_expected		<= {rx_crc_expected[23:0], rx_byte};
			rx_bytes_valid_adv	<= rx_bytes_valid_adv + 1;
		end

		//Push data words down pipeline and out
		//Need delay so we don't accidentally send CRC
		if(rx_data_valid_adv) begin
			rx_data_valid_adv2		<= 1;
			rx_data_adv2			<= rx_data_adv;
			rx_bytes_valid_adv		<= 0;

			//Drive full words
			if(rx_data_valid_adv2) begin
				mac_rx_bus.data_valid	<= 1;
				mac_rx_bus.data			<= rx_data_adv2;
				mac_rx_bus.bytes_valid	<= 4;
			end

		end

		//Last partial word
		if(rx_byte_valid_ff && !rmii_tx_en) begin
			mac_rx_bus.data_valid	<= 1;
			mac_rx_bus.data			<= rx_data_adv2;
			mac_rx_bus.bytes_valid	<= rx_bytes_valid_adv;
			rx_data_valid_adv2		<= 0;
		end

		//Push CRC down pipeline
		rx_byte_valid_ff	<= rx_byte_valid;
		if(rx_byte_valid_ff) begin
			rx_crc_calculated_ff5	<= rx_crc_calculated_ff4;
			rx_crc_calculated_ff4	<= rx_crc_calculated_ff3;
			rx_crc_calculated_ff3	<= rx_crc_calculated_ff2;
			rx_crc_calculated_ff2	<= rx_crc_calculated_ff;
			rx_crc_calculated_ff	<= rx_crc_calculated;
		end

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for a frame to start

			RX_STATE_IDLE: begin

				if(rmii_tx_en)
					rx_state		<= RX_STATE_CARRIER;

			end	//end RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Preamble and header

			RX_STATE_CARRIER: begin
				if(!rmii_tx_en)
					rx_state		<= RX_STATE_IDLE;

				else begin
					if(rmii_txd == 2'b01)
						rx_state	<= RX_STATE_PREAMBLE;
				end

			end	//end RX_STATE_CARRIER

			RX_STATE_PREAMBLE: begin

				if(!rmii_tx_en)
					rx_state		<= RX_STATE_IDLE;

				//Got a SFD
				else if(rmii_txd == 2'b11) begin
					rx_bitcount			<= 0;
					rx_byte			<= 0;
					rx_data_adv			<= 0;
					rx_data_valid_adv	<= 0;
					rx_bytes_valid_adv	<= 0;
					mac_rx_bus.start	<= 1;
					rx_state			<= RX_STATE_DATA;
				end

			end	//end RX_STATE_PREAMBLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Frame data

			RX_STATE_DATA: begin

				if(!rmii_tx_en)
					rx_state		<= RX_STATE_CRC_0;

				else begin
					rx_bitcount		<= rx_bitcount + 2;
					rx_byte			<= { rmii_txd, rx_byte[7:2] };

					if(rx_bitcount == 6)
						rx_byte_valid	<= 1;
				end

			end	//end RX_STATE_DATA

			RX_STATE_CRC_0: begin
				rx_state			<= RX_STATE_CRC_1;
			end	//end RX_STATE_CRC_0

			RX_STATE_CRC_1: begin
				if(rx_crc_expected == rx_crc_calculated_ff5)
					mac_rx_bus.commit	<= 1;
				else
					mac_rx_bus.drop		<= 1;

				rx_state			<= RX_STATE_IDLE;
			end	//end RX_STATE_CRC_1

		endcase

	end

endmodule
