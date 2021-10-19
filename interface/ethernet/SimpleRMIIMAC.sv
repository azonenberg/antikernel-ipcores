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

	//MCU-side interface
	output logic				rmii_rx_en	= 0,
	output logic[1:0]			rmii_rxd	= 0
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
	// Logic analyzer for debug

	ila_0 ila(
		.clk(clk_50mhz),
		.probe0(rmii_rx_en),
		.probe1(rmii_rxd),
		.probe2(mac_tx_bus),
		.probe3(tx_state),
		.probe4(tx_count),
		.probe5(tx_crc),
		.probe6(tx_valid_ff),
		.probe7(tx_data_ff),
		.probe8(tx_valid_ff2),
		.probe9(tx_data_ff2),
		.probe10(tx_valid_ff3),
		.probe11(tx_data_ff3),
		.probe12(mac_tx_ready),
		.probe13(tx_valid_ff4),
		.probe14(tx_data_ff4)
	);

endmodule
