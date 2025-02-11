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
	@brief Serial Chip to Chip Bus - low level bridge block without any transceiver specific wrappers

	External logic is responsible for 8b10b coding/decoding and aligning commas to lane 0

	APB write:
		K27.7 0xfb
		Address
		Data
		K28.6 0xdc end of frame
		CRC
 */
module SCCB_APBBridge #(

	parameter SYMBOL_WIDTH 	= 4,	//Typical config: 40 bit bus width for 7 series GTP or U+ GTY
									//(4x 8b10b symbols per block)
									//Note, on GTP this requires a 20 bit internal width and TXUSRCLK at 2x the rate
									//of TXUSRCLK2

	parameter TX_CDC_BYPASS	= 1,	//If set to 0, a CDC block will be added between apb_comp and the internal logic
									//adding a small amount of latency.
									//If set to 1, the CDC is bypassed and apb_comp.pclk must be tx_clk

	localparam DATA_WIDTH	= 8*SYMBOL_WIDTH	//SERDES data width for 8-bit data portion
) (
	//SERDES ports
	input wire						rx_clk,
	input wire[SYMBOL_WIDTH-1:0]	rx_kchar,
	input wire[DATA_WIDTH-1:0]		rx_data,
	input wire						rx_data_valid,

	input wire						tx_clk,
	output wire[SYMBOL_WIDTH_1:0]	tx_kchar,
	output wire[DATA_WIDTH-1:0]		tx_data,

	//APB ports
	//SERDES RX clock is used as requester clock
	//Completer clock can be anything if TX_CDC_BYPASS = 0; if pclk is tx_clk set TX_CDC_BYPASS=1 to reduce latency
	APB.requester					apb_req,
	APB.completer					apb_comp

	//TODO: AHB ports

	//TODO: AXI ports
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX: Clock domain crossing on the completer port

	APB #(.DATA_WIDTH(apb_comp.DATA_WIDTH), .ADDR_WIDTH(apb_comp.ADDR_WIDTH), .USER_WIDTH(0)) apb_comp_tx();

	//Bypass the TX CDC
	if(TX_CDC_BYPASS) begin
		APBRegisterSlice #(
			.UP_REG(0),
			.DOW_REG(0)
		) tx_bypass (
			.upstream(apb_comp),
			.downstream(apb_comp_tx)
		);

	end

	//Add a CDC
	else begin
		APB_CDC tx_cdc(
			.upstream(apb_comp),
			.downstream_pclk(tx_clk),
			.downstream(apb_comp_tx));
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Link layer processing

	SCCB_LinkLayer #(
		.SYMBOL_WIDTH(SYMBOL_WIDTH)
	) link_layer (
		.rx_clk(rx_clk),
		.rx_kchar(rx_kchar),
		.rx_data(rx_data),
		.rx_data_valid(rx_data_valid),

		.tx_clk(tx_clk),
		.tx_kchar(tx_kchar),
		.tx_data(tx_data)

		//TODO: link layer outputs
	);

endmodule
