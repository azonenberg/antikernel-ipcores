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

	localparam DATA_WIDTH	= 8*SYMBOL_WIDTH,	//SERDES data width for 8-bit data portion

	localparam APB_WRITE		= 8'hfb,	//K27.7
	localparam APB_READ			= 8'h1c,	//K28.0
	localparam APB_COMP_DATA	= 8'hfd,	//K29.7
	localparam APB_COMP_EMPTY	= 8'hf7		//K23.7
) (
	//SERDES ports
	input wire						rx_clk,
	input wire[SYMBOL_WIDTH-1:0]	rx_kchar,
	input wire[DATA_WIDTH-1:0]		rx_data,
	input wire						rx_data_valid,

	input wire						tx_clk,
	output wire[SYMBOL_WIDTH-1:0]	tx_kchar,
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
	// Link layer block

	//TODO: handle widths other than 32 bits

	wire		rx_ll_link_up;
	wire		rx_ll_start;
	wire		rx_ll_valid;
	wire[2:0]	rx_ll_nvalid;
	wire[31:0]	rx_ll_data;
	wire		rx_ll_commit;
	wire		rx_ll_drop;

	SCCB_LinkLayer #(
		.SYMBOL_WIDTH(SYMBOL_WIDTH)
	) link_layer (
		.rx_clk(rx_clk),
		.rx_kchar(rx_kchar),
		.rx_data(rx_data),
		.rx_data_valid(rx_data_valid),

		.tx_clk(tx_clk),
		.tx_kchar(tx_kchar),
		.tx_data(tx_data),

		//TODO: transmit side of the bridge
		.tx_ll_link_up(),
		.tx_ll_start(0),
		.tx_ll_valid(0),
		.tx_ll_data(0),

		//RX side outputs to upper layer
		.rx_ll_link_up(rx_ll_link_up),
		.rx_ll_start(rx_ll_start),
		.rx_ll_valid(rx_ll_valid),
		.rx_ll_nvalid(rx_ll_nvalid),
		.rx_ll_data(rx_ll_data),
		.rx_ll_commit(rx_ll_commit),
		.rx_ll_drop(rx_ll_drop)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB RX processing

	//Tie off unused APB signals
	assign apb_req.pprot 	= 0;
	assign apb_req.pwakeup 	= 0;
	assign apb_req.pauser	= 0;
	assign apb_req.pwuser	= 0;

	//Reset starts asserted, then clears after first cycle
	initial begin
		apb_req.preset_n	= 0;
		apb_req.penable 	= 0;
		apb_req.psel		= 0;
	end

	//Hook up clock
	assign apb_req.pclk = rx_clk;

	enum logic[2:0]
	{
		RX_STATE_IDLE,
		RX_STATE_WRITE_1,
		RX_STATE_WRITE_2,
		RX_STATE_WRITE_SELECT
	} rx_state = RX_STATE_IDLE;

	always_ff @(posedge rx_clk) begin

		apb_req.preset_n	<= 0;

		//Clear pending APB request when we get an acknowledgement
		if(apb_req.pready) begin
			apb_req.psel 	<= 0;
			apb_req.penable <= 0;
		end

		case(rx_state)

			//Wait for start of frame
			RX_STATE_IDLE: begin
				if(rx_ll_start) begin

					//First byte is the frame type
					case(rx_ll_data[7:0])

						//Start of an APB write request
						//Followed by address and data
						//For now, always assume both are 32 bits in size
						APB_WRITE: begin
							//second byte is sequence number
							//then high 16 of address
							apb_req.paddr[31:24]	<= rx_ll_data[23:16];
							apb_req.paddr[23:16]	<= rx_ll_data[31:24];
							apb_req.pwrite			<= 1;
							apb_req.pstrb			<= 4'b1111;
							rx_state				<= RX_STATE_WRITE_1;
						end

						//Ignore anything we don't understand
						default: begin
						end
					endcase

				end
			end //RX_STATE_IDLE

			RX_STATE_WRITE_1: begin
				apb_req.paddr[15:8]		<= rx_ll_data[7:0];
				apb_req.paddr[7:0]		<= rx_ll_data[15:8];
				apb_req.pwdata[31:24]	<= rx_ll_data[23:16];
				apb_req.pwdata[23:16]	<= rx_ll_data[31:24];
				rx_state				<= RX_STATE_WRITE_2;

				//Handle drops
				if(rx_ll_drop || !rx_ll_valid)
					rx_state	<= RX_STATE_IDLE;
			end

			RX_STATE_WRITE_2: begin
				apb_req.pwdata[15:8]	<= rx_ll_data[7:0];
				apb_req.pwdata[7:0]		<= rx_ll_data[15:8];
				apb_req.psel			<= 1;
				rx_state				<= RX_STATE_WRITE_SELECT;

				//Handle drops
				if(rx_ll_drop || !rx_ll_valid)
					rx_state	<= RX_STATE_IDLE;
			end

			RX_STATE_WRITE_SELECT: begin

				//Handle drops
				if(rx_ll_drop) begin
					apb_req.psel	<= 0;
					rx_state		<= RX_STATE_IDLE;
				end

				//Dispatch the APB traffic on commit
				if(rx_ll_commit) begin
					apb_req.penable	<= 1;
					rx_state		<= RX_STATE_IDLE;
				end

			end

		endcase

		//If link is down, reset rx state machine
		if(!rx_ll_link_up)
			rx_state	<= RX_STATE_IDLE;

	end

endmodule
