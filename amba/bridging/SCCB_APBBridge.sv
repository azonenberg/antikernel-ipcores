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
	output wire						rx_ll_link_up,

	input wire						tx_clk,
	output wire[SYMBOL_WIDTH-1:0]	tx_kchar,
	output wire[DATA_WIDTH-1:0]		tx_data,
	output wire						tx_ll_link_up,

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
			.DOWN_REG(0)
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

	wire		rx_ll_start;
	wire		rx_ll_valid;
	wire[2:0]	rx_ll_nvalid;
	wire[31:0]	rx_ll_data;
	wire		rx_ll_commit;
	wire		rx_ll_drop;

	logic		tx_ll_start		= 0;
	logic[2:0]	tx_ll_valid		= 0;
	logic[31:0]	tx_ll_data		= 0;

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

		//TX side inputs from upper layer
		.tx_ll_link_up(tx_ll_link_up),
		.tx_ll_start(tx_ll_start),
		.tx_ll_valid(tx_ll_valid),
		.tx_ll_data(tx_ll_data),

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
	// Send completion when an APB transaction finishes

	//RX domain
	logic		completion_has_data;
	logic		completion_success;
	logic[31:0]	completion_data;

	//TX domain
	wire		completion_req_sync;
	wire		completion_has_data_sync;
	wire		completion_success_sync;
	wire[31:0]	completion_data_sync;

	RegisterSynchronizer #(
		.WIDTH(34),
		.INIT(34'h0),
		.IN_REG(0)
	) sync_apb_completion (
		.clk_a(rx_clk),
		.en_a(apb_req.pready),
		.ack_a(),
		.reg_a({ completion_has_data, completion_success, completion_data} ),

		.clk_b(tx_clk),
		.updated_b(completion_req_sync),
		.reset_b(1'b0),
		.reg_b({ completion_has_data_sync, completion_success_sync, completion_data_sync})
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive completions and push to the TX domain so we can ACK them

	logic		ack_pending = 0;
	logic		ack_error	= 0;
	logic[31:0]	ack_data	= 0;

	wire		ack_req_sync;
	wire		ack_error_sync;
	wire[31:0]	ack_data_sync;

	RegisterSynchronizer #(
		.WIDTH(33),
		.INIT(1'h0),
		.IN_REG(0)
	) sync_apb_ack (
		.clk_a(rx_clk),
		.en_a(ack_pending && rx_ll_commit),
		.ack_a(),
		.reg_a({ ack_error, ack_data }),

		.clk_b(tx_clk),
		.updated_b(ack_req_sync),
		.reset_b(1'b0),
		.reg_b({ ack_error_sync, ack_data_sync })
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB TX processing

	//Tie off unused APB signals
	assign apb_comp_tx.pruser = 0;
	assign apb_comp_tx.pbuser = 0;

	enum logic[2:0]
	{
		TX_STATE_IDLE,
		TX_STATE_APB_1,
		TX_STATE_APB_2,
		TX_STATE_COMPLETION_1
	} tx_state = TX_STATE_IDLE;

	logic	apb_tx_pending	= 0;
	logic	completion_pending	= 0;

	//Combinatorially assert PREADY/PSLVERR/PRDATA
	always_comb begin

		apb_comp_tx.prdata	= ack_data_sync;

		if(ack_req_sync) begin
			apb_comp_tx.pready	= 1;
			apb_comp_tx.pslverr	= ack_error_sync;
		end

		else begin
			apb_comp_tx.pready	= 0;
			apb_comp_tx.pslverr	= 0;
		end

	end

	always_ff @(posedge tx_clk) begin

		//Default to not sending any link layer data
		tx_ll_data	<= 0;
		tx_ll_start	<= 0;
		tx_ll_valid	<= 0;

		//Save completion-pending flag if we don't get to it this cycle because something else is going on
		if(completion_req_sync)
			completion_pending	<= 1;

		//Clear pending flag when we get an ACK
		if(ack_req_sync)
			apb_tx_pending		<= 0;

		case(tx_state)

			TX_STATE_IDLE: begin

				//Link down? Nothing to do
				if(!tx_ll_link_up) begin
				end

				//Send completion if needed
				else if(completion_req_sync || completion_pending) begin
					completion_pending			<= 0;

					tx_ll_start					<= 1;

					//Does the completion have data? Send the appropriate reply
					if(completion_has_data_sync) begin
						tx_ll_valid				<= 2;
						tx_ll_data[7:0]			<= APB_COMP_DATA;

						//placeholder for sequence number
						tx_ll_data[15:8]		<= 0;

						tx_ll_data[23:16]		<= completion_data_sync[31:24];
						tx_ll_data[31:24]		<= completion_data_sync[23:16];
						tx_state				<= TX_STATE_COMPLETION_1;
					end

					//No data, just acknowledge it
					else begin
						tx_ll_valid				<= 1;

						tx_ll_data[7:0]			<= APB_COMP_EMPTY;

						//placeholder for sequence number
						tx_ll_data[15:8]		<= 0;

						if(completion_success_sync)
							tx_ll_data[31:16]	<= 1;
						else
							tx_ll_data[31:16]	<= 0;

						//no state change needed, this was all we needed to do

					end

				end

				//Start a new APB transaction if we don't already have one in the pipe
				else if(apb_comp_tx.penable && !apb_comp_tx.pready && !apb_tx_pending) begin

					tx_ll_start				<= 1;
					tx_ll_valid				<= 2;

					//select the correct header byte
					if(apb_comp_tx.pwrite)
						tx_ll_data[7:0]		<= APB_WRITE;
					else
						tx_ll_data[7:0]		<= APB_READ;

					//placeholder for sequence number
					tx_ll_data[15:8]		<= 0;

					//payload
					if(apb_comp_tx.ADDR_WIDTH >= 24)
						tx_ll_data[23:16]	<= apb_comp_tx.paddr[apb_comp_tx.ADDR_WIDTH-1 : 24];
					else
						tx_ll_data[23:16]	<= 0;

					if(apb_comp_tx.ADDR_WIDTH >= 24)
						tx_ll_data[31:24]	<= apb_comp_tx.paddr[23:16];
					else if(apb_comp_tx.ADDR_WIDTH >= 16)
						tx_ll_data[31:24]	<= apb_comp_tx.paddr[apb_comp_tx.ADDR_WIDTH-1:16];
					else
						tx_ll_data[31:24]	<= 0;

					tx_state				<= TX_STATE_APB_1;
					apb_tx_pending			<= 1;

				end

			end //TX_STATE_IDLE

			//Continue an APB read or write
			TX_STATE_APB_1: begin

				tx_ll_data[7:0]			<= apb_comp_tx.paddr[15:8];
				tx_ll_data[15:8]		<= apb_comp_tx.paddr[7:0];

				if(apb_comp_tx.pwrite) begin
					tx_ll_valid			<= 4;
					tx_ll_data[23:16]	<= apb_comp_tx.pwdata[31:24];
					tx_ll_data[31:24]	<= apb_comp_tx.pwdata[23:16];

					tx_state			<= TX_STATE_APB_2;
				end

				//We just sent a read
				else begin
					tx_ll_valid			<= 2;
					tx_state			<= TX_STATE_IDLE;
				end

			end	//TX_STATE_APB_1

			//Continue an APB write
			TX_STATE_APB_2: begin
				tx_ll_valid			<= 2;
				tx_ll_data[31:16]	<= 0;
				tx_ll_data[7:0]		<= apb_comp_tx.pwdata[15:8];
				tx_ll_data[15:8]	<= apb_comp_tx.pwdata[7:0];

				tx_state			<= TX_STATE_IDLE;
			end //TX_STATE_APB_2

			//Continue a completion with data
			TX_STATE_COMPLETION_1: begin
				tx_ll_valid			<= 2;
				tx_ll_data[31:16]	<= 0;
				tx_ll_data[7:0]		<= completion_data_sync[15:8];
				tx_ll_data[15:8]	<= completion_data_sync[7:0];

				tx_state			<= TX_STATE_IDLE;
			end //TX_STATE_COMPLETION_1

		endcase

	end

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
		apb_req.paddr		= 0;
		apb_req.pstrb		= 0;
	end

	//Hook up clock
	assign apb_req.pclk = rx_clk;

	enum logic[2:0]
	{
		RX_STATE_IDLE,
		RX_STATE_WRITE_1,
		RX_STATE_WRITE_2,
		RX_STATE_WRITE_SELECT,
		RX_STATE_READ_1,
		RX_STATE_COMP_1
	} rx_state = RX_STATE_IDLE;

	//Combinatorially assert PENABLE and PSEL to reduce latency
	logic		penable_ff	= 0;
	logic		psel_ff		= 0;
	logic[31:0]	pwdata_ff	= 0;
	always_comb begin
		apb_req.penable		= penable_ff;
		apb_req.psel		= psel_ff;
		apb_req.pwdata		= pwdata_ff;

		if(apb_req.psel && rx_ll_commit)
			apb_req.penable	= 1;

		if(rx_ll_start  && ( (rx_ll_data[7:0] == APB_READ) || (rx_ll_data[7:0] == APB_WRITE) ) )
			apb_req.psel	= 1;

		if(rx_ll_drop) begin
			apb_req.penable	= 0;
			apb_req.psel	= 0;
		end

		if(rx_state == RX_STATE_WRITE_2) begin
			apb_req.pwdata[15:8]	= rx_ll_data[7:0];
			apb_req.pwdata[7:0]		= rx_ll_data[15:8];
		end

	end

	//Combinatorially send completions
	logic		completion_has_data_ff	= 0;
	logic		completion_success_ff	= 0;
	logic[31:0]	completion_data_ff		= 0;

	always_comb begin

		if(apb_req.pready) begin
			completion_has_data	<= !apb_req.pwrite && !apb_req.pslverr;
			completion_success	<= !apb_req.pslverr;
			completion_data		<= apb_req.prdata;
		end

		else begin
			completion_has_data	= completion_has_data_ff;
			completion_success	= completion_success_ff;
			completion_data		= completion_data_ff;
		end

	end

	always_ff @(posedge rx_clk) begin

		//Clear single cycle flags
		apb_req.preset_n	<= 1;

		//Pipeline APB combinatorial signals
		penable_ff				<= apb_req.penable;
		psel_ff					<= apb_req.psel;
		completion_has_data_ff	<= completion_has_data;
		completion_success_ff	<= completion_success;
		completion_data_ff		<= completion_data;
		pwdata_ff				<= apb_req.pwdata;

		//Handle acknowledgements
		if(apb_req.pready) begin
			psel_ff		 		<= 0;
			penable_ff		 	<= 0;
		end

		if(ack_pending && (rx_ll_commit || rx_ll_drop) )
			ack_pending	<= 0;

		case(rx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for start of frame
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

						//Start of an APB read request
						APB_READ: begin
							apb_req.paddr[31:24]	<= rx_ll_data[23:16];
							apb_req.paddr[23:16]	<= rx_ll_data[31:24];
							apb_req.pwrite			<= 0;
							apb_req.pstrb			<= 4'h0;
							rx_state				<= RX_STATE_READ_1;
						end

						//APB completion with no data (write acknowledgement or failure)
						APB_COMP_EMPTY: begin
							ack_pending				<= 1;
							ack_data				<= 0;
							ack_error				<= !rx_ll_data[16];
						end

						//APB completion with data (read acknowledgement)
						APB_COMP_DATA: begin
							ack_data[31:24]			<= rx_ll_data[23:16];
							ack_data[23:16]			<= rx_ll_data[31:24];

							ack_pending				<= 1;
							ack_error				<= 0;

							rx_state				<= RX_STATE_COMP_1;
						end

						//Ignore anything we don't understand
						default: begin
						end
					endcase

				end
			end //RX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Write path

			RX_STATE_WRITE_1: begin
				apb_req.paddr[15:8]		<= rx_ll_data[7:0];
				apb_req.paddr[7:0]		<= rx_ll_data[15:8];
				pwdata_ff[31:24]		<= rx_ll_data[23:16];
				pwdata_ff[23:16]		<= rx_ll_data[31:24];
				rx_state				<= RX_STATE_WRITE_2;

				//Handle drops
				if(rx_ll_drop || !rx_ll_valid)
					rx_state			<= RX_STATE_IDLE;
			end

			RX_STATE_WRITE_2: begin
				rx_state				<= RX_STATE_IDLE;
			end

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read path

			RX_STATE_READ_1: begin
				apb_req.paddr[15:8]		<= rx_ll_data[7:0];
				apb_req.paddr[7:0]		<= rx_ll_data[15:8];

				rx_state				<= RX_STATE_IDLE;
			end //RX_STATE_READ_1

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Completions

			RX_STATE_COMP_1: begin
				ack_data[15:8]			<= rx_ll_data[7:0];
				ack_data[7:0]			<= rx_ll_data[15:8];

				rx_state				<= RX_STATE_IDLE;

			end //RX_STATE_COMP_1

		endcase

		//If link is down, reset rx state machine
		if(!rx_ll_link_up)
			rx_state	<= RX_STATE_IDLE;

	end

endmodule
