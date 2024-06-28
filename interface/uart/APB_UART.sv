`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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
	@brief APB wrapper around a UART including TX/RX FIFOs
 */
module APB_UART #(
	parameter TX_FIFO_SIZE	= 2048,
	parameter RX_FIFO_SIZE	= 2048
)(
	//The APB bus
	APB.completer 		apb,

	//UART signals
	input wire			rx,
	output wire			tx
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support up to 32-bit APB due to register alignment, throw synthesis error for anything else

	if(apb.DATA_WIDTH > 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[3:0]
	{
		REG_STATUS		= 'h00,		//0 = TX FIFO full
									//1 = RX FIFO data ready to read

		REG_CLK_DIV		= 'h04,		//clock divisor from PCLK to baud
		REG_TX_DATA		= 'h08,		//write to the TX FIFO
		REG_RX_DATA		= 'h0c		//read from the RX fifo
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual UART

	logic[15:0]	clkdiv = 0;

	wire		rx_data_ready;
	wire[7:0]	rx_data;

	wire[7:0]	tx_data;
	logic		tx_en	= 0;
	wire		tx_done;

	UART uart(
		.clk(apb.pclk),
		.clkdiv(clkdiv),
		.rx(rx),
		.rxactive(),
		.rx_data(rx_data),
		.rx_en(rx_data_ready),

		.tx(tx),
		.tx_data(tx_data),
		.tx_en(tx_en),
		.txactive(),
		.tx_done(tx_done));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transmit FIFO

	logic	tx_fifo_wr;
	logic	tx_fifo_rd = 0;
	wire	tx_fifo_empty;
	wire	tx_fifo_full;

	SingleClockFifo #(
		.WIDTH(8),
		.DEPTH(TX_FIFO_SIZE),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) tx_fifo (
		.clk(apb.pclk),

		.wr(tx_fifo_wr),
		.din(apb.pwdata[7:0]),

		.rd(tx_fifo_rd),
		.dout(tx_data),

		.overflow(),
		.underflow(),

		.empty(tx_fifo_empty),
		.full(tx_fifo_full),

		.rsize(),
		.wsize(),

		.reset(!apb.preset_n)
	);

	logic	tx_busy	= 0;

	always_ff @(posedge apb.pclk) begin

		tx_en		<= tx_fifo_rd;
		tx_fifo_rd	<= 0;

		if(tx_done)
			tx_busy	<= 0;

		if(!tx_busy && !tx_fifo_empty) begin
			tx_fifo_rd	<= 1;
			tx_busy		<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receive FIFO TODO

	logic		rx_fifo_rd;
	wire[7:0]	rx_fifo_rd_data;
	wire		rx_fifo_empty;

	SingleClockFifo #(
		.WIDTH(8),
		.DEPTH(RX_FIFO_SIZE),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) rx_fifo (
		.clk(apb.pclk),

		.wr(rx_data_ready),
		.din(rx_data),

		.rd(rx_fifo_rd),
		.dout(rx_fifo_rd_data),

		.overflow(),
		.underflow(),

		.empty(rx_fifo_empty),
		.full(),

		.rsize(),
		.wsize(),

		.reset(!apb.preset_n)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	logic				apb_pready_next;

	//Combinatorial readback, but with one cycle of latency because of registered FIFO reads

	always_comb begin

		apb_pready_next	= apb.psel && apb.penable && !apb.pready;

		apb.prdata	= 0;
		apb.pslverr	= 0;

		tx_fifo_wr	= apb.pready && (apb.paddr == REG_TX_DATA) && apb.pwrite;
		rx_fifo_rd	= apb_pready_next && !apb.pwrite && (apb.paddr == REG_RX_DATA);

		//this is one cycle after the request is valid due to pipeline latency on reads
		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin

				case(apb.paddr)
					REG_STATUS:		apb.prdata	= { 30'h0, !rx_fifo_empty, tx_fifo_full };
					REG_CLK_DIV:	apb.prdata	= { 16'h0, clkdiv };
					REG_TX_DATA:	apb.pslverr	= 1;
					REG_RX_DATA:	apb.prdata	= rx_fifo_rd_data;
					default:		apb.pslverr	= 1;
				endcase

			end

			//write
			else begin

				case(apb.paddr)

					REG_STATUS: 	apb.pslverr	= 1;

					REG_CLK_DIV: begin
					end

					REG_TX_DATA: begin
					end

					REG_RX_DATA:	apb.pslverr	= 1;
					default:		apb.pslverr	= 1;

				endcase

			end
		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			apb.pready			<= 0;
			clkdiv				<= 0;
		end

		//Normal path
		else begin

			//Register request flags
			//address/write data don't need to be registered, they'll be kept stable
			apb.pready		<= apb_pready_next;

			//Handle writes
			if(apb.pready && apb.pwrite) begin

				case(apb.paddr)

					REG_CLK_DIV: clkdiv	<= apb.pwdata;

					//TX_DATA writes handled in fifo block

					default: begin
					end

				endcase

			end

		end

	end

endmodule
