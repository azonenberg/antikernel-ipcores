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

/**
	@file
	@author Andrew D. Zonenberg
	@brief SPI slave-mode transceiver based on oversampling.

	clk frequency must be at least 4x SCK frequency.
 */
module SPISlave(
	input wire			clk,

	input wire			spi_mosi,
	input wire			spi_sck,
	input wire			spi_cs_n,
	output logic		spi_miso = 0,

	output wire			cs_falling,
	input wire[7:0]		tx_data,
	input wire			tx_data_valid,
	output logic[7:0]	rx_data			= 0,
	output logic		rx_data_valid	= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize all of the inputs to our local clock domain

	wire	spi_mosi_sync;
	wire	spi_sck_sync;
	wire	spi_cs_n_sync;

	ThreeStageSynchronizer #(.IN_REG(0)) sync_mosi (
		.clk_in(clk),
		.din(spi_mosi),
		.clk_out(clk),
		.dout(spi_mosi_sync));

	ThreeStageSynchronizer #(.IN_REG(0)) sync_sck (
		.clk_in(clk),
		.din(spi_sck),
		.clk_out(clk),
		.dout(spi_sck_sync));

	ThreeStageSynchronizer #(.IN_REG(0)) sync_cs_n (
		.clk_in(clk),
		.din(spi_cs_n),
		.clk_out(clk),
		.dout(spi_cs_n_sync));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Detect falling CS# edge and use this to (synchronously) reset byte indexing etc

	logic	spi_cs_n_ff	= 1;
	logic	spi_sck_ff = 0;

	always_ff @(posedge clk) begin
		spi_cs_n_ff	<= spi_cs_n_sync;
		spi_sck_ff	<= spi_sck_sync;
	end

	assign	cs_falling = (spi_cs_n_ff && !spi_cs_n_sync);
	wire	sck_rising = (!spi_sck_ff && spi_sck_sync);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Transceiver logic

	logic[2:0] nbit = 0;
	logic[7:0] rx_temp = 0;
	logic[7:0] tx_temp = 0;

	always_comb
		spi_miso	= tx_temp[7];

	always_ff @(posedge clk) begin

		rx_data_valid	<= 0;

		if(cs_falling) begin
			nbit		<= 0;
			rx_temp		<= 0;
			tx_temp		<= tx_data;
		end

		if(sck_rising) begin
			nbit		<= nbit + 1'h1;
			rx_temp		<= {rx_temp[6:0], spi_mosi_sync};
			tx_temp		<= {tx_temp[6:0], 1'b0 };

			if(nbit == 7) begin
				rx_data			<= {rx_temp[6:0], spi_mosi_sync};
				rx_data_valid	<= 1;
			end
		end

		if(tx_data_valid)
			tx_temp		<= tx_data;
	end

endmodule
