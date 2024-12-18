`timescale 1ns/1ps
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

`include "../../../antikernel-ipcores/amba/apb/APBTypes.sv"

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	APB register access to a QSPI bus controller

	Includes a control for a single chip select pin. Additional chip selects, if needed, must be provided by
	a separate GPIO block.
 */
module APB_QSPIHostInterface #(

	//Indicates which edge of SCK the local end samples data on
	//NORMAL = same as remote
	//INVERTED = opposite
	localparam LOCAL_EDGE = "INVERTED"
)(

	//The APB bus
	APB.completer 					apb,

	//QSPI interface
	output wire						qspi_sck,
	output wire[3:0]				qspi_dq_out,
	input wire[3:0]					qspi_dq_in,
	output wire[3:0]				qspi_dq_tris,
	output logic					qspi_cs_n = 1
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32 bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	//Align all writable registers to 0x20 boundaries to work around STM32H7 OCTOSPI bugs
	//Registers are a strict superset of APB_SPIHostInterface
	typedef enum logic[8:0]
	{
		REG_CLK_DIV			= 'h00,		//clock divider from PCLK to SCK
		REG_DATA			= 'h20,		//[7:0] data to send/receive
		REG_CS_N			= 'h40,		//[0] = chip select output value
		REG_STATUS			= 'h60,		//[0] = busy flag
		REG_STATUS_2		= 'h80,		//duplicate of REG_STATUS for use with QSPI clients
		REG_BURST_RDLEN		= 'ha0,		//write number of bytes to read in a burst (up to 256)
		REG_QUAD_CAP		= 'hc0,		//always 0 for regular SPI, 1 for QSPI
		REG_QBURST_RDLEN	= 'he0,		//write number of bytes to read in a burst (up to 256)
		REG_BURST_RXBUF		= 'h100		//base address for receive buffer
										//Must be aligned to 64 byte boundary
										//Must be last register in the peripheral
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The SPI controller

	logic[15:0]	clkdiv	= 100;

	logic		shift_en;
	logic		quad_shift_en;
	logic[7:0]	shift_data;
	wire		shift_done;
	wire[7:0]	rx_data;
	logic		auto_restart;

	QSPIHostInterface #(
		.LOCAL_EDGE(LOCAL_EDGE)
	) spi(
		.clk(apb.pclk),
		.clkdiv(clkdiv),

		.qspi_sck(qspi_sck),
		.qspi_dq_in(qspi_dq_in),
		.qspi_dq_out(qspi_dq_out),
		.qspi_dq_tris(qspi_dq_tris),

		.shift_en(shift_en),
		.quad_shift_en(quad_shift_en),
		.shift_done(shift_done),
		.tx_data(shift_data),
		.rx_data(rx_data),
		.auto_restart(auto_restart));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Burst mode data buffer (only 32 bit accesses supported)

	//For now, max 256 bytes (64 32-bit words) supported.
	//RAM64M is 3 bits per slice (4 LUTS) so for 32 bits we should use 11 slices / 44 LUTs. Not too bad.
	logic[5:0]	burst_rptr 		= 0;
	logic[5:0]	burst_wptr 		= 0;
	logic[31:0] burst_wdata 	= 0;
	logic		burst_wr		= 0;
	logic[1:0]	burst_wvalid	= 0;
	logic[31:0] burst_rdata;

	logic[31:0] burst_buf[63:0];

	//Combinatorial read
	always_comb begin
		burst_rdata = burst_buf[burst_rptr];
	end

	//Registered write
	always_ff @(posedge apb.pclk) begin
		if(burst_wr)
			burst_buf[burst_wptr]	<= burst_wdata;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	logic		shift_busy	= 0;
	logic[8:0]	burst_count	= 0;

	enum logic[1:0]
	{
		BURST_IDLE,
		BURST_NORMAL,
		BURST_QUAD
	} burst_mode = BURST_IDLE;

	wire burst_active;
	assign burst_active = (burst_mode != BURST_IDLE);

	always_comb begin

		//Combinatorially assert PREADY when selected
		apb.pready		= apb.psel && apb.penable;

		//Default to no errors and no read data
		apb.prdata		= 0;
		apb.pslverr		= 0;

		//Clear control signals
		quad_shift_en	= 0;
		shift_en		= 0;
		shift_data		= 0;

		//Continue an existing burst
		auto_restart	= (burst_count > 1);

		//Combinatorially read burst buffer memory
		burst_rptr	= apb.paddr[7:2];

		if(apb.pready) begin

			if(apb.pwrite) begin

				case(apb.paddr)

					//Start a read or write
					REG_DATA: begin
						shift_en	= 1;
						shift_data	= apb.pwdata[7:0];
					end

					//write is sequential, just need to not trigger default case
					REG_CLK_DIV: begin
					end
					REG_CS_N: begin
					end

					//Send 0x00 and start a shift
					REG_BURST_RDLEN: begin
						shift_en		= 1;
						shift_data		= 8'h0;
					end

					REG_QBURST_RDLEN: begin
						quad_shift_en	= 1;
						shift_data		= 8'h0;
					end

					//unmapped address
					default:	apb.pslverr		= 1;

				endcase

			end

			else begin

				//read rx buffer
				if(apb.paddr >= REG_BURST_RXBUF)
					apb.prdata	= burst_rdata;

				else begin
					case(apb.paddr)

						REG_CLK_DIV:	apb.prdata	= clkdiv + 1;
						REG_DATA:		apb.prdata	= {23'h0, rx_data};
						REG_CS_N:		apb.prdata	= {31'h0, qspi_cs_n};
						REG_STATUS:		apb.prdata	= {31'h0, shift_busy | burst_active};
						REG_STATUS_2:	apb.prdata	= {31'h0, shift_busy | burst_active};
						REG_QUAD_CAP:	apb.prdata	= 32'h1;

						//unmapped address
						default:		apb.pslverr		= 1;

					endcase
				end

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			qspi_cs_n		<= 1;
			clkdiv			<= 100;
			shift_busy		<= 0;
			burst_mode		<= BURST_IDLE;
			burst_count		<= 0;
			burst_wr		<= 0;
			burst_wdata		<= 0;
			burst_wvalid	<= 0;
		end

		//Normal path
		else begin

			if(shift_en)
				shift_busy	<= 1;
			if(shift_done)
				shift_busy	<= 0;
			burst_wr	<= 0;

			//APB writes
			if(apb.pready && apb.pwrite) begin
				case(apb.paddr)
					REG_CS_N:		qspi_cs_n	<= apb.pwdata[0];
					REG_CLK_DIV:	clkdiv		<= apb.pwdata[15:0] - 1;

					//Start a burst read
					REG_BURST_RDLEN: begin
						burst_count	<= apb.pwdata[8:0];
						burst_mode	<= BURST_NORMAL;
						burst_wptr	<= 0;

						//clamp large burst sizes
						if(apb.pwdata[8:0] > 256)
							burst_count	<= 256;
					end

					REG_QBURST_RDLEN: begin
						burst_count	<= apb.pwdata[8:0];
						burst_mode	<= BURST_QUAD;
						burst_wptr	<= 0;

						//clamp large burst sizes
						if(apb.pwdata[8:0] > 256)
							burst_count	<= 256;
					end

					default: begin
					end
				endcase
			end

			//Handle writes
			if(burst_wr) begin
				burst_wvalid	<= 0;
				burst_wptr		<= burst_wptr + 1;
			end

			//Handle bursts
			if(burst_mode != BURST_IDLE) begin

				//End of a burst
				if(shift_done) begin

					//Save data little endian
					case(burst_wvalid)
						0: burst_wdata[7:0]		<= rx_data;
						1: burst_wdata[15:8]	<= rx_data;
						2: burst_wdata[23:16]	<= rx_data;

						3: begin
							burst_wdata[31:24]	<= rx_data;
							burst_wr			<= 1;
						end
					endcase

					//Count progress
					burst_count		<= burst_count - 1;
					burst_wvalid	<= burst_wvalid + 1;

					if(burst_count <= 1)
						burst_mode	<= BURST_IDLE;
				end

			end

		end

	end

endmodule
