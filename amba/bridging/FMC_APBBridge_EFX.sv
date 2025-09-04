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
	@brief A bridge from the STM32 FMC to 32 and 64 bit APBv5 for Efinix FPGAs

	The FMC clock is used as the APB PCLK and is expected to be free-running.

	TODO: add APB completer to query performance counters

	The bridge maps to a 32-bit APB segment mapped starting at 0000_0000,
	and a 64-bit segment mapped starting at BASE_X64.
 */
module FMC_APBBridge_EFX #(
	parameter BASE_X64				= 'h0800000,	//Base address for 64-bit segment

	//number of cycles it takes for a wait cycle on write to end before the new data gets here
	localparam WAIT_CYCLE_RTT		= 1
)(

	//APB root bus to interconnect bridge
	APB.requester		apb_x32,
	APB.requester		apb_x64,

	//Clock control signals
	input wire			pll_lock,

	//FMC pins to MCU
	input wire			fmc_clk,
	output logic		fmc_nwait,
	input wire			fmc_noe,
	inout wire[15:0]	fmc_ad,
	input wire			fmc_nwe,
	input wire[1:0]		fmc_nbl,
	input wire			fmc_nl_nadv,
	input wire[9:0]		fmc_a_hi,			//high address bits (optional, as needed for desired address space size)
	input wire			fmc_cs_n			//bank 1 chip select (NE1)
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Validate width of each bus

	if(apb_x32.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();
	if(apb_x64.DATA_WIDTH != 64)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O buffers

	wire[15:0]	adbus_in;
	logic[15:0]	adbus_out = 0;

	BidirectionalBuffer #(
		.WIDTH(16),
		.OE_INVERT(0)
	) ad_iobuf (
		.fabric_in(adbus_in),
		.fabric_out(adbus_out),
		.pad(fmc_ad),
		.oe(!fmc_noe)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Recovery timeout: abort if peripheral takes too long to process a transaction (keeps MCU from getting stuck)

	logic[7:0]	stuck_count = 0;
	logic		stuck_release	= 0;

	always_ff @(posedge apb_x32.pclk or negedge apb_x32.preset_n) begin
		if(!apb_x32.preset_n) begin
			stuck_count		<= 0;
			stuck_release	<= 0;
		end

		else begin

			stuck_release	<= 0;

			//start counting at beginning of transaction
			if(apb_x32.psel && !apb_x32.penable)
				stuck_count	<= 0;

			//continue counting if active
			if(apb_x32.penable) begin
				stuck_count	<= stuck_count + 1;

				if(stuck_count == 8'hff)
					stuck_release	<= 1;
			end

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Hold APB segments in reset until PLL locks, then keep asserted for a few clocks after

	assign	apb_x32.pclk	= fmc_clk;
	assign	apb_x64.pclk	= fmc_clk;

	logic	rst_n	= 0;
	assign	apb_x32.preset_n	= rst_n;
	assign	apb_x64.preset_n	= rst_n;
	logic[3:0]	rst_count	= 1;

	always_ff @(posedge apb_x32.pclk or negedge pll_lock) begin

		//Assert reset if PLL loses lock
		if(!pll_lock) begin
			rst_n		<= 0;
			rst_count	<= 1;
		end

		//Once PLL is locked, hold reset for 15 clocks
		else if(!rst_n) begin
			rst_count	<= rst_count + 1;
			if(rst_count == 0)
				rst_n	<= 1;

			//If bus gets stuck, reset it
			if(stuck_release) begin
				rst_n		<= 0;
				rst_count	<= 1;
			end
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	//Tie off unused signals
	assign apb_x32.pprot 	= 0;
	assign apb_x32.pwakeup 	= 0;
	assign apb_x32.pauser	= 0;
	assign apb_x32.pwuser	= 0;

	assign apb_x64.pprot 	= 0;
	assign apb_x64.pwakeup 	= 0;
	assign apb_x64.pauser	= 0;
	assign apb_x64.pwuser	= 0;

	enum logic[3:0]
	{
		STATE_ADDR,
		STATE_WAIT,
		STATE_WAIT2,
		STATE_WDATA_LO,
		STATE_WDATA_HI,
		STATE_WDATA_LO2,
		STATE_WDATA_HI2,
		STATE_WDATA_WAIT,
		STATE_RDATA_LO,
		STATE_RDATA_HI,
		STATE_RD_END,
		STATE_ACTIVE,

		STATE_LAST	//unused
	} state = STATE_ADDR;

	//Saved address/write enable flag before we started a write
	logic[25:0]		pending_addr	= 0;
	logic			pending_write	= 0;
	logic			apb_busy		= 0;
	logic[3:0]		wait_count		= 0;

	//Saved read data
	logic[31:0]		prdata_latched	= 0;

	always_ff @(posedge apb_x32.pclk or negedge pll_lock) begin

		if(!pll_lock) begin
			fmc_nwait	<= 1;
		end

		else begin

			//Default to not waiting
			fmc_nwait		<= 1;

			//Transaction active? Might have wait states
			if(!fmc_cs_n) begin

				//Block if APB is busy
				if(apb_busy)
					fmc_nwait	<= 0;

				//Block if transaction is being dispatched
				if( (state == STATE_WAIT) && !pending_write && !apb_busy )
					fmc_nwait	<= 0;

			end

		end

	end

	always_ff @(posedge apb_x32.pclk) begin

		case(state)
			STATE_RDATA_LO:	adbus_out	<= prdata_latched[15:0];
			STATE_RDATA_HI:	adbus_out	<= prdata_latched[31:16];
			default:		adbus_out	<= 0;
		endcase

	end

	//Figure out if address is in the 32 or 64 bit segment
	logic		addr_is_x64 = 0;

	wire[25:0]	pending_addr_next;
	assign pending_addr_next = { fmc_a_hi, adbus_in, 1'b0};

	logic		apb_busy_ff	= 0;
	always_ff @(posedge apb_x32.pclk) begin

		apb_busy_ff	<= apb_busy;

		//Activate
		if(apb_x32.psel && !apb_x32.penable)
			apb_x32.penable	<= 1;
		if(apb_x64.psel && !apb_x64.penable)
			apb_x64.penable	<= 1;

		//Complete a transaction
		if(apb_x32.pready) begin
			apb_x32.penable		<= 0;
			apb_x32.psel		<= 0;
			apb_busy			<= 0;
			prdata_latched		<= apb_x32.prdata;
		end

		if(apb_x64.pready) begin
			apb_x64.penable		<= 0;
			apb_x64.psel		<= 0;
			apb_busy			<= 0;
			prdata_latched		<= apb_x64.prdata;
		end

		//Selected! Let's do something
		if(!fmc_cs_n) begin

			case(state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Common path

				//Save address and write enable flag
				//LSB of address is always implicitly zero because of 16 bit bus width
				STATE_ADDR: begin

					pending_addr	<= pending_addr_next;
					addr_is_x64		<= (pending_addr_next >= BASE_X64);
					pending_write	<= !fmc_nwe;

					//Move on as soon as we get the address latched
					if(!fmc_nl_nadv)
						state				<= STATE_WAIT;

				end

				//Wait state
				STATE_WAIT: begin

					//it's a write
					if(pending_write)
						state				<= STATE_WAIT2;

					//it's a read, dispatch it when available
					else if(!apb_busy) begin

						//32-bit requests
						if(addr_is_x64) begin
							apb_x64.paddr		<= pending_addr;
							apb_x64.pwrite		<= pending_write;
							apb_x64.psel		<= 1;
							apb_busy			<= 1;
						end

						//64-bit requests
						else begin
							apb_x32.paddr		<= pending_addr;
							apb_x32.pwrite		<= pending_write;
							apb_x32.psel		<= 1;
							apb_busy			<= 1;
						end

						//Waiting for read data to come back
						state				<= STATE_RDATA_LO;
					end

				end

				//second wait state
				STATE_WAIT2: begin
					if(!apb_busy_ff)
						state		<= STATE_WDATA_LO;
				end

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Writes

				//Word 0 of write data (low half of x32, low half of first word for x64)
				STATE_WDATA_LO: begin
					apb_x32.pstrb[1:0]		<= ~fmc_nbl;
					apb_x32.pwdata[15:0]	<= adbus_in;

					apb_x64.pstrb[5:4]		<= ~fmc_nbl;
					apb_x64.pwdata[47:32]	<= adbus_in;

					if(apb_busy_ff) begin
						state			<= STATE_WDATA_WAIT;
						wait_count		<= 0;
					end
					else
						state			<= STATE_WDATA_HI;

				end

				//wait state is active, delay for several clocks
				STATE_WDATA_WAIT: begin
					if(!apb_busy_ff) begin
						wait_count		<= wait_count + 1;
						if(wait_count >= WAIT_CYCLE_RTT)
							state		<= STATE_WDATA_HI;
					end
				end

				//Word 1 of write data (high half of x32, high half of first word for x64)
				STATE_WDATA_HI: begin
					apb_x32.pstrb[3:2]		<= ~fmc_nbl;
					apb_x32.pwdata[31:16]	<= adbus_in;

					apb_x64.pstrb[7:6]		<= ~fmc_nbl;
					apb_x64.pwdata[63:48]	<= adbus_in;

					//If 64 bit, more to come
					if(addr_is_x64)
						state					<= STATE_WDATA_LO2;

					//Dispatch the transaction now if it's a 32 bit
					else begin
						state					<= STATE_ACTIVE;

						apb_x32.paddr			<= pending_addr;
						apb_x32.pwrite			<= pending_write;
						apb_x32.psel			<= 1;
						apb_busy				<= 1;
					end

				end

				//Word 2 of write data (low half of second word for x64)
				STATE_WDATA_LO2: begin
					apb_x64.pstrb[1:0]		<= ~fmc_nbl;
					apb_x64.pwdata[15:0]	<= adbus_in;
					state					<= STATE_WDATA_HI2;
				end

				//Word 3 of write data (high half of second word for x64)
				STATE_WDATA_HI2: begin
					apb_x64.pstrb[3:2]		<= ~fmc_nbl;
					apb_x64.pwdata[31:16]	<= adbus_in;

					apb_x64.paddr			<= pending_addr;
					apb_x64.pwrite			<= pending_write;
					apb_x64.psel			<= 1;
					apb_busy				<= 1;

					state					<= STATE_ACTIVE;
				end

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Reads

				STATE_RDATA_LO: begin
					if(!apb_busy )
						state			<= STATE_RDATA_HI;
				end

				STATE_RDATA_HI: begin
					state				<= STATE_RD_END;
				end

				STATE_RD_END: begin
					//ignore any further activity until CS# goes high
					//(STM32H735 errata 2.6.1, two dummy clocks at end of burst)
				end

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Stop at end of a burst

				default: begin
					//nothing to do
				end

			endcase

		end

		//Deselected? return to idle state
		else
			state		<= STATE_ADDR;

		//Reset entire state machine and trigger a bus reset if we get stuck
		if(stuck_release) begin
			apb_busy		<= 0;
			apb_x32.penable	<= 0;
			apb_x32.psel	<= 0;
			state			<= STATE_ADDR;
		end

	end

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Performance counters (TODO make these APB readable)

	//CLOCK_PERIOD is in nanoseconds, e.g. 8 = 125 MHz, convert to PCLK cycles per second
	localparam PCLK_CYCLES_PER_SEC = 1000 * 1000 * 1000 / CLOCK_PERIOD;
	localparam SEC_COUNT_BITS = $clog2(PCLK_CYCLES_PER_SEC);
	integer SEC_COUNT_MAX = PCLK_CYCLES_PER_SEC - 1;
	logic[SEC_COUNT_BITS-1:0]	count_1hz = 0;
	logic 						tick_1hz = 0;
	always_ff @(posedge pclk) begin
		count_1hz	<= count_1hz + 1;
		tick_1hz	<= 0;
		if(count_1hz >= SEC_COUNT_MAX) begin
			tick_1hz	<= 1;
			count_1hz	<= 0;
		end
	end

	wire[47:0]	apb_reads_per_sec_raw;
	PerformanceCounter perf_count_apb_reads_per_sec(
		.clk(apb_x32.pclk),
		.en(apb_x32.pready && !apb_x32.pwrite),
		.delta(1),
		.rst(tick_1hz),
		.count(apb_reads_per_sec_raw));

	wire[47:0]	apb_writes_per_sec_raw;
	PerformanceCounter perf_count_apb_writes_per_sec(
		.clk(apb_x32.pclk),
		.en(apb_x32.pready && apb_x32.pwrite),
		.delta(1),
		.rst(tick_1hz),
		.count(apb_writes_per_sec_raw));

	wire[47:0]	apb_active_per_sec_raw;
	PerformanceCounter perf_count_apb_active_per_sec(
		.clk(apb_x32.pclk),
		.en(apb_x32.penable),
		.delta(1),
		.rst(tick_1hz),
		.count(apb_active_per_sec_raw));

	wire[47:0]	fmc_active_per_sec_raw;
	PerformanceCounter perf_count_fmc_active_per_sec(
		.clk(apb_x32.pclk),
		.en(!fmc_cs_n),
		.delta(1),
		.rst(tick_1hz),
		.count(fmc_active_per_sec_raw));

	wire[47:0]	fmc_ifg_per_sec_raw;
	PerformanceCounter perf_count_fmc_ifg_per_sec(
		.clk(apb_x32.pclk),
		.en(!fmc_nl_nadv),
		.delta(2),
		.rst(tick_1hz),
		.count(fmc_ifg_per_sec_raw));

	logic[47:0] apb_reads_per_sec = 0;
	logic[47:0] apb_writes_per_sec = 0;
	logic[47:0] apb_active_per_sec = 0;
	logic[47:0] fmc_active_per_sec = 0;
	logic[47:0] fmc_ifg_per_sec = 0;
	always_ff @(posedge pclk) begin
		if(tick_1hz) begin
			apb_reads_per_sec	<= apb_reads_per_sec_raw;
			apb_writes_per_sec	<= apb_writes_per_sec_raw;
			apb_active_per_sec	<= apb_active_per_sec_raw;
			fmc_active_per_sec	<= fmc_active_per_sec_raw;
			fmc_ifg_per_sec		<= fmc_ifg_per_sec_raw;
		end
	end
	*/

endmodule
