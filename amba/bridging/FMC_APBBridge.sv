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
	@brief A bridge from the STM32 FMC to APBv5

	The FMC clock is used as the APB PCLK and is expected to be free-running.
 */
module FMC_APBBridge #(
	parameter EARLY_READ 	= 1,	//do not change unless altering latency on FMC IP
	parameter CLOCK_PHASE	= 90,	//phase shift for internal PLL to optimize read capture timing

	parameter CLOCK_PERIOD	= 8,	//FMC clock period in ns (default 250 MHz)
	parameter VCO_MULT		= 8		//PLL multiplier (default 4 for 1 GHz with 250 MHz PCLK)
)(

	//APB root bus to interconnect bridge
	APB.requester		apb,

	//FMC pins to MCU
	input wire			fmc_clk,
	(* iob = "true" *) output logic		fmc_nwait,
	input wire			fmc_noe,
	inout wire[15:0]	fmc_ad,
	input wire			fmc_nwe,
	input wire[1:0]		fmc_nbl,
	input wire			fmc_nl_nadv,
	input wire[2:0]		fmc_a_hi,
	input wire			fmc_cs_n			//bank chip select
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB due to the native bus width. Throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O buffers

	wire[15:0]	adbus_in;
	(* iob = "true" *) logic[15:0]	adbus_out = 0;

	BidirectionalBuffer #(
		.WIDTH(16),
		.OE_INVERT(0)
	) ad_iobuf (
		.fabric_in(adbus_in),
		.fabric_out(adbus_out),
		.pad(fmc_ad),
		.oe(fmc_noe)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PLL for deskewing clock buffer delay

	//TODO: portable cross generation PLL for UltraScale etc

	wire	clk_fb;
	wire	clk_fb_bufg;
	wire	pclk_raw;
	wire	pll_lock;

	PLLE2_BASE #(
		.BANDWIDTH("OPTIMIZED"),
		.CLKFBOUT_MULT(VCO_MULT),
		.DIVCLK_DIVIDE(1),
		.CLKFBOUT_PHASE(0),
		.CLKIN1_PERIOD(CLOCK_PERIOD),
		.STARTUP_WAIT("FALSE"),

		.CLKOUT0_DIVIDE(VCO_MULT),	//Fout = Fin
		.CLKOUT1_DIVIDE(8),
		.CLKOUT2_DIVIDE(8),
		.CLKOUT3_DIVIDE(8),
		.CLKOUT4_DIVIDE(8),
		.CLKOUT5_DIVIDE(8),

		.CLKOUT0_PHASE(CLOCK_PHASE),	//phase shift on clock to center data eyes better
		.CLKOUT1_PHASE(0),
		.CLKOUT2_PHASE(0),
		.CLKOUT3_PHASE(0),
		.CLKOUT4_PHASE(0),
		.CLKOUT5_PHASE(0),

		.CLKOUT0_DUTY_CYCLE(0.5),
		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5)
	) fmc_pll (
		.CLKIN1(fmc_clk),
		.CLKFBIN(clk_fb_bufg),
		.CLKFBOUT(clk_fb),
		.RST(1'b0),
		.CLKOUT0(pclk_raw),
		.CLKOUT1(),
		.CLKOUT2(),
		.CLKOUT3(),
		.CLKOUT4(),
		.CLKOUT5(),
		.LOCKED(pll_lock),
		.PWRDWN(0)
	);

	BUFG bufg_clk_fb(
		.I(clk_fb),
		.O(clk_fb_bufg));

	BUFGCE bufg_pclk(
		.I(pclk_raw),
		.O(apb.pclk),
		.CE(pll_lock));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Hold APB in reset until PLL locks, then keep asserted for a few clocks after

	logic	rst_n	= 0;
	assign	apb.preset_n	= rst_n;
	logic[3:0]	rst_count	= 1;

	always_ff @(posedge apb.pclk or negedge pll_lock) begin

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
		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	//Tie off unused signals
	assign apb.pprot 	= 0;
	assign apb.pwakeup 	= 0;
	assign apb.pauser	= 0;
	assign apb.pwuser	= 0;

	enum logic[3:0]
	{
		STATE_IDLE,
		STATE_ADDR,
		STATE_WAIT,
		STATE_WDATA_LO,
		STATE_WDATA_HI,
		STATE_WDATA_WAIT,
		STATE_RDATA_LO,
		STATE_RDATA_HI,
		STATE_RD_END,
		STATE_ACTIVE,

		STATE_LAST	//unused
	} state = STATE_IDLE;

	//Saved address/write enable flag before we started a write
	logic[18:0]		pending_addr	= 0;
	logic			pending_write	= 0;
	logic			apb_busy		= 0;
	logic[3:0]		wait_count		= 0;

	//Saved read data
	logic[31:0]		prdata_latched	= 0;

	//number of cycles it takes for a wait cycle to end before the new data gets here
	localparam WAIT_CYCLE_RTT		= 2;

	always_comb begin

		//Not waiting if no transaction active
		if(fmc_cs_n)
			fmc_nwait	= 1;

		//Waiting if blocking on previous transaction
		else if( (state == STATE_WAIT) && apb_busy)
			fmc_nwait	= 0;

		else
			fmc_nwait	= !apb_busy;

	end

	always_ff @(posedge apb.pclk) begin

		//Activate
		if(apb.penable && !apb.psel)
			apb.psel	<= 1;

		//Complete a transaction
		if(apb.pready) begin
			apb.penable		<= 0;
			apb.psel		<= 0;
			apb_busy		<= 0;
			prdata_latched	<= apb.prdata;
		end

		//Selected! Let's do something
		if(!fmc_cs_n) begin

			case(state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Common path

				STATE_IDLE: begin
					state			<= STATE_ADDR;
				end

				//Save address and write enable flag
				//LSB of address is always implicitly zero because of 16 bit bus width
				STATE_ADDR: begin
					pending_addr	<= { fmc_a_hi, adbus_in, 1'b0};
					pending_write	<= !fmc_nwe;
					state			<= STATE_WAIT;

					//Dispatch reads as soon as we can
					if(!apb_busy && fmc_nwe) begin
						apb.paddr		<= { fmc_a_hi, adbus_in, 1'b0};;
						apb.pwrite		<= 0;
						apb.penable		<= 1;
						apb_busy		<= 1;

						//Waiting for read data to come back
						state			<= STATE_RDATA_LO;
					end

				end

				//Wait state
				STATE_WAIT: begin

					//it's a write
					if(pending_write)
						state			<= STATE_WDATA_LO;

					//it's a read, dispatch it
					else begin
						if(apb_busy) begin
							//wait until the previous transaction finishes
						end

						else begin

							//Kick off the read request
							apb.paddr			<= pending_addr;
							apb.pwrite			<= 0;
							apb.penable			<= 1;
							apb_busy			<= 1;

							//Waiting for read data to come back
							state				<= STATE_RDATA_LO;
						end
					end

				end

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Writes

				//Bottom half of write data
				STATE_WDATA_LO: begin
					apb.pstrb[1:0]		<= ~fmc_nbl;
					apb.pwdata[15:0]	<= adbus_in;

					if(!fmc_nwait) begin
						state			<= STATE_WDATA_WAIT;
						wait_count		<= 0;
					end
					else
						state			<= STATE_WDATA_HI;

				end

				//wait state is active, delay for several clocks
				STATE_WDATA_WAIT: begin
					if(fmc_nwait) begin
						wait_count		<= wait_count + 1;
						if(wait_count >= WAIT_CYCLE_RTT)
							state		<= STATE_WDATA_HI;
					end
				end

				//High half of write data
				STATE_WDATA_HI: begin
					apb.pstrb[3:2]		<= ~fmc_nbl;
					apb.pwdata[31:16]	<= adbus_in;
					state				<= STATE_ACTIVE;

					//Apply the pending changes
					apb.paddr			<= pending_addr;
					apb.pwrite			<= pending_write;
					apb.penable			<= 1;
					apb_busy			<= 1;
				end

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Reads

				STATE_RDATA_LO: begin

					if(apb.pready && EARLY_READ) begin
						adbus_out		<= apb.prdata[15:0];
						state			<= STATE_RDATA_HI;
					end

					if(!apb_busy) begin
						adbus_out		<= prdata_latched[15:0];
						state			<= STATE_RDATA_HI;
					end
				end

				STATE_RDATA_HI: begin
					adbus_out			<= prdata_latched[31:16];
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
		else begin
			state	<= STATE_IDLE;
		end

	end

endmodule
