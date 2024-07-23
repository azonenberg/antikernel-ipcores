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
	parameter EARLY_READ_DISPATCH	= 0,	//set true to dispatch reads one clock earlier (less latency, worse timing)
	parameter CAPTURE_CLOCK_PHASE	= 0,	//phase shift for internal PLL to optimize data capture timing
	parameter LAUNCH_CLOCK_PHASE	= 0,	//phase shift for internal PLL to optimize data launch timing

	parameter CLOCK_PERIOD			= 8,	//FMC clock period in ns (default 125 MHz)
	parameter VCO_MULT				= 8,	//PLL multiplier (default 8 for 1 GHz with 125 MHz PCLK)

	//number of cycles it takes for a wait cycle on write to end before the new data gets here
	localparam WAIT_CYCLE_RTT		= 1
)(

	//Slow management clock
	input wire			clk_mgmt,

	//APB root bus to interconnect bridge
	APB.requester		apb,

	//FMC pins to MCU
	input wire			fmc_clk,
	(* iob="true" *)  output logic		fmc_nwait = 1,
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
	// VIO for adjusting output clock phase
	// TODO: make this APB programmable??

	wire		phase_en;
	wire[2:0]	phase_mux;
	wire[5:0]	phase_delay;

	vio_0 vio(
		.clk(clk_mgmt),
		.probe_out0(phase_en),
		.probe_out1(phase_mux),
		.probe_out2(phase_delay)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DRP controls

	logic		drp_en	= 0;
	logic		drp_we	= 0;
	logic[15:0]	drp_di	= 0;
	wire[15:0]	drp_do;
	wire		drp_ready;
	logic[6:0]	drp_addr;

	enum logic[3:0]
	{
		DRP_STATE_IDLE		= 'h0,
		DRP_STATE_READ1		= 'h1,
		DRP_STATE_WRITE1	= 'h2,
		DRP_STATE_READ2		= 'h3,
		DRP_STATE_WRITE2	= 'h4,

		DRP_STATE_DONE		= 'hf
	} drp_state = DRP_STATE_IDLE;

	always_ff @(posedge clk_mgmt) begin

		drp_en	<= 0;

		case(drp_state)

			DRP_STATE_IDLE: begin

				if(phase_en) begin
					drp_state	<= DRP_STATE_READ1;
					drp_we		<= 0;
					drp_en		<= 1;
					drp_addr	<= 7'h08;	//CLKOUT0 register 1
				end

			end

			DRP_STATE_READ1: begin
				if(drp_ready) begin
					drp_state	<= DRP_STATE_WRITE1;
					drp_we		<= 1;
					drp_en		<= 1;
					drp_di		<= { phase_mux, drp_do[12:0] };
				end
			end

			DRP_STATE_WRITE1: begin
				if(drp_ready) begin
					drp_state	<= DRP_STATE_READ2;
					drp_we		<= 0;
					drp_en		<= 1;
					drp_addr	<= 7'h09;	//CLKOUT0 register 2
				end
			end

			DRP_STATE_READ2: begin
				if(drp_ready) begin
					drp_state	<= DRP_STATE_WRITE2;
					drp_we		<= 1;
					drp_en		<= 1;
					drp_di		<= { drp_do[15:6], phase_delay };
				end
			end

			DRP_STATE_WRITE2: begin
				drp_state	<= DRP_STATE_DONE;
			end

			DRP_STATE_DONE: begin
				if(!phase_en) begin
					drp_state	<= DRP_STATE_IDLE;
				end
			end

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PLL for deskewing clock buffer delay

	//TODO: portable cross generation PLL for UltraScale etc

	wire	clk_launch;
	wire	pll_lock;

	wire	clk_fb;
	wire	clk_fb_bufg;
	wire	pclk_raw;
	wire	launch_clk_raw;

	PLLE2_ADV #(
		.BANDWIDTH("OPTIMIZED"),
		.CLKFBOUT_MULT(VCO_MULT),
		.DIVCLK_DIVIDE(1),
		.CLKFBOUT_PHASE(0),
		.CLKIN1_PERIOD(CLOCK_PERIOD),
		.STARTUP_WAIT("FALSE"),

		.CLKOUT0_DIVIDE(VCO_MULT),	//Fout = Fin
		.CLKOUT1_DIVIDE(VCO_MULT),	//same speed as PCLK but phase shifted
		.CLKOUT2_DIVIDE(8),
		.CLKOUT3_DIVIDE(8),
		.CLKOUT4_DIVIDE(8),
		.CLKOUT5_DIVIDE(8),

		.CLKOUT0_PHASE(CAPTURE_CLOCK_PHASE),	//phase shift on clock to center data eyes better
		.CLKOUT1_PHASE(LAUNCH_CLOCK_PHASE),
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
		.CLKIN2(1'b0),
		.CLKINSEL(1'b1),	//select CLKIN1

		//.CLKFBIN(clk_fb_bufg),
		.CLKFBIN(clk_fb),	//do not compensate for bufg insertion delay
		.CLKFBOUT(clk_fb),

		.RST(1'b0),
		.PWRDWN(0),
		.LOCKED(pll_lock),

		.CLKOUT0(pclk_raw),
		.CLKOUT1(launch_clk_raw),
		.CLKOUT2(),
		.CLKOUT3(),
		.CLKOUT4(),
		.CLKOUT5(),

		//DRP
		.DI(drp_di),
		.DADDR(drp_addr),
		.DWE(drp_we),
		.DEN(drp_en),
		.DCLK(clk_mgmt),
		.DO(drp_do),
		.DRDY(drp_ready)
	);
	/*
	BUFG bufg_clk_fb(
		.I(clk_fb),
		.O(clk_fb_bufg));*/

	BUFGCE bufg_pclk(
		.I(pclk_raw),
		.O(apb.pclk),
		.CE(pll_lock));

	BUFGCE bufg_launch_clk(
		.I(launch_clk_raw),
		.O(clk_launch),
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
		STATE_ADDR,
		STATE_WAIT,
		STATE_WAIT2,
		STATE_WDATA_LO,
		STATE_WDATA_HI,
		STATE_WDATA_WAIT,
		STATE_RDATA_LO,
		STATE_RDATA_HI,
		STATE_RD_END,
		STATE_ACTIVE,

		STATE_LAST	//unused
	} state = STATE_ADDR;

	//Saved address/write enable flag before we started a write
	logic[18:0]		pending_addr	= 0;
	logic			pending_write	= 0;
	logic			apb_busy		= 0;
	logic[3:0]		wait_count		= 0;

	//Saved read data
	logic[31:0]		prdata_latched	= 0;

	//send NWAIT on launch clock
	always_ff @(posedge clk_launch or negedge pll_lock) begin

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

	//send read data on launch clock
	always_ff @(posedge clk_launch) begin

		case(state)
			STATE_RDATA_LO:	begin
				/*if(apb.pready)
					adbus_out	<= apb.prdata[15:0];
				else*/
					adbus_out	<= prdata_latched[15:0];
			end
			STATE_RDATA_HI:	adbus_out	<= prdata_latched[31:16];

			//DEBUG
			STATE_RD_END:	adbus_out	<= 16'h4141;

			default:		adbus_out	<= 0;
		endcase

	end

	logic		apb_busy_ff	= 0;
	always_ff @(posedge apb.pclk) begin

		apb_busy_ff	<= apb_busy;

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

				//Save address and write enable flag
				//LSB of address is always implicitly zero because of 16 bit bus width
				STATE_ADDR: begin

					//Update as much of the buses as we can outside of nl_nadv to improve timing
					//(this may be worse for power but its not a lot of signals and they toggle slow)
					pending_addr	<= { fmc_a_hi, adbus_in, 1'b0};
					pending_write	<= !fmc_nwe;
					apb.pwrite		<= !fmc_nwe;
					apb.paddr		<= { fmc_a_hi, adbus_in, 1'b0};

					//DEBUG: clear prdata_latched
					prdata_latched	<= 16'hcccc;

					//Move on as soon as we get the address latched
					if(!fmc_nl_nadv) begin
						state			<= STATE_WAIT;

						//Dispatch reads as soon as we can
						if(!apb_busy && fmc_nwe && EARLY_READ_DISPATCH) begin
							apb.penable		<= 1;
							apb_busy		<= 1;

							//Waiting for read data to come back
							state			<= STATE_RDATA_LO;
						end
					end

				end

				//Wait state
				STATE_WAIT: begin

					//it's a write
					if(pending_write)
						state			<= STATE_WAIT2;

					//it's a read, dispatch it when available
					else if(!apb_busy) begin

						//Kick off the read request
						apb.paddr			<= pending_addr;
						apb.penable			<= 1;
						apb_busy			<= 1;

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

				//Bottom half of write data
				STATE_WDATA_LO: begin
					apb.pstrb[1:0]		<= ~fmc_nbl;
					apb.pwdata[15:0]	<= adbus_in;

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
					if(!apb_busy /* || apb.pready */)
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

	end

endmodule
