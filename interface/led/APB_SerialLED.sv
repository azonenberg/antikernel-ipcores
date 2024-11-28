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
	@brief A serial RGB LED controller, readable/writable in blocks over APB
 */
module APB_SerialLED #(
	parameter NUM_LEDS 		= 2,

	//Length of short, long, inter-frame gap, and reset pulses, in PCLK cycles. Varies by LED and PCLK frequency
	//Default is for Everlight 19/C47 LEDs with 100 MHz PCLK
	parameter SHORT_TIME 	= 30,
	parameter LONG_TIME		= 90,
	parameter IFG_TIME		= 200,
	parameter RESET_TIME	= 750
)(
	//The APB bus
	APB.completer 			apb,

	//The GPIO signals
	output logic			led_ctrl = 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support up 32-bit APB due to register alignment, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//Combinatorial readback: not allowed, deny all read requests
			if(!apb.pwrite) begin
				apb.pslverr	= 1;
			end

			//Writes: allow all valid addresses
			else begin
				if( (apb.paddr[1:0]) || (apb.paddr[apb.ADDR_WIDTH-1 : 2] >= NUM_LEDS) ) begin
					apb.pslverr	 = 1;
				end
			end

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Framebuffer

	logic[23:0]	framebuffer[NUM_LEDS-1:0];

	//LEDs start out off in the idle state
	initial begin
		for(integer i=0; i<NUM_LEDS; i++)
			framebuffer[i] = 24'h000000;
	end

	logic				fb_read	= 0;
	logic[IDX_BITS-1:0]	nled = 0;
	logic[23:0]			pixcolor = 24'h000000;
	logic				update_req	= 0;

	always_ff @(posedge apb.pclk) begin

		if(!apb.preset_n) begin
			update_req	<= 0;
		end

		else begin

			update_req	<= 0;

			if(apb.pwrite && apb.pready) begin
				framebuffer[apb.paddr[apb.ADDR_WIDTH-1 : 2]]	<= apb.pwdata;
				update_req	<= 1;
			end

			//Reads
			if(fb_read)
				pixcolor	<= framebuffer[nled];

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// LED control logic

	localparam IDX_BITS = $clog2(NUM_LEDS);

	logic				update_pending	= 1;
	logic[9:0] 			clkdiv	= 0;
	logic[9:0] 			target	= 0;
	logic[4:0]			nbit	= 0;

	enum logic[2:0]
	{
		STATE_IDLE,
		STATE_RESET,
		STATE_HIGH,
		STATE_LOW,
		STATE_IFG
	} state = STATE_IDLE;

	wire[15:0] clkdiv_next;
	assign clkdiv_next = clkdiv + 1;

	always_ff @(posedge apb.pclk) begin

		if(!apb.preset_n) begin
			update_pending	<= 1;
			clkdiv		<= 0;
			target		<= 0;
			nbit		<= 0;
			nled		<= 0;
			fb_read		<= 0;
		end

		else begin

			//Clear single cycle flags
			fb_read	<= 0;

			//Mark update as pending
			if(update_req)
				update_pending	<= 1;

			case(state)

				STATE_IDLE: begin

					//Wait for a reset request
					if(update_pending) begin
						led_ctrl	<= 0;
						clkdiv		<= 0;
						target		<= RESET_TIME;
						state		<= STATE_RESET;
						update_pending	<= 0;
					end

				end

				//Hold data line low for full reset duration
				STATE_RESET: begin
					led_ctrl	<= 0;
					clkdiv		<= clkdiv_next;

					if(clkdiv_next >= target) begin

						state		<= STATE_IFG;
						nled		<= 0;

						clkdiv		<= 0;
						target		<= IFG_TIME;
						led_ctrl	<= 0;

						//Load the first LED color from the framebuffer
						fb_read		<= 1;

					end
				end

				STATE_IFG: begin

					clkdiv		<= clkdiv_next;

					//We're now ready to send the first data bit
					if(clkdiv_next >= target) begin

						//Select pulse length
						led_ctrl	<= 1;
						if(pixcolor[23])
							target	<= LONG_TIME;
						else
							target	<= SHORT_TIME;

						nbit		<= 23;
						clkdiv		<= 0;
						state		<= STATE_HIGH;

					end

				end

				STATE_HIGH: begin

					clkdiv		<= clkdiv_next;

					//Done with high pulse, bring data line low
					if(clkdiv_next >= target) begin

						//Select pulse length
						led_ctrl	<= 0;
						if(pixcolor[nbit])
							target	<= SHORT_TIME;
						else
							target	<= LONG_TIME;

						clkdiv		<= 0;
						state		<= STATE_LOW;

					end

				end

				STATE_LOW: begin

					clkdiv		<= clkdiv_next;

					//Done with low pulse, move on to next data word
					if(clkdiv_next >= target) begin

						//More bits to send?
						if(nbit != 0) begin
							nbit	<= nbit - 1;

							//Send the next data bit
							led_ctrl	<= 1;
							if(pixcolor[nbit - 1])
								target	<= LONG_TIME;
							else
								target	<= SHORT_TIME;

							clkdiv	<= 0;
							state	<= STATE_HIGH;

						end

						//No, this was the last bit. Move on to the next LED (if we have one)
						else begin

							//Last LED?
							if( (nled + 1) >= NUM_LEDS)
								state	<= STATE_IDLE;

							//Nope, we have more
							else begin

								//Now in inter-frame gap
								state		<= STATE_IFG;
								clkdiv		<= 0;
								target		<= IFG_TIME;

								//Load the next LED color from the framebuffer
								nled		<= nled + 1;
								fb_read		<= 1;

							end

						end

					end

				end

			endcase
		end

	end

endmodule
