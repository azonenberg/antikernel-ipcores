`default_nettype none
`timescale 1ns/1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg                                                                          *
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

/*
	@brief Phase control to align BUFG and BUFIO clocks
 */
module OversamplingPhaseAlignment(
	input wire				clk_312p5mhz,
	input wire				clk_625mhz_fabric,
	input wire				clk_625mhz_io_0,

	output logic			phase_shift_en,
	output logic			phase_shift_inc,
	input wire				phase_shift_done,

	output logic			done	= 0
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loopback path to measure phase shift

	logic	serdes_rst	= 0;

	//Drive an output clocked by the OSERDESE2 with a constant, known data pattern
	wire oserdes_out;
	OSERDESE2 #(
		.DATA_RATE_OQ("DDR"),
		.DATA_RATE_TQ("BUF"),
		.DATA_WIDTH("4"),
		.SERDES_MODE("MASTER"),
		.TRISTATE_WIDTH(1),
		.TBYTE_CTL("FALSE"),
		.TBYTE_SRC("FALSE")
	) oserdes (
		.OQ(),
		.OFB(oserdes_out),
		.TQ(),
		.TFB(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.CLK(clk_625mhz_fabric),
		.CLKDIV(clk_312p5mhz),
		.D1(1'b1),
		.D2(1'b0),
		.D3(1'b0),
		.D4(1'b0),
		.D5(1'b0),
		.D6(1'b0),
		.D7(1'b0),
		.D8(1'b0),
		.TCE(1'b1),
		.OCE(1'b1),
		.TBYTEIN(1'b1),
		.TBYTEOUT(),
		.RST(serdes_rst),
		.SHIFTIN1(),
		.SHIFTIN2(),
		.T1(1'b0),
		.T2(1'b0),
		.T3(1'b0),
		.T4(1'b0)
	);

	//Sample it with an ISERDESE2 using the BUFIOs
	wire[3:0] deser;
	ISERDESE2 #(
		.DATA_RATE("DDR"),
		.DATA_WIDTH("4"),
		.DYN_CLKDIV_INV_EN("FALSE"),
		.DYN_CLK_INV_EN("FALSE"),
		.INTERFACE_TYPE("NETWORKING"),
		.NUM_CE(1),
		.OFB_USED("TRUE"),
		.SERDES_MODE("MASTER"),
		.IOBDELAY("BOTH")
	) iserdes (
		.Q1(deser[0]),
		.Q2(deser[1]),
		.Q3(deser[2]),
		.Q4(deser[3]),
		.Q5(),
		.Q6(),
		.Q7(),
		.Q8(),
		.O(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.D(),
		.DDLY(),
		.CLK(clk_625mhz_io_0),
		.CLKB(!clk_625mhz_io_0),
		.CE1(1'b1),
		.CE2(1'b1),
		.RST(serdes_rst),
		.CLKDIV(clk_312p5mhz),
		.CLKDIVP(1'b0),
		.OCLK(1'b0),
		.OCLKB(1'b0),
		.BITSLIP(1'b0),
		.SHIFTIN1(),
		.SHIFTIN2(),
		.OFB(oserdes_out),
		.DYNCLKDIVSEL(),
		.DYNCLKSEL()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Phase control state machine

	enum logic[2:0]
	{
		STATE_BOOT_0 		= 'h0,
		STATE_BOOT_1		= 'h1,
		STATE_MEASURE_FIRST	= 'h2,
		STATE_MEASURE		= 'h3,
		STATE_MEASURE_DONE	= 'h4,
		STATE_SHIFT_WAIT	= 'h5,
		STATE_ALIGNED		= 'h6
	} state = STATE_BOOT_0;

	typedef enum logic[2:0]
	{
		MEASUREMENT_0001,
		MEASUREMENT_0010,
		MEASUREMENT_0100,
		MEASUREMENT_1000,
		MEASUREMENT_UNSTABLE
	} measurement_t;

	measurement_t	cur;
	measurement_t	cur_ff;
	logic[9:0]		bumps_since_last_unstable	= 0;
	logic			found_unstable				= 0;
	logic[8:0]		bumps_left					= 0;

	//Figure out current bit position
	always_comb begin
		case(deser)
			4'b0001:	cur	= MEASUREMENT_0001;
			4'b0010:	cur	= MEASUREMENT_0010;
			4'b0100:	cur	= MEASUREMENT_0100;
			4'b1000:	cur	= MEASUREMENT_1000;
			default:	cur	= MEASUREMENT_UNSTABLE;
		endcase
	end

	logic[4:0] count = 0;

	always_ff @(posedge clk_312p5mhz) begin

		phase_shift_en	<= 0;

		case(state)

			//Wait for reset to assert
			STATE_BOOT_0: begin
			end	//end STATE_BOOT_0

			//Wait for reset to clear
			STATE_BOOT_1: begin
				done							<= 0;

				if(!serdes_rst) begin
					state						<= STATE_MEASURE_FIRST;
					bumps_since_last_unstable	<= 0;
					found_unstable				<= 0;
				end
			end	//end STATE_BOOT_1

			//Measure several consecutive values and see what we get
			//Ideally we want to see a single stable value
			STATE_MEASURE_FIRST: begin
				cur_ff		<= cur;
				count		<= 1;
				state		<= STATE_MEASURE;
			end	//end STATE_MEASURE_FIRST

			STATE_MEASURE: begin
				count		<= count + 1;
				if(cur != cur_ff)
					cur_ff	<= MEASUREMENT_UNSTABLE;

				//Last cycle
				if(count == 0)
					state	<= STATE_MEASURE_DONE;

			end	//end STATE_MEASURE

			//Measurement is finished.
			STATE_MEASURE_DONE: begin

				//We're shifting either way
				phase_shift_en	<= 1;
				state			<= STATE_SHIFT_WAIT;

				//We are CURRENTLY UNSTABLE.
				if(cur == MEASUREMENT_UNSTABLE) begin

					//Is this the first block of instability? Move ahead and look for the next
					if(!found_unstable) begin
						found_unstable				<= 1;
						bumps_since_last_unstable	<= 1;
						phase_shift_inc				<= 1;
					end

					//Second block of instability.
					//Jump back to the midpoint
					else begin
						bumps_left					<= bumps_since_last_unstable[9:1] - 1;
						phase_shift_inc				<= 0;
					end

				end

				//Currently stable. Keep searching.
				else begin
					phase_shift_inc				<= 1;
					bumps_since_last_unstable	<= bumps_since_last_unstable + 1;
				end

			end //end STATE_MEASURE_DONE

			//Wait for phase shift to complete
			STATE_SHIFT_WAIT: begin

				if(phase_shift_done) begin

					//Still searching? Prepare to take another measurement
					if(phase_shift_inc)
						state	<= STATE_MEASURE_FIRST;

					//Still moving back to the optimal phase alignment? Shift again
					else if(bumps_left != 0) begin
						phase_shift_en	<= 1;
						bumps_left		<= bumps_left - 1;
					end

					//All done
					else begin
						state	<= STATE_ALIGNED;
						done	<= 1;
					end

				end

			end //end STATE_SHIFT_WAIT

			//Continually monitor and re-align if we ever see an error
			//Otherwise wait for external logic to trigger a reset
			STATE_ALIGNED: begin

				if(cur == MEASUREMENT_UNSTABLE) begin
					state			<= STATE_MEASURE_FIRST;
					found_unstable	<= 0;
					phase_shift_inc	<= 1;
				end

			end

		endcase

		if(serdes_rst)
			state	<= STATE_BOOT_1;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Boot time reset logic

	logic[7:0] boot_count = 1;
	always_ff @(posedge clk_312p5mhz) begin

		if(boot_count) begin
			boot_count <= boot_count + 1;
			if(boot_count == 127)
				serdes_rst <= 1;
		end

		else
			serdes_rst	<= 0;
	end

endmodule
