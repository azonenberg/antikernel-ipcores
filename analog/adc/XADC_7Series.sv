`default_nettype none
`timescale 1ns / 1ps
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2018 Andrew D. Zonenberg                                                                          *
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
	@brief	Controller for the Xilinx 7-series XADC
 */
module XADC_7Series(
	input wire			clk,				//nominal 125 MHz

	output logic		adc_valid	= 0,
	output logic[15:0]	adc_code	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XADC primitive

	logic[15:0]	xadc_din		= 0;
	wire[15:0]	xadc_dout;
	logic[6:0]	xadc_addr		= 0;
	logic		xadc_en			= 0;
	logic		xadc_we			= 0;
	wire		xadc_ready;
	wire[7:0]	xadc_alarm;
	wire		xadc_overheat;
	wire		xadc_busy;
	logic		xadc_reset		= 1;

	wire		xadc_eoc;

	XADC #(
		.INIT_40(16'h0003),		//CH = 3 (dedicated analog inputs), no averaging, continuous sampling mode
		.INIT_41(16'h3000),		//no sequencing
		.INIT_42(16'h0500),		//Not powered down, DCLK = clk / 5
		.INIT_43(16'h0000),		//factory test
		.INIT_44(16'h0000),		//factory test
		.INIT_45(16'h0000),		//factory test
		.INIT_46(16'h0000),		//factory test
		.INIT_47(16'h0000),		//factory test
		.INIT_48(16'h0000),		//no sequencing
		.INIT_49(16'h0000),		//no aux channels
		.INIT_4A(16'h0000),		//no averaging
		.INIT_4B(16'h0000),		//no aux channel averaging
		.INIT_4C(16'h0000),		//internal sensors unipolar
		.INIT_4D(16'h0000),		//external inputs unipolar (but not used)
		.INIT_4E(16'h0000),		//no additional settling time
		.INIT_4F(16'h0000),		//no additional settling time
		.INIT_50(16'hb5ed),		//temp alarm +85C
		.INIT_51(16'h5999),		//vccint alarm 1.05V
		.INIT_52(16'hA147),		//vccaux alarm 1.89V
		.INIT_53(16'hdddd),		//thermal shutdown at 125C
		.INIT_54(16'ha93a),		//reset temp alarm at 60C
		.INIT_55(16'h5111),		//vccint alarm 0.95V
		.INIT_56(16'h91eb),		//vccaux alarm 1.71V
		.INIT_57(16'hae4e),		//reset thermal shutdown at 70C
		.INIT_58(16'h5999),		//vccbram alarm 1.05V
		//59 - 5b reserved
		.INIT_5C(16'h5111)		//vccbram alarm 0.95V
	) xadc (
		.DI(xadc_din),
		.DO(xadc_dout),
		.DADDR(xadc_addr),
		.DEN(xadc_en),
		.DWE(xadc_we),
		.DCLK(clk),
		.DRDY(xadc_ready),
		.RESET(xadc_reset),
		.CONVST(1'b0),
		.CONVSTCLK(1'b0),
		.VP(1'b0),
		.VN(1'b0),
		.VAUXP(16'h0),
		.VAUXN(16'h0),
		.ALM(xadc_alarm),
		.OT(xadc_overheat),
		.MUXADDR(),				//no external mux
		.CHANNEL(),
		.EOC(xadc_eoc),
		.EOS(),
		.BUSY(xadc_busy),
		.JTAGLOCKED(),			//ignore jtag stuff, we assume its not in use
		.JTAGMODIFIED(),
		.JTAGBUSY()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Control logic

	reg[3:0] 	xadc_state = 0;
	reg[7:0] 	xadc_count = 0;
	logic		valid_toggle	= 0;
	always @(posedge clk) begin

		xadc_din		<= 0;
		xadc_en			<= 0;
		xadc_we			<= 0;

		adc_valid		<= 0;

		//Just poll the sensors in a tight loop
		//TODO: Sync to new-data alerts?
		case(xadc_state)

			//Power-up reset
			0: begin
				xadc_reset	<= 1;
				xadc_count	<= xadc_count + 8'h1;
				if(xadc_count == 8'hff)
					xadc_state	<= 1;
			end

			1: begin
				xadc_reset	<= 0;
				xadc_count	<= xadc_count + 8'h1;
				if(xadc_count == 8'hff)
					xadc_state	<= 2;
			end

			//Read ADC data when a conversion finishes
			2: begin
				if(xadc_eoc) begin
					xadc_en		<= 1;
					xadc_addr	<= 16'h0003;
					xadc_state	<= 3;
				end
			end

			3: begin
				if(xadc_ready) begin
					adc_valid		<= valid_toggle;
					valid_toggle	<= !valid_toggle;		//something to do with dual adc, only half of the samples have new data??
					adc_code		<= xadc_dout;
					xadc_state		<= 2;
				end
			end

		endcase

	end

endmodule
