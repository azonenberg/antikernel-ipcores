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

/**
	@brief Clock/data recovery block for 1.25 Gbps data (SGMII / 1000baseX) oversampled with a 7 series LVDS input

	Loosely based on XAPP523 but completely different data recovery state machine and simpler clocking architecture
 */
module OversamplingCDR #(
	parameter INVERT = 0
) (
	input wire			clk_625mhz_0,
	input wire			clk_625mhz_90,
	input wire			clk_312p5mhz,

	input wire			din_p,
	input wire			din_n,

	output logic[4:0]	rx_data,
	output logic[2:0]	rx_data_count
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Input buffer and delay lines

	wire	data_p;
	wire	data_n;

	IBUFDS_DIFF_OUT #(
		//.DIFF_TERM("TRUE"),
		.IBUF_LOW_PWR("FALSE"),
		//.IOSTANDARD("LVDS")
		.IOSTANDARD("DIFF_HSTL_I_18")
	) ibuf (
		.I(din_p),
		.IB(din_n),
		.O(data_p),
		.OB(data_n)
	);

	wire	data_p_delayed;
	wire	data_n_delayed;

	IODelayBlock #(
		.WIDTH(1),
		.CAL_FREQ(400),
		.INPUT_DELAY(00),
		.DIRECTION("IN"),
		.IS_CLOCK(0)
	) idelay_p (
		.i_pad(data_p),
		.i_fabric(data_p_delayed),

		.o_pad(),
		.o_fabric(),

		.input_en(1'b1)
	);

	IODelayBlock #(
		.WIDTH(1),
		.CAL_FREQ(400),
		.INPUT_DELAY(200),
		.DIRECTION("IN"),
		.IS_CLOCK(0)
	) idelay_n (
		.i_pad(data_n),
		.i_fabric(data_n_delayed),

		.o_pad(),
		.o_fabric(),

		.input_en(1'b1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initial oversampling: 4 phases of 625 MHz = 2.5 Gsps, times P and N = 5 Gsps

	wire[3:0] deser_p;
	wire[3:0] deser_n;

	logic	serdes_rst = 0;

	//Interleave with negative before positive
	ISERDESE2 #(
		.DATA_RATE("DDR"),
		.DATA_WIDTH("4"),
		.DYN_CLKDIV_INV_EN("FALSE"),
		.DYN_CLK_INV_EN("FALSE"),
		.INTERFACE_TYPE("OVERSAMPLE"),
		.NUM_CE(1),
		.OFB_USED("FALSE"),
		.SERDES_MODE("MASTER"),
		.IOBDELAY("BOTH")
	) iserdes_p (
		.Q1(deser_p[0]),
		.Q2(deser_p[2]),
		.Q3(deser_p[1]),
		.Q4(deser_p[3]),
		.Q5(),
		.Q6(),
		.Q7(),
		.Q8(),
		.O(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.D(),
		.DDLY(data_p_delayed),
		.CLK(clk_625mhz_0),
		.CLKB(!clk_625mhz_0),
		.CE1(1'b1),
		.CE2(1'b1),
		.RST(serdes_rst),
		.CLKDIV(),
		.CLKDIVP(1'b0),
		.OCLK(clk_625mhz_90),
		.OCLKB(!clk_625mhz_90),
		.BITSLIP(1'b0),
		.SHIFTIN1(),
		.SHIFTIN2(),
		.OFB(),
		.DYNCLKDIVSEL(),
		.DYNCLKSEL()
	);

	ISERDESE2 #(
		.DATA_RATE("DDR"),
		.DATA_WIDTH("4"),
		.DYN_CLKDIV_INV_EN("FALSE"),
		.DYN_CLK_INV_EN("FALSE"),
		.INTERFACE_TYPE("OVERSAMPLE"),
		.NUM_CE(1),
		.OFB_USED("FALSE"),
		.SERDES_MODE("MASTER"),
		.IOBDELAY("BOTH")
	) iserdes_n (
		.Q1(deser_n[0]),
		.Q2(deser_n[2]),
		.Q3(deser_n[1]),
		.Q4(deser_n[3]),
		.Q5(),
		.Q6(),
		.Q7(),
		.Q8(),
		.O(),
		.SHIFTOUT1(),
		.SHIFTOUT2(),
		.D(),
		.DDLY(data_n_delayed),
		.CLK(clk_625mhz_0),
		.CLKB(!clk_625mhz_0),
		.CE1(1'b1),
		.CE2(1'b1),
		.RST(serdes_rst),
		.CLKDIV(),
		.CLKDIVP(1'b0),
		.OCLK(clk_625mhz_90),
		.OCLKB(!clk_625mhz_90),
		.BITSLIP(1'b0),
		.SHIFTIN1(),
		.SHIFTIN2(),
		.OFB(),
		.DYNCLKDIVSEL(),
		.DYNCLKSEL()
	);

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register ISERDES output and merge the P/N outputs into a single 8 bit data stream

	logic[3:0] deser_p_ff = 0;
	logic[3:0] deser_n_ff = 0;

	always_ff @(posedge clk_625mhz_0) begin
		deser_p_ff <= deser_p;
		deser_n_ff <= deser_n;
	end

	wire[7:0] deser_merged;
	assign deser_merged =
	{
		deser_p_ff[3],
		~deser_n_ff[3],
		deser_p_ff[2],
		~deser_n_ff[2],
		deser_p_ff[1],
		~deser_n_ff[1],
		deser_p_ff[0],
		~deser_n_ff[0]
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Width extension from 1x to 2x at half rate

	logic[7:0] samples_hi = 0;
	logic[7:0] samples_lo = 0;

	always_ff @(posedge clk_625mhz_0) begin
		samples_hi	<= samples_lo;
		samples_lo	<= deser_merged;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register half-width signals on alternate clock edges

	logic	toggle_ff = 0;
	logic	toggle_ff2 = 0;

	logic[7:0] samples_hi_2x = 0;
	logic[7:0] samples_lo_2x = 0;

	logic toggle = 0;

	always_ff @(posedge clk_625mhz_0) begin
		toggle <= !toggle;

		if(toggle) begin
			samples_hi_2x <= samples_hi;
			samples_lo_2x <= samples_lo;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Shift wide data to 312.5 MHz clock domain and combine to a single 16-bit data stream

	logic[7:0] samples_hi_sync = 0;
	logic[7:0] samples_lo_sync = 0;

	always_ff @(posedge clk_312p5mhz) begin
		samples_hi_sync <= samples_hi_2x;
		samples_lo_sync <= samples_lo_2x;
	end

	//Swap from LSB first (serdes style) to MSB first (more readable) bit ordering
	wire[15:0] s0_samples;
	assign s0_samples =
	{
		samples_hi_sync[0] ^ INVERT[0],
		samples_hi_sync[1] ^ INVERT[0],
		samples_hi_sync[2] ^ INVERT[0],
		samples_hi_sync[3] ^ INVERT[0],
		samples_hi_sync[4] ^ INVERT[0],
		samples_hi_sync[5] ^ INVERT[0],
		samples_hi_sync[6] ^ INVERT[0],
		samples_hi_sync[7] ^ INVERT[0],

		samples_lo_sync[0] ^ INVERT[0],
		samples_lo_sync[1] ^ INVERT[0],
		samples_lo_sync[2] ^ INVERT[0],
		samples_lo_sync[3] ^ INVERT[0],
		samples_lo_sync[4] ^ INVERT[0],
		samples_lo_sync[5] ^ INVERT[0],
		samples_lo_sync[6] ^ INVERT[0],
		samples_lo_sync[7] ^ INVERT[0]
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 1: Find edges, then delay the data to match the edge signals

	logic[16:0]	s1_edges		= 0;
	logic[15:0] s1_samples		= 0;

	always_ff @(posedge clk_312p5mhz) begin
		s1_samples		<= s0_samples;
		s1_edges[16]	<= s1_samples[0] ^ s1_samples[1];
		s1_edges[15] 	<= s0_samples[15] ^ s1_samples[0];
		s1_edges[14:0]	<= s0_samples[14:0] ^ s0_samples[15:1];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PLL for data recovery

	logic		done = 0;
	logic[3:0]	edgepos = 15;
	logic		sample_first_bit = 0;

	always_ff @(posedge clk_312p5mhz) begin

		//Sample first bit if we're wrapping around
		if(sample_first_bit) begin
			rx_data			= {4'b0, s1_samples[15]};
			rx_data_count 	= 1;
		end

		else begin
			rx_data			= 0;
			rx_data_count	= 0;
		end

		sample_first_bit	= 0;
		done				= 0;

		//Can have up to four bits of output data, plus one more if we wrapped around and sampled at i=0
		for(integer i=0; i<5; i=i+1) begin

			//If we've run out of bits, stop
			if(done) begin
			end

			else begin

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Sample the input on the NCO clock

				//Sample one cycle after the NCO edge (if not past the end of the block)
				if(edgepos > 0) begin
					rx_data				= {rx_data[3:0], s1_samples[edgepos-1]};
					rx_data_count		= rx_data_count + 1;
				end

				//Edge in the last cycle, so sample the first of the next block
				else
					sample_first_bit	= 1;

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Edge detection and phase accumulator updates

				//If edge before nominal point, bump phase backward
				if(s1_edges[edgepos+1]) begin
					done	= (edgepos < 3);
					edgepos = edgepos - 3;
				end

				//If edge after nominal point, bump phase forward
				else if((edgepos > 0) && s1_edges[edgepos-1]) begin
					done	= (edgepos < 5);
					edgepos = edgepos - 5;
				end

				//No change to phase, we're locked (no edge, or edge exactly where we want it)
				else begin
					done	= (edgepos < 4);
					edgepos = edgepos - 4;
				end

			end

		end

	end

endmodule
