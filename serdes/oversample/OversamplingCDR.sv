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
 */
module OversamplingCDR #(
	parameter INVERT = 0
) (
	input wire			clk_625mhz_io_0,
	input wire			clk_625mhz_io_90,
	input wire			clk_625mhz_fabric,
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
		.DIFF_TERM("TRUE"),
		.IBUF_LOW_PWR("FALSE"),
		.IOSTANDARD("LVDS")
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
		.CLK(clk_625mhz_io_0),
		.CLKB(!clk_625mhz_io_0),
		.CE1(1'b1),
		.CE2(1'b1),
		.RST(serdes_rst),
		.CLKDIV(),
		.CLKDIVP(1'b0),
		.OCLK(clk_625mhz_io_90),
		.OCLKB(!clk_625mhz_io_90),
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
		.CLK(clk_625mhz_io_0),
		.CLKB(!clk_625mhz_io_0),
		.CE1(1'b1),
		.CE2(1'b1),
		.RST(serdes_rst),
		.CLKDIV(),
		.CLKDIVP(1'b0),
		.OCLK(clk_625mhz_io_90),
		.OCLKB(!clk_625mhz_io_90),
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
	// ISERDES output is in the 625 MHz I/O clock domain
	// Capture into the fabric domain

	logic[3:0] deser_p_ff = 0;
	logic[3:0] deser_n_ff = 0;

	always_ff @(posedge clk_625mhz_fabric) begin
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

	always_ff @(posedge clk_625mhz_fabric) begin
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

	always_ff @(posedge clk_625mhz_fabric) begin
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

	logic[3:0]	s0_last		= 0;

	logic[15:0] s1_samples	= 0;
	logic[18:0]	s1_edges	= 0;

	//Detect edges in the incoming data stream
	always_ff @(posedge clk_312p5mhz) begin
		s1_edges[18] 	<= s0_last[2] ^ s0_last[3];
		s1_edges[17] 	<= s0_last[1] ^ s0_last[2];
		s1_edges[16] 	<= s0_last[0] ^ s0_last[1];
		s1_edges[15] 	<= s0_samples[15] ^ s0_last[0];
		s1_samples		<= s0_samples;

		for(integer i=0; i<15; i=i+1)
			s1_edges[i] <= s0_samples[i] ^ s0_samples[i+1];

		s0_last			<= s0_samples[3:0];
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 2: First data bit

	logic[3:0]	s2_start = 0;
	logic		s2_data = 0;
	logic[15:0] s2_samples = 0;
	logic[15:0] s2_edges = 0;
	logic		s2_done;
	always_ff @(posedge clk_312p5mhz) begin

		s2_samples	<= s1_samples;
		s2_edges	<= s1_edges;

		//Check how long it's been since the last edge in the previous block
		s2_done = 0;
		//if(!s1_edges[18:15]

		//We know we're oversampling by about 4x.
		//So the first edge should be somewhere in the first 4 bits.
		for(integer i=15; i>11; i=i-1) begin
			if(s2_done) begin
			end

			else if(s1_edges[i]) begin
				s2_data 	<= s1_samples[i];
				s2_done 	= 1;
				s2_start	<= i-1;
			end
		end

		//If we get here, all 4 of the first samples had the same value
		//Use that as our sample value then continue searching at the 5th
		if(!s2_done) begin
			s2_data 	<= s1_samples[12];
			s2_start	<= 11;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 3: Second data bit

	//Similar to stage 2, but limits are now dynamic based on s2_start rather than hard coded.

	logic[3:0]	s3_start = 0;
	logic[1:0]	s3_data = 0;
	logic[15:0] s3_samples = 0;
	logic[15:0] s3_edges = 0;
	logic		s3_done;
	always_ff @(posedge clk_312p5mhz) begin

		s3_samples	<= s2_samples;
		s3_edges	<= s2_edges;

		s3_done = 0;
		for(integer i=0; i<4; i=i+1) begin
			if(s3_done) begin
			end

			else if(s2_edges[s2_start - i]) begin
				s3_data 	<= {s2_data, s2_samples[s2_start - i]};
				s3_done 	= 1;
				s3_start	<= s2_start - (i+1);
			end
		end

		//If we get here, all 4 of the next samples had the same value
		//Use that as our sample value then continue searching at the 5th
		if(!s3_done) begin
			s3_data 	<= {s2_data, s2_samples[s2_start - 3]};
			s3_start	<= s2_start - 4;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 4: Third data bit

	//Exactly the same as stage 3, just propagating one more upstream data bit

	logic[3:0]	s4_start = 0;
	logic[2:0]	s4_data = 0;
	logic[15:0] s4_samples = 0;
	logic[15:0] s4_edges = 0;
	logic		s4_done;
	always_ff @(posedge clk_312p5mhz) begin

		s4_samples	<= s3_samples;
		s4_edges	<= s3_edges;

		s4_done = 0;
		for(integer i=0; i<4; i=i+1) begin
			if(s4_done) begin
			end

			else if(s3_edges[s3_start - i]) begin
				s4_data 	<= {s3_data, s3_samples[s3_start - i]};
				s4_done 	= 1;
				s4_start	<= s3_start - (i+1);
			end
		end

		//If we get here, all 4 of the next samples had the same value
		//Use that as our sample value then continue searching at the 5th
		if(!s4_done) begin
			s4_data 	<= {s3_data, s3_samples[s3_start - 3]};
			s4_start	<= s3_start - 4;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 5: Fourth data bit (not always present)

	//Same as stage 4, but need to bounds check since we can run off the end of the 16-bit block

	logic[3:0]	s5_start = 0;
	logic[3:0]	s5_data = 0;
	logic[2:0]	s5_data_valid = 0;
	logic[15:0] s5_samples = 0;
	logic[15:0] s5_edges = 0;
	logic		s5_done;
	always_ff @(posedge clk_312p5mhz) begin

		s5_samples		<= s4_samples;
		s5_edges		<= s4_edges;
		s5_data_valid	<= 3;
		s5_data			<= {1'b0, s4_data};

		s5_done = 0;
		for(integer i=0; i<4; i=i+1) begin
			if(s5_done) begin
			end

			//off the end
			else if(i > s4_start) begin
				s4_done 		= 1;
			end

			else if(s4_edges[s4_start - i]) begin
				s5_data 		<= {s4_data, s4_samples[s4_start - i]};
				s5_done 		= 1;
				s5_data_valid	<= 4;
				s5_start		<= s4_start - (i+1);
			end
		end

		//If we get here, all 4 of the samples had the same value
		//Use that as our sample value then continue searching at the 5th
		if(!s5_done) begin
			s5_data 		<= {s4_data, s4_samples[s4_start - 3]};
			s5_start		<= s4_start - 4;
			s5_data_valid	<= 4;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline stage 6: Fifth data bit (rarely present)

	logic[4:0]	s6_data = 0;
	logic[2:0]	s6_data_valid = 0;
	logic		s6_done;
	always_ff @(posedge clk_312p5mhz) begin

		s6_data_valid	<= s5_data_valid;
		s6_data			<= {1'b0, s5_data};

		s6_done = 0;
		for(integer i=0; i<4; i=i+1) begin
			if(s6_done) begin
			end

			//off the end, or underflowed
			else if( (i > s5_start) || (s5_start > 6) ) begin
				s6_done 		= 1;
			end

			else if(s5_edges[s5_start - i]) begin
				s6_data 		<= {s5_data, s5_samples[s5_start - i]};
				s6_done 		= 1;
				s6_data_valid	<= 5;
			end
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output

	always_comb begin
		rx_data = s6_data;
		rx_data_count = s6_data_valid;
	end

endmodule
