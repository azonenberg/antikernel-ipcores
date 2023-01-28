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

	output wire[4:0]	rx_data,
	output wire[2:0]	rx_data_count
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

	logic[15:0] s1_samples		= 0;
	logic[31:0]	s1_edges		= 0;
	logic[4:0]	s1_first_edge	= 0;
	logic[3:0]	s1_start		= 0;
	logic[3:0]	s1_start_old	= 0;
	logic		s1_found_edge	= 0;

	//Detect edges in the incoming data stream
	always_ff @(posedge clk_312p5mhz) begin
		s1_samples		<= s0_samples;

		s1_edges[31:16] = s1_edges[15:0];
		s1_edges[15] 	= s0_samples[15] ^ s1_samples[0];
		s1_edges[14:0]	= s0_samples[14:0] ^ s0_samples[15:1];

		//Max allowed run length is 22 consecutive bits with the same value, so there's guaranteed to be an edge
		//somewhere in the first 32 bits.
		//Use a somewhat weird search pattern to try and get most accurate phase alignment.
		s1_start_old = s1_start;
		s1_found_edge = 0;
		s1_first_edge = 0;

		//If there's an edge in the first 4 bits, use it
		for(integer i=15; i>=12; i=i-1) begin
			if(s1_edges[i] && !s1_found_edge) begin
				s1_first_edge = i;
				s1_found_edge = 1;
				s1_start = i - 1;

				//Edge cases around split: a run of 2+3 (5) or 6+3 (9) or 10+3 should be skipped, because the previous
				//block already saw the edge. But a run of 3+3 (6) or 4+3 (7) should be rounded up to two bits, and a run
				//of 1+3 should be rounded up to 1.
				if(i == 12) begin

					//Even trickier edge case: if the entire previous block has no edges, and started sampling as N=12
					//then its last sample will be at n=0
					//so we're in a 2+3 case and should skip
					if(!s1_edges[31:16] && (s1_start_old == 12) ) begin
					end

					else if(!s1_edges[28:15] && s1_edges[29]) begin
					end

					else if(!s1_edges[20:15] && s1_edges[21]) begin
					end

					else if(!s1_edges[24:15] && s1_edges[25]) begin
					end

					else if(!s1_edges[17:15] || s1_edges[16])
						s1_start = 15;
				end

				//A run of 5+2 should decode to two bits, and 1+2 should decode to one.
				//6+2 should decode to two. 10+2 should decode to three in the previous block and none here.
				//7+2 should decode to two in the previous and none here.
				//11+2 should decode to three in the previous block and none here
				//15+2 should decode to four plus none here, as should 14+3
				if(i == 13) begin

					//Even trickier edge case: if the entire previous block has no edges, and started sampling as N=12
					//then its last sample will be at n=0
					//so we're in a 2+16+2 case and should skip (total of 20).
					//Same applies to 3+16+2, total 21
					if(!s1_edges[31:16] && ( (s1_start_old == 12) || (s1_start_old == 13) ) ) begin
					end

					else if(!s1_edges[30:15] && s1_edges[31]) begin
					end

					else if(!s1_edges[29:15] && s1_edges[30]) begin
					end

					else if(!s1_edges[28:15] && s1_edges[29]) begin
					end

					else if(!s1_edges[25:15] && s1_edges[26]) begin
					end

					else if(!s1_edges[24:15] && s1_edges[25]) begin
					end

					else if(!s1_edges[21:15] && s1_edges[22]) begin
					end

					else if(!s1_edges[20:15] && s1_edges[21]) begin
					end

					else if(!s1_edges[19:14] || s1_edges[16])
						s1_start = 15;
				end

			end
		end

		if(!s1_found_edge) begin

			//If not, look back into the previous block
			for(integer i=16; i<31; i=i+1) begin
				if(s1_edges[i] && !s1_found_edge) begin
					s1_first_edge = i;
					s1_found_edge = 1;
				end
			end

			//If not found, search forwards
			for(integer i = 11; i>=0; i=i-1) begin
				if(s1_edges[i] && !s1_found_edge) begin
					s1_first_edge = i;
					s1_found_edge = 1;
				end
			end

			//First sample point is one after the first edge (mod 4)
			case(s1_first_edge[1:0])
				0:	s1_start	= 15;
				1:	s1_start	= 12;
				2:	s1_start	= 13;
				3:	s1_start	= 14;
			endcase

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// First data bit

	wire[3:0]	s2_start;
	wire[2:0]	s2_count;
	wire[4:0]	s2_data;
	wire[15:0]	s2_samples;
	wire[15:0]	s2_edges;
	wire		s2_done;

	OversamplingCDRBitslice #(
		.OUT_REG(1)
	) stage2 (
		.clk(clk_312p5mhz),

		.samples(s1_samples),
		.edges(s1_edges[15:0]),
		.start(s1_start),
		.data(5'b0),
		.count(3'b0),
		.done(1'b0),

		.samples_next(s2_samples),
		.edges_next(s2_edges),
		.start_next(s2_start),
		.data_next(s2_data),
		.count_next(s2_count),
		.done_next(s2_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second data bit

	wire[3:0]	s3_start;
	wire[2:0]	s3_count;
	wire[4:0]	s3_data;
	wire[15:0]	s3_samples;
	wire[15:0]	s3_edges;
	wire		s3_done;

	OversamplingCDRBitslice #(
		.OUT_REG(1)
	) stage3 (
		.clk(clk_312p5mhz),

		.samples(s2_samples),
		.edges(s2_edges),
		.start(s2_start),
		.data(s2_data),
		.count(s2_count),
		.done(s2_done),

		.samples_next(s3_samples),
		.edges_next(s3_edges),
		.start_next(s3_start),
		.data_next(s3_data),
		.count_next(s3_count),
		.done_next(s3_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Third data bit

	wire[3:0]	s4_start;
	wire[2:0]	s4_count;
	wire[4:0]	s4_data;
	wire[15:0]	s4_samples;
	wire[15:0]	s4_edges;
	wire		s4_done;

	OversamplingCDRBitslice #(
		.OUT_REG(1)
	) stage4 (
		.clk(clk_312p5mhz),

		.samples(s3_samples),
		.edges(s3_edges),
		.start(s3_start),
		.data(s3_data),
		.count(s3_count),
		.done(s3_done),

		.samples_next(s4_samples),
		.edges_next(s4_edges),
		.start_next(s4_start),
		.data_next(s4_data),
		.count_next(s4_count),
		.done_next(s4_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Fourth (usually present) data bit

	wire[3:0]	s5_start;
	wire[2:0]	s5_count;
	wire[4:0]	s5_data;
	wire[15:0]	s5_samples;
	wire[15:0]	s5_edges;
	wire		s5_done;

	OversamplingCDRBitslice #(
		.OUT_REG(1)
	) stage5 (
		.clk(clk_312p5mhz),

		.samples(s4_samples),
		.edges(s4_edges),
		.start(s4_start),
		.data(s4_data),
		.count(s4_count),
		.done(s4_done),

		.samples_next(s5_samples),
		.edges_next(s5_edges),
		.start_next(s5_start),
		.data_next(s5_data),
		.count_next(s5_count),
		.done_next(s5_done)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Fifth (rarely present) data bit

	OversamplingCDRBitslice #(
		.OUT_REG(1)
	) stage6 (
		.clk(clk_312p5mhz),

		.samples(s5_samples),
		.edges(s5_edges),
		.start(s5_start),
		.data(s5_data),
		.count(s5_count),
		.done(s5_done),

		.samples_next(),
		.edges_next(),
		.start_next(),
		.data_next(rx_data),
		.count_next(rx_data_count),
		.done_next()
	);

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OversamplingCDRBitslice

module OversamplingCDRBitslice #(
	parameter OUT_REG = 1
)(
	input wire			clk,

	input wire[15:0]	samples,
	input wire[15:0]	edges,
	input wire[3:0]		start,
	input wire[4:0]		data,
	input wire[2:0]		count,
	input wire			done,

	output logic[15:0]	samples_next = 0,
	output logic[15:0]	edges_next = 0,
	output logic[3:0]	start_next = 0,
	output logic[4:0]	data_next = 0,
	output logic[2:0]	count_next = 0,
	output logic		done_next = 0
);

	logic[3:0]	start_comb;
	logic[4:0]	data_comb;
	logic[2:0]	count_comb;
	logic		done_comb;

	always_comb begin

		//Push down the pipeline
		data_comb		= data;
		count_comb		= count;
		done_comb		= done;

		//Default to not touching  start/done
		start_comb		= start;
		done_comb		= done;

		//If we're done, nothing further to do
		if(done) begin
		end

		//If this is the very first sample (count == 0),
		//and there is an edge the cycle after our sample, do nothing
		else if(edges[start-1] && !count) begin
		end

		else begin

			//Take a data sample
			count_comb		= count + 1;
			data_comb		= { data[3:0], samples[start] };

			//We expect an edge (or lack thereof) 3-5 samples after the previous edge
			if(edges[start - 2]) begin
				start_comb	= start - 3;
				done_comb	= (start < 3);
			end
			else if(edges[start - 3]) begin
				start_comb	= start - 4;
				done_comb	= (start < 4);
			end
			else if(edges[start - 4]) begin
				start_comb	= start - 5;
				done_comb	= (start < 5);
			end

			//If we don't find one, next sample should be 4 later
			else begin
				start_comb	= start - 4;
				done_comb	= (start < 4);
			end

		end

	end

	//Registered output
	if(OUT_REG) begin
		always_ff @(posedge clk) begin
			samples_next	<= samples;
			edges_next		<= edges;

			count_next		<= count_comb;
			data_next		<= data_comb;
			done_next		<= done_comb;
			start_next		<= start_comb;
		end
	end

	//Combinatorial passthrough
	else begin
		always_comb begin
			samples_next	= samples;
			edges_next		= edges;

			count_next		= count_comb;
			data_next		= data_comb;
			done_next		= done_comb;
			start_next		= start_comb;
		end
	end

endmodule
