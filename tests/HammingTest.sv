`timescale 1ns / 1ps
`default_nettype none
module HammingTest();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clocking

	logic	clk = 0;
	always begin
		#5;
		clk = !clk;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Encoder

	logic[63:0] plaintext = 0;
	wire		codeword_valid;
	wire[71:0]	codeword;

	HammingEncoder #(
		.DATA_BITS(64)
	) encoder (
		.clk(clk),
		.data_in(plaintext),
		.valid_in(1'b1),
		.valid_out(codeword_valid),
		.data_out(codeword)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Error injection

	logic		corrupted_valid = 0;
	logic[71:0]	corrupted = 0;

	integer		numbits;
	integer		bitpos;

	always_ff @(posedge clk) begin
		corrupted_valid	<= codeword_valid;
		corrupted		<= codeword;

		//Generate plaintext
		plaintext = {$urandom(), $urandom()};

		//Decide how many bits to corrupt
		numbits = $urandom_range(0, 2);

		//Corrupt the requested number of bits
		//No check for duplication so it's possible a "double bit" error may result in a single bit error instead
		//TODO: is it worth working around this?
		for(integer i=0; i<numbits; i=i+1) begin
			bitpos = $urandom_range(0, 71);
			corrupted[bitpos] <= !codeword[bitpos];
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Decoder

	wire		decoded_valid;
	wire[63:0]	decoded;

	wire		correctable_err;
	wire		uncorrectable_err;

	HammingDecoder #(
		.DATA_BITS(64)
	) decoder (
		.clk(clk),
		.data_in(corrupted),
		.valid_in(corrupted_valid),
		.valid_out(decoded_valid),
		.data_out(decoded),
		.correctable_err(correctable_err),
		.uncorrectable_err(uncorrectable_err)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Result verification

	logic[63:0]	plaintext_ff	= 0;
	logic[63:0]	plaintext_ff2	= 0;
	logic[63:0]	plaintext_ff3	= 0;

	integer	numbits_ff		= 0;
	integer	numbits_ff2		= 0;

	logic	uncorrectable_expected;
	logic	correctable_expected;
	always_comb begin
		correctable_expected = (numbits_ff2 == 1);
		uncorrectable_expected = (numbits_ff2 == 2);
	end

	always_ff @(posedge clk) begin

		//Pipeline delay inputs
		plaintext_ff	<= plaintext;
		plaintext_ff2	<= plaintext_ff;
		plaintext_ff3	<= plaintext_ff2;

		numbits_ff		<= numbits;
		numbits_ff2		<= numbits_ff;

		//Sanity check results
		if(decoded_valid) begin
			assert(correctable_expected == correctable_err);
			assert(uncorrectable_expected == uncorrectable_err);

			if(!uncorrectable_err)
				assert(decoded == plaintext_ff3);

		end

	end

endmodule

