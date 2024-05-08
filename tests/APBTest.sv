`timescale 1ns/1ps
`default_nettype none

import APBTypes::*

module APBTest();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock generation

	logic clk = 0;
	always begin
		#1;
		clk = 1;
		#1;
		clk = 0;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sample bus

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(32), .USER_WIDTH(0)) apb();

	wire[63:0]	regval;
	wire		updated;

	APBSimpleRegister #(
		.REG_WIDTH(64),
		.INIT(64'hcccccccc_cccccccc)
	) reg1 (
		.apb(apb),
		.regval_out(regval),
		.updated(updated)
	);

	TestGenerator gen(
		.clk(clk),
		.apb(apb)
	);

endmodule

module TestGenerator(
	input wire	clk,

	APB.requester apb
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Test logic

	assign apb.pclk = clk;

	logic[7:0] state = 0;

	always_ff @(posedge clk) begin

		case(state)

			0: begin
				apb.preset_n 	<= 0;
				apb.psel		<= 0;
				apb.penable		<= 0;
				apb.pwrite		<= 0;
				apb.pwdata		<= 0;
				state			<= 1;
			end

			1: begin
				apb.preset_n	<= 1;
				state			<= 2;
			end

			//Do a write
			2: begin
				apb.psel		<= 1;
				apb.pwrite		<= 1;
				apb.paddr		<= 4;
				apb.pwdata		<= 16'haaaa;
				state			<= 3;
			end

			3: begin
				apb.penable		<= 1;
				state			<= 4;
			end

			4: begin
				if(apb.pready) begin
					apb.penable	<= 0;
					apb.psel	<= 0;
					state		<= 5;
				end
			end

			//todo
			5: begin
			end

		endcase

	end

endmodule
