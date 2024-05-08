`timescale 1ns/1ps
`default_nettype none

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
	// Interconnect bridge

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(32), .USER_WIDTH(0)) processorBus();
	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(12), .USER_WIDTH(0)) peripheralBus[3:0]();

	APBBridge #(
		.BASE_ADDR(32'h4000_0000),
		.BLOCK_SIZE(32'h800),
		.NUM_PORTS(4)
	) bridge (
		.upstream(processorBus),
		.downstream(peripheralBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Peripherals

	wire[63:0]	regvalA;
	wire		updatedA;

	wire[63:0]	regvalB;
	wire		updatedB;

	//4000_0000
	APBSimpleRegister #(
		.REG_WIDTH(64),
		.INIT(64'haaaaaaaa_aaaaaaaa)
	) regA (
		.apb(peripheralBus[0]),
		.regval_out(regvalA),
		.updated(updatedA)
	);

	//4000_0800
	APBSimpleRegister #(
		.REG_WIDTH(64),
		.INIT(64'hbbbbbbbb_bbbbbbbb)
	) regB (
		.apb(peripheralBus[1]),
		.regval_out(regvalB),
		.updated(updatedB)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Stimulus generation

	TestGenerator gen(
		.clk(clk),
		.apb(processorBus),

		.regvalA(regvalA),
		.regvalB(regvalB),
		.updatedA(updatedA),
		.updatedB(updatedB)
	);

endmodule

module TestGenerator(
	input wire	clk,

	APB.requester apb,

	input wire[63:0]	regvalA,
	input wire[63:0]	regvalB,
	input wire			updatedA,
	input wire			updatedB
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Test logic

	assign apb.pclk = clk;

	logic[7:0] state = 0;

	always_ff @(posedge clk) begin

		if(apb.pready)
			apb.penable	<= 0;

		case(state)

			0: begin
				apb.preset_n 	<= 0;
				apb.psel		<= 0;
				apb.penable		<= 0;
				apb.pwrite		<= 0;
				apb.pwdata		<= 0;
				state			<= 1;

				//clear other fields we don't use
				apb.pwakeup		<= 0;
				apb.pprot		<= 0;
				apb.pstrb		<= 2'b11;
			end

			1: begin
				apb.preset_n	<= 1;
				state			<= 2;
			end

			//Do a write to the first device
			2: begin
				apb.psel		<= 1;
				apb.pwrite		<= 1;
				apb.paddr		<= 32'h4000_0004;
				apb.pwdata		<= 16'h5555;
				state			<= 3;
			end

			3: begin
				apb.penable		<= 1;
				state			<= 4;
			end

			//Do a write to the second device
			4: begin
				if(apb.pready) begin
					state			<= 5;
					apb.psel		<= 1;
					apb.pwrite		<= 1;
					apb.paddr		<= 32'h4000_0806;
					apb.pwdata		<= 16'hcccc;
					state			<= 5;
				end
			end

			5: begin

				//verify the previous write was successful
				assert(updatedA) else $display("A not updated on first write");
				assert(regvalA[4*8 +: 16] == 16'h5555);

				//and that the other device wasn't touched
				assert(!updatedB) else $display("B was updated on first write");
				assert(regvalB[4*8 +: 16] == 16'hbbbb);

				apb.penable		<= 1;
				state			<= 6;
			end

			//Read from the second device
			6: begin
				if(apb.pready) begin
					apb.psel	<= 1;
					apb.pwrite	<= 0;
					apb.paddr	<= 32'h4000_0800;
					state		<= 7;
				end
			end

			7: begin
				//verify the write was successful
				assert(updatedB) else $display("B not updated on second write");
				assert(regvalB[6*8 +: 16] == 16'hcccc);

				//and that the other device wasn't touched
				assert(!updatedA) else $display("A was updated on second write");
				assert(regvalA[6*8 +: 16] == 16'haaaa);

				apb.penable	<= 1;
				state		<= 8;
			end

			8: begin

				if(apb.pready) begin

					assert(!updatedA) else $display("A updated on read!");
					assert(!updatedB) else $display("B updated on read!");
					assert(apb.prdata == 16'hbbbb);

					apb.psel	<= 1;
					apb.pwrite	<= 0;
					apb.paddr	<= 32'h4000_0000;
					state		<= 9;
				end

			end

			9: begin
				apb.penable		<= 1;
				state			<= 10;
			end

			10: begin
				if(apb.pready) begin

					assert(!updatedA) else $display("A updated on read!");
					assert(!updatedB) else $display("B updated on read!");
					assert(apb.prdata == 16'haaaa);
					state		<= 11;
				end
			end

			11: begin
			end

		endcase

	end

endmodule
