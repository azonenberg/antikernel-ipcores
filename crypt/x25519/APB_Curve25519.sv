`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg                                                                          *
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

import Curve25519Registers::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief	APB wrapper around Curve25519 accelerator block
 */
module APB_Curve25519 #(
	parameter REGFILE_OUT_REG	= 0		//pipeline register for register file to enable block RAM interfence on efinix
)(
	APB.completer 				apb
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		//Words in EC point registers may be written in any order,
		//except for the rightmost word, which must be written last
		//(as this write triggers the push to the accelerator block).
		//All point registers are 256 bits (32 bytes / 8 words) long.
		//All input point registers are read-only.

		//Common
		REG_E			= 'h000,		//Outer loop variable (e for crypto_scalarmult, s for scalarmult)

		REG_STATUS		= 'h020,		//[0] = busy
		REG_RD_ADDR		= 'h024,		//read address
		REG_CMD			= 'h028,		//command (write to start an operation)
										//1 = crypto_scalarmult

		REG_STATUS_2	= 'h040,		//copy of REG_STATUS

		//crypto_scalarmult()
		REG_WORK		= 'h060,		//Input point

		//scalarmult()
		REG_Q_0			= 'h080,		//Expanded input point, block 0
		REG_Q_1			= 'h0a0,		//Expanded input point, block 1
										//Starts the point multiply automatically when last word is written

		//scalarbase()
		REG_BASE_Q_0	= 'h100,		//Expanded input point, block 0
										//Starts the point multiply automatically when last word is written

		//Readback
		REG_DATA_OUT	= 'h140,		//Readback

		//Placeholder for end of the memory map
		REG_LAST		= 'h15f

	} regid_t;

	//Block indexes
	typedef enum logic[3:0]
	{
		BLOCK_E			= REG_E[8:5],
		BLOCK_WORK		= REG_WORK[8:5],
		BLOCK_Q_0		= REG_Q_0[8:5],
		BLOCK_Q_1		= REG_Q_1[8:5],
		BLOCK_BASE_Q_0	= REG_BASE_Q_0[8:5],
		BLOCK_DATA_OUT	= REG_DATA_OUT[8:5]
	} blockid_t;

	typedef enum logic[7:0]
	{
		CMD_CRYPTO_SCALARMULT	= 'h1
	} cmd;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	logic			crypto_active	= 0;

	logic			crypt_en		= 0;
	logic[255:0]	crypt_work_in	= 0;
	logic[255:0]	crypt_e			= 0;
	wire			crypt_out_valid;
	wire[255:0]		crypt_work_out;

	logic			crypt_dsa_en;
	logic			crypt_dsa_base_en;
	logic			crypt_dsa_load;
	logic			crypt_dsa_rd;
	wire			crypt_dsa_done;
	logic[1:0]		crypt_dsa_addr;

	//index of 32-bit word within the point register
	logic[2:0]		crypt_index;

	//register we're writing to
	logic[3:0]		crypt_pointreg;

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		crypt_index		= apb.paddr[4:2];
		crypt_pointreg	= apb.paddr[8:5];

		if(apb.pready) begin

			//write
			if(apb.pwrite) begin

				//Fail if address is out of range
				if(apb.paddr > REG_LAST)
					apb.pslverr	= 1;

			end

			//read
			else begin

				//Status register readback
				if( (apb.paddr == REG_STATUS) || (apb.paddr == REG_STATUS_2) )
					apb.prdata	= { 31'h0, crypto_active };

				//Readback only allowed from results register
				else if(crypt_pointreg == BLOCK_DATA_OUT)
					apb.prdata	= crypt_work_out[crypt_index*32 +: 32];

				else
					apb.pslverr	= 1;

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			crypt_en			<= 0;
			crypto_active		<= 0;
			crypt_dsa_en		<= 0;
			crypt_dsa_base_en	<= 0;
			crypt_dsa_load		<= 0;
			crypt_dsa_rd		<= 0;
		end

		//Normal path
		else begin

			crypt_en			<= 0;
			crypt_dsa_en		<= 0;
			crypt_dsa_base_en	<= 0;
			crypt_dsa_rd		<= 0;
			crypt_dsa_load		<= 0;

			//Finish a crypto operation
			if(crypt_out_valid || crypt_dsa_done)
				crypto_active	<= 0;
			if(crypt_en || crypt_dsa_en || crypt_dsa_base_en)
				crypto_active	<= 1;

			if(apb.pready && apb.pwrite) begin

				if(apb.paddr == REG_RD_ADDR) begin
					crypt_dsa_rd	<= 1;
					crypt_dsa_addr	<= apb.pwdata[1:0];
				end

				//Start a crypto operation
				else if(apb.paddr == REG_CMD) begin

					case(apb.pwdata)
						CMD_CRYPTO_SCALARMULT:		crypt_en		<= 1;
						default: begin
						end
					endcase

				end

				//Write to a specific block
				else begin

					case(crypt_pointreg)

						BLOCK_E: crypt_e[crypt_index*32 +: 32]	<= apb.pwdata;

						BLOCK_WORK: crypt_work_in[crypt_index*32 +: 32]	<= apb.pwdata;

						BLOCK_Q_0: begin
							crypt_work_in[crypt_index*32 +: 32]	<= apb.pwdata;
							crypt_dsa_addr		<= 0;

							if(crypt_index == 7)
								crypt_dsa_load	<= 1;
						end

						BLOCK_Q_1: begin
							crypt_work_in[crypt_index*32 +: 32]	<= apb.pwdata;
							crypt_dsa_addr		<= 1;

							if(crypt_index == 7) begin
								crypt_dsa_load	<= 1;
								crypt_dsa_en	<= 1;
							end
						end

						BLOCK_BASE_Q_0: begin
							crypt_work_in[crypt_index*32 +: 32]	<= apb.pwdata;
							crypt_dsa_addr			<= 0;

							if(crypt_index == 7) begin
								crypt_dsa_load		<= 1;
								crypt_dsa_base_en	<= 1;
							end
						end

						//illegal, ignore
						default: begin
						end

					endcase
				end

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The crypto accelerator

	X25519_ScalarMult #(
		.REGFILE_OUT_REG(REGFILE_OUT_REG)
	) crypto (
		.clk(apb.pclk),

		//Common inputs
		.e(crypt_e),
		.work_in(crypt_work_in),

		//ECDH signals
		.dh_en(crypt_en),

		//ECDSA signals
		.dsa_en(crypt_dsa_en),
		.dsa_base_en(crypt_dsa_base_en),
		.dsa_load(crypt_dsa_load),
		.dsa_rd(crypt_dsa_rd),
		.dsa_done(crypt_dsa_done),
		.dsa_addr(crypt_dsa_addr),

		//Common outputs
		.out_valid(crypt_out_valid),
		.work_out(crypt_work_out)
	);

endmodule
