`timescale 1ns / 1ps
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

/**
	@brief SystemVerilog wrapper around a single UltraRAM primitive

	Pretty thin wrapper mostly just making the cascade less painful
 */
module UltraRAMWrapper #(
	parameter CASCADE_ORDER_A		= "NONE",		//NONE, FIRST, MIDDLE, LAST
	parameter CASCADE_ORDER_B		= "NONE",
	parameter IN_REG_A				= "FALSE",		//Pipeline register on port A inputs
	parameter IN_REG_B				= "FALSE",		//Pipeline register on port B inputs
	parameter OUT_REG_A				= "FALSE",		//Pipeline register on port A outputs
	parameter OUT_REG_B				= "FALSE",		//Pipeline register on port B outputs
	parameter OUT_REG_ECC_A			= "FALSE",		//Pipeline register on port A ECC outputs
	parameter OUT_REG_ECC_B			= "FALSE",		//Pipeline register on port B ECC outputs
	parameter CASCADE_REG_A			= "FALSE",		//Pipeline register on port A cascade inputs / data outputs
	parameter CASCADE_REG_B			= "FALSE",		//Pipeline register on port A cascade inputs / data outputs
	parameter MATRIX_ID				= "NONE",		//Nickname for the RAM cascade block for power reporting
	parameter SELF_ADDR_A			= 11'h000,		//Our address for cascading
	parameter SELF_ADDR_B			= 11'h000,		//Our address for cascading
	parameter SELF_MASK_A			= 11'h7ff,		//Bitmask for cascading
	parameter SELF_MASK_B			= 11'h7ff,		//Bitmask for cascading
	parameter PORTA_ECC_WR			= "FALSE",
	parameter PORTA_ECC_RD			= "FALSE",
	parameter PORTB_ECC_WR			= "FALSE",
	parameter PORTB_ECC_RD			= "FALSE"
)(

	//Shared clock for everything
	input wire					clk,

	//Power gating
	input wire					sleep,

	//Port A
	input wire					a_en,
	input wire[22:0]			a_addr,
	input wire					a_wr,
	input wire[8:0]				a_bwe,
	input wire[71:0]			a_wdata,
	input wire					a_inject_seu,
	input wire					a_inject_deu,
	input wire					a_core_ce,
	input wire					a_ecc_ce,
	input wire					a_rst,
	output wire[71:0]			a_rdata,
	output wire					a_rdaccess,
	output wire					a_seu,
	output wire					a_deu,

	//Port B
	input wire					b_en,
	input wire[22:0]			b_addr,
	input wire					b_wr,
	input wire[8:0]				b_bwe,
	input wire[71:0]			b_wdata,
	input wire					b_inject_seu,
	input wire					b_inject_deu,
	input wire					b_core_ce,
	input wire					b_ecc_ce,
	input wire					b_rst,
	output wire[71:0]			b_rdata,
	output wire					b_rdaccess,
	output wire					b_seu,
	output wire					b_deu,

	//Cascade ports
	UltraRAMCascadeBus.south	a_cascade_south,
	UltraRAMCascadeBus.north	a_cascade_north,
	UltraRAMCascadeBus.south	b_cascade_south,
	UltraRAMCascadeBus.north	b_cascade_north
);

	//TODO: enable ECCs?
	URAM288 #(
		.CASCADE_ORDER_A(CASCADE_ORDER_A),
		.CASCADE_ORDER_B(CASCADE_ORDER_B),
		.EN_AUTO_SLEEP_MODE("FALSE"),
		.EN_ECC_RD_A(PORTA_ECC_RD),
		.EN_ECC_RD_B(PORTB_ECC_RD),
		.EN_ECC_WR_A(PORTA_ECC_WR),
		.EN_ECC_WR_B(PORTB_ECC_WR),
		.IREG_PRE_A(IN_REG_A),
		.IREG_PRE_B(IN_REG_B),
		.IS_CLK_INVERTED(1'b0),
		.IS_EN_A_INVERTED(1'b0),
		.IS_EN_B_INVERTED(1'b0),
		.IS_RDB_WR_A_INVERTED(1'b0),
		.IS_RDB_WR_B_INVERTED(1'b0),
		.IS_RST_A_INVERTED(1'b0),
		.IS_RST_B_INVERTED(1'b0),
		.MATRIX_ID(MATRIX_ID),
		.OREG_A(OUT_REG_A),
		.OREG_B(OUT_REG_B),
		.OREG_ECC_A(OUT_REG_ECC_A),
		.OREG_ECC_B(OUT_REG_ECC_B),
		.REG_CAS_A(CASCADE_REG_A),
		.REG_CAS_B(CASCADE_REG_B),
		.RST_MODE_A("SYNC"),
		.RST_MODE_B("SYNC"),
		.SELF_ADDR_A(SELF_ADDR_A),
		.SELF_ADDR_B(SELF_ADDR_B),
		.SELF_MASK_A(SELF_MASK_A),
		.SELF_MASK_B(SELF_MASK_B),
		.USE_EXT_CE_A("FALSE"),
		.USE_EXT_CE_B("FALSE")
	) ram (

		//Common
		.CLK(clk),
		.SLEEP(1'b0),

		//Port A
		.ADDR_A(a_addr),
		.EN_A(a_en),
		.RDB_WR_A(a_wr),
		.BWE_A(a_bwe),
		.DIN_A(a_wdata),
		.INJECT_SBITERR_A(a_inject_seu),
		.INJECT_DBITERR_A(a_inject_deu),
		.OREG_CE_A(a_core_ce),
		.OREG_ECC_CE_A(a_ecc_ce),
		.RST_A(a_rst),
		.DOUT_A(a_rdata),
		.RDACCESS_A(a_rdaccess),
		.SBITERR_A(a_seu),
		.DBITERR_A(a_deu),

		//Port B
		.ADDR_B(b_addr),
		.EN_B(b_en),
		.RDB_WR_B(b_wr),
		.BWE_B(b_bwe),
		.DIN_B(b_wdata),
		.INJECT_SBITERR_B(b_inject_seu),
		.INJECT_DBITERR_B(b_inject_deu),
		.OREG_CE_B(b_core_ce),
		.OREG_ECC_CE_B(b_ecc_ce),
		.RST_B(b_rst),
		.DOUT_B(b_rdata),
		.RDACCESS_B(b_rdaccess),
		.SBITERR_B(b_seu),
		.DBITERR_B(b_deu),

		//Cascade
		.CAS_IN_ADDR_A(a_cascade_south.addr),
		.CAS_IN_EN_A(a_cascade_south.en),
		.CAS_IN_BWE_A(a_cascade_south.bwe),
		.CAS_IN_RDB_WR_A(a_cascade_south.rdb),
		.CAS_IN_DIN_A(a_cascade_south.din),
		.CAS_IN_DOUT_A(a_cascade_south.dout),
		.CAS_IN_RDACCESS_A(a_cascade_south.rdaccess),
		.CAS_IN_SBITERR_A(a_cascade_south.sbiterr),
		.CAS_IN_DBITERR_A(a_cascade_south.dbiterr),

		.CAS_OUT_ADDR_A(a_cascade_north.addr),
		.CAS_OUT_EN_A(a_cascade_north.en),
		.CAS_OUT_BWE_A(a_cascade_north.bwe),
		.CAS_OUT_RDB_WR_A(a_cascade_north.rdb),
		.CAS_OUT_DIN_A(a_cascade_north.din),
		.CAS_OUT_DOUT_A(a_cascade_north.dout),
		.CAS_OUT_RDACCESS_A(a_cascade_north.rdaccess),
		.CAS_OUT_SBITERR_A(a_cascade_north.sbiterr),
		.CAS_OUT_DBITERR_A(a_cascade_north.dbiterr),

		.CAS_IN_ADDR_B(b_cascade_south.addr),
		.CAS_IN_EN_B(b_cascade_south.en),
		.CAS_IN_BWE_B(b_cascade_south.bwe),
		.CAS_IN_RDB_WR_B(b_cascade_south.rdb),
		.CAS_IN_DIN_B(b_cascade_south.din),
		.CAS_IN_DOUT_B(b_cascade_south.dout),
		.CAS_IN_RDACCESS_B(b_cascade_south.rdaccess),
		.CAS_IN_SBITERR_B(b_cascade_south.sbiterr),
		.CAS_IN_DBITERR_B(b_cascade_south.dbiterr),

		.CAS_OUT_ADDR_B(b_cascade_north.addr),
		.CAS_OUT_EN_B(b_cascade_north.en),
		.CAS_OUT_BWE_B(b_cascade_north.bwe),
		.CAS_OUT_RDB_WR_B(b_cascade_north.rdb),
		.CAS_OUT_DIN_B(b_cascade_north.din),
		.CAS_OUT_DOUT_B(b_cascade_north.dout),
		.CAS_OUT_RDACCESS_B(b_cascade_north.rdaccess),
		.CAS_OUT_SBITERR_B(b_cascade_north.sbiterr),
		.CAS_OUT_DBITERR_B(b_cascade_north.dbiterr)
	);

endmodule

module UltraRAMCascadeTieoff(
	UltraRAMCascadeBus.north	tieoff
);

	assign tieoff.addr		= 0;
	assign tieoff.en		= 0;
	assign tieoff.bwe		= 0;
	assign tieoff.rdb		= 0;
	assign tieoff.din		= 0;
	assign tieoff.dout		= 0;
	assign tieoff.rdaccess	= 0;
	assign tieoff.sbiterr	= 0;
	assign tieoff.dbiterr	= 0;

endmodule
