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
	@file Curve25519_Types.sv
	@author Andrew D. Zonenberg
	@brief Structure definitions for Curve25519 stuff
 */
package Curve25519Registers;

typedef struct packed
{
	logic[15:0]		u;
} uint16_t;

typedef struct packed
{
	uint16_t[31:0]	blocks;
} bignum_t;

typedef struct packed
{
	logic[31:0]		u;
} uint32_t;

typedef struct packed
{
	uint32_t[31:0]	blocks;
} bignum32_t;


typedef enum logic[3:0]
{
	//General purpose registers, writable and usable everywhere
	REG_TEMP_0		= 4'h00,
	REG_TEMP_1		= 4'h01,
	REG_TEMP_2		= 4'h02,
	REG_TEMP_3		= 4'h03,
	REG_TEMP_4		= 4'h04,
	REG_TEMP_5		= 4'h05,
	REG_TEMP_6		= 4'h06,
	REG_TEMP_7		= 4'h07,
	REG_TEMP_8		= 4'h08,
	REG_TEMP_9		= 4'h09,
	REG_TEMP_10		= 4'h0a,

	//Special registers (named, but not always usable in every operation)
	REG_121665		= 4'h0b,	//constant 121665
	REG_ZERO		= 4'h0c,	//constant 0, writes ignored
	REG_ONE			= 4'h0d,	//constant 1
	REG_D2			= 4'h0e,	//constant 256'h2406d9dc56dffce7198e80f2eef3d13000e0149a8283b156ebd69b9426b2f159
	REG_Y			= 4'h0f		//constant 256'h6666666666666666666666666666666666666666666666666666666666666658

} xregid_t;

typedef logic[263:0] regval_t;

endpackage
