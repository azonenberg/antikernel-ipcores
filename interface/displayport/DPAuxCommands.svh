/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg                                                                          *
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

`ifndef DPAuxCommands_h
`define DPAuxCommands_h

typedef enum logic[3:0]
{
	DP_AUX_REQ_I2C_WRITE		= 4'b0000,
	DP_AUX_REQ_I2C_WRITE_MOT	= 4'b0100,

	DP_AUX_REQ_I2C_READ			= 4'b0001,
	DP_AUX_REQ_I2C_READ_MOT		= 4'b0101,

	DP_AUX_REQ_NATIVE_WRITE		= 4'b1000,
	DP_AUX_REQ_NATIVE_READ		= 4'b1001
} auxreq_t;

typedef enum logic[3:0]
{
	DP_AUX_REPLY_AUX_ACK	= 4'b0000,
	DP_AUX_REPLY_AUX_NACK	= 4'b0001,
	DP_AUX_REPLY_AUX_DEFER	= 4'b0010
} auxreply_t;

typedef enum logic[3:0]
{
	DP_AUX_REPLY_I2C_ACK	= 4'b0000,
	DP_AUX_REPLY_I2C_DEFER	= 4'b1000
} auxreply_i2c_t;

`endif
