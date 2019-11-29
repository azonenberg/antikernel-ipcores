`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2019 Andrew D. Zonenberg                                                                          *
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

`include "I2CTransceiver.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief Fortuna-based CSPRNG
 */
module RandomNumberGenerator(
	input wire				clk,

	//API interface
	input wire				gen_en,
	output logic			rng_valid	= 0,
	output logic[31:0]		rng_out		= 0,

	//Die serial number (from DeviceInfo_7series)
	//Tie to zero if not available in target FPGA
	input wire				die_serial_valid
	input wire[63:0]		die_serial,

	//EEPROM serial number (from I2CMACAddressReader)
	input wire				eeprom_serial_valid,
	input wire[127:0]		eeprom_serial,

	//I2C bus to EEPROM for persisting PRNG state
	output wire				i2c_driver_req,
	input wire				i2c_driver_ack,
	output wire				i2c_driver_done,
	output i2c_in_t			i2c_driver_cin,
	input wire i2c_out_t	i2c_driver_cout,

	//XADC input for runtime entropy accumulation (from OnDieSensors_7series)
	//Tie to zero if no sensors are available, but any sensor data is better than none.
	input wire[15:0]		die_temp,
	input wire[15:0]		volt_core,
	input wire[15:0]		volt_ram,
	input wire[15:0]		volt_aux,

	//Additional entropy injection from user-supplied events. Can be anything.
	//Suggested sources include clock domain jitter and timing of external events.
	input wire				entropy_en,
	input wire[31:0]		entropy_data
);

endmodule
