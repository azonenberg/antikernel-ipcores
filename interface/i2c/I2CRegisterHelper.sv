`timescale 1ns / 1ps
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
	@brief Helper for accessing I2C registers

	Client interface:
		Select slave_addr
		Assert open for one clock
		Wait for ready to go high
		For each transaction:
			Set addr to register address
			Set we high for write / low for read
			Set burst_len to number of bytes to be read/written
			Assert select for one clock
		Assert close for one clock when done
 */
module I2CRegisterHelper #(
	parameter ADDR_BYTES	= 1
)(
	input wire						clk,
	input wire[7:0]					slave_addr,

	//Driver signals
	input wire						open,
	output logic					ready	= 0,
	input wire						select,
	input wire[8*ADDR_BYTES-1:0]	addr,
	input wire						we,
	input wire[7:0]					burst_len,
	input wire						close,
	output logic					rdata_valid = 0,
	output logic[7:0]				rdata		= 0,
	output logic					err			= 0,	//set if slave fails to ack any traffic
	input wire						wdata_valid,
	input wire[7:0]					wdata,
	output logic					need_wdata	= 0,
	output logic					burst_done	= 0,

	//Ports to arbiter
	output logic					request = 0,
	output logic					done = 0,
	input wire						ack,
	output i2c_in_t					cin,
	input wire i2c_out_t			cout
);

	enum logic[3:0]
	{
		STATE_IDLE						= 4'h0,
		STATE_REQUEST					= 4'h1,
		STATE_OPEN_IDLE					= 4'h2,
		STATE_SELECT_SLAVE_ADDR_WRITE	= 4'h3,
		STATE_SELECT_REG_ADDR_0			= 4'h4,
		STATE_SELECT_REG_ADDR_1			= 4'h5,
		STATE_SELECT_SLAVE_ADDR_READ	= 4'h6,
		STATE_READ_ADDR					= 4'h7,
		STATE_READ_DATA 				= 4'h8,
		STATE_WRITE_WAIT				= 4'h9,
		STATE_WRITE_DATA				= 4'ha,
		STATE_STOP						= 4'hb
	} state = STATE_IDLE;

	localparam ADDR_BITS 	= $clog2(ADDR_BYTES);
	localparam ADDR_SIZE	= 8*ADDR_BYTES;

	logic[ADDR_SIZE-1:0]	saved_addr	= 0;
	wire[ADDR_SIZE+7:0]		saved_addr_shifted	= {saved_addr, 8'h0};
	logic[ADDR_BITS-1:0]	addr_count	= 0;
	logic[7:0]				bytes_left	= 0;

	always_ff @(posedge clk) begin

		request			<= 0;
		done			<= 0;

		rdata_valid		<= 0;

		cin.tx_en		<= 0;
		cin.rx_en		<= 0;
		cin.rx_ack		<= 1;
		cin.start_en	<= 0;
		cin.restart_en	<= 0;
		cin.stop_en		<= 0;

		need_wdata		<= 0;

		burst_done		<= 0;

		case(state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for stuff to happen

			//Wait for somebody to open a session
			STATE_IDLE: begin
				if(open) begin
					request	<= 1;
					state	<= STATE_REQUEST;
				end
			end	//end STATE_IDLE

			//Wait for the arbiter to ACK us
			STATE_REQUEST: begin
				if(ack) begin
					ready	<= 1;
					state	<= STATE_OPEN_IDLE;
				end
			end	//end STATE_REQUEST

			//We have a session, waiting for commands
			STATE_OPEN_IDLE: begin

				if(select) begin
					ready			<= 0;
					cin.start_en	<= 1;
					state			<= STATE_SELECT_SLAVE_ADDR_WRITE;
				end

				if(close) begin
					done			<= 1;
					ready			<= 0;
					state			<= STATE_IDLE;
				end

			end	//end STATE_OPEN_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Select the device and send the address header

			//Slave address
			STATE_SELECT_SLAVE_ADDR_WRITE: begin

				if(!cout.busy) begin
					cin.tx_en		<= 1;
					cin.tx_data		<= { slave_addr[7:1], 1'b0 };
					saved_addr		<= addr;
					addr_count		<= 0;
					state			<= STATE_SELECT_REG_ADDR_0;
				end

			end	//end STATE_SELECT_SLAVE_ADDR_WRITE

			//Address byte
			STATE_SELECT_REG_ADDR_0: begin

				if(!cout.busy) begin
					cin.tx_en		<= 1;
					cin.tx_data		<= saved_addr[8*(ADDR_BYTES-1)+:8];
					saved_addr		<= saved_addr_shifted[ADDR_SIZE-1:0];
					addr_count		<= addr_count + 1'h1;
					state			<= STATE_SELECT_REG_ADDR_1;
				end

			end	//end STATE_SELECT_REG_ADDR_0

			//Prepare for next address byte
			STATE_SELECT_REG_ADDR_1: begin
				if(!cout.busy) begin

					//Done sending address
					if(addr_count >= ADDR_BYTES) begin

						bytes_left			<= burst_len;

						//In read mode, send a restart
						if(!we) begin
							cin.restart_en	<= 1;
							state			<= STATE_SELECT_SLAVE_ADDR_READ;
						end

						//In write mode, send data right after here (no restart)
						else begin
							need_wdata		<= 1;
							state			<= STATE_WRITE_WAIT;
						end

					end

					//Nope, send next byte
					else begin
					end

				end
			end	//end STATE_SELECT_REG_ADDR_1

			//Wait for restart then send slave address
			STATE_SELECT_SLAVE_ADDR_READ: begin
				if(!cout.busy) begin
					cin.tx_en		<= 1;
					cin.tx_data		<= { slave_addr[7:1], 1'b1 };
					state			<= STATE_READ_ADDR;
				end
			end	//end STATE_SELECT_SLAVE_ADDR_READ

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Read stuff

			//Wait for address to send, then read first data word
			STATE_READ_ADDR: begin
				if(!cout.busy) begin
					cin.rx_en	<= 1;
					state		<= STATE_READ_DATA;
				end
			end	//end STATE_READ_ADDR

			STATE_READ_DATA: begin
				if(cout.rx_rdy) begin
					rdata_valid		<= 1;
					rdata			<= cout.rx_out;

					bytes_left		<= bytes_left - 1'h1;

					if(bytes_left == 1) begin
						cin.stop_en		<= 1;
						state			<= STATE_STOP;
					end

					else
						cin.rx_en	<= 1;

				end
			end	//end STATE_READ_DATA

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Write stuff

			//Wait for the driver to send us a byte
			STATE_WRITE_WAIT: begin
				if(wdata_valid) begin
					cin.tx_en	<= 1;
					cin.tx_data	<= wdata;
					state		<= STATE_WRITE_DATA;
				end
			end	//end STATE_WRITE_WAIT

			//Wait for the byte to be written, then ask for the next if needed
			STATE_WRITE_DATA: begin

				if(!cout.busy) begin

					bytes_left		<= bytes_left - 1'h1;

					if(bytes_left == 1) begin
						cin.stop_en		<= 1;
						state			<= STATE_STOP;
					end

					else begin
						need_wdata	<= 1;
						state		<= STATE_WRITE_WAIT;
					end

				end
			end	//end STATE_WRITE_DATA

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Wait for stop

			STATE_STOP: begin
				if(!cout.busy) begin
					ready		<= 1;
					burst_done	<= 1;
					state		<= STATE_OPEN_IDLE;
				end
			end	//end STATE_STOP

		endcase

	end

endmodule
