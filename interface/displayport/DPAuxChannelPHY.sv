`timescale 1ns/1ps
`default_nettype none
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

`include "DPAuxCommands.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief DisplayPort auxiliary channel PHY

	Clock divider is sized as 9 bits because that's enoguh for a 512 MHz input
	There's no sane reason to use anything larger, we have PLLs with dividers

	DPRX shall reply to DPTX within 300us turnaround period
	On current test setup, after ~4ms of no reply we get an AUX_NACK not sure from where
 */
module DPAuxChannelPHY(

	//Debug ILA
	APB.completer			ila_apb_control,
	APB.completer			ila_apb_rom,
	APB.completer			ila_apb_data,

	//Internal system clock
	input wire				clk,

	//Clock divider for nominal symbol period
	input wire[8:0]			baud_div,

	//Aux channel PHY interface to external buffer / transceiver
	//Tested on Alinx FH6141, TODO describe the setup generically
	output wire				aux_out,
	output wire				aux_oe,
	input wire				aux_in,

	//Incoming packet stuff
	output wire				rx_header_valid,
	output wire				rx_header_has_len,
	output wire[3:0]		rx_header_command,
	output wire[19:0]		rx_header_addr,
	output wire[7:0]		rx_header_len,
	output wire				rx_packet_done,
	output wire				rx_data_valid,
	output wire[7:0]		rx_data,

	//Outgoing packet requests
	input wire				tx_start,
	input wire[3:0]			tx_header_command,
	input wire[19:0]		tx_header_addr,
	input wire[7:0]			tx_header_len,
	output wire				tx_next_byte,
	input wire[7:0]			tx_data_byte,
	output wire				tx_packet_done
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronizer on the input

	wire	aux_in_sync;

	ThreeStageSynchronizer #(.INIT(1), .IN_REG(0) )
		sync_rx (.clk_in(1'b0), .din(aux_in), .clk_out(clk), .dout(aux_in_sync));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output registers

	logic						aux_out_int	= 0;
	logic						aux_oe_int	= 0;

	logic						rx_header_valid_int		= 0;
	logic[3:0]					rx_header_command_int	= 0;
	logic[19:0]					rx_header_addr_int		= 0;
	logic[19:0]					rx_header_len_int		= 0;
	logic						rx_packet_done_int		= 0;
	logic						rx_data_valid_int		= 0;
	logic[7:0]					rx_data_int				= 0;
	logic						rx_header_has_len_int	= 0;

	logic						tx_next_byte_int		= 0;
	logic						tx_packet_done_int		= 0;

	assign aux_out				= aux_out_int;
	assign aux_oe				= aux_oe_int;
	assign rx_header_valid		= rx_header_valid_int;
	assign rx_header_command	= rx_header_command_int;
	assign rx_header_addr		= rx_header_addr_int;
	assign rx_header_len		= rx_header_len_int;
	assign rx_packet_done		= rx_packet_done_int;
	assign rx_data_valid		= rx_data_valid_int;
	assign rx_data				= rx_data_int;
	assign rx_header_has_len	= rx_header_has_len_int;

	assign tx_next_byte			= tx_next_byte_int;
	assign tx_packet_done		= tx_packet_done_int;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Helper values

	logic[7:0]				chip_time;
	logic[6:0]				half_chip_time;
	logic[8:0]				pulse_max_len;
	logic[9:0]				pulse_long_len;

	//Registered to improve timing
	//TODO: don't recompute these every clock needlessly
	always_ff @(posedge clk) begin

		//Duration of a single chip within a Manchester bit
		chip_time		<= baud_div[8:1];

		//Half the duration of a chip
		half_chip_time	<= baud_div[8:2];

		//Maximum length of a pulse (1.5 chip times)
		pulse_max_len	<= chip_time + half_chip_time;

		//Idle time before end of a packet
		pulse_long_len	<= { baud_div, 2'b0 };

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PHY bit sequence state machine

	//Manchester-II: logic 1 is high-low, logic 0 is low-high

	logic		rx_bit_valid	= 0;

	//sending LSB first
	typedef enum logic[1:0]
	{
		BIT_PAIR_LOGIC_0	= 2'b10,
		BIT_PAIR_LOGIC_1	= 2'b01,
		BIT_PAIR_DOUBLE_1	= 2'b11,
		BIT_PAIR_DOUBLE_0	= 2'b00
	} bitpair_t;

	logic[1:0]	rx_bit_value	= BIT_PAIR_LOGIC_0;

	enum logic[1:0]
	{
		//No bit sync achieved
		RX_BIT_STATE_IDLE	= 0,

		//First half of a symbol
		RX_BIT_STATE_FIRST	= 1,

		//Second half of a symbol
		RX_BIT_STATE_SECOND	= 2

	} rx_bit_state = RX_BIT_STATE_IDLE;

	//Counter since entering the current state
	logic[8:0]	rx_bit_count 		= 0;

	logic[8:0]	rx_last_toggle	= 0;
	logic		aux_in_sync_ff	= 0;
	logic		aux_in_edge;

	always_comb begin
		aux_in_edge	= (aux_in_sync != aux_in_sync_ff);
	end

	always_ff @(posedge clk) begin

		rx_bit_valid		<= 0;
		aux_in_sync_ff		<= aux_in_sync;

		//Default to advancing the count
		rx_bit_count		<= rx_bit_count + 1;

		case(rx_bit_state)

			//No bit sync. Wait for data to go high.
			RX_BIT_STATE_IDLE: begin

				//Assuming we're in the preamble, a low-to-high transition indicates the center of a logic 0 symbol
				if(aux_in_sync && !aux_in_sync_ff) begin
					rx_bit_valid	<= 1;
					rx_bit_value	<= 2'b10;
					rx_bit_state	<= RX_BIT_STATE_SECOND;
					rx_bit_count	<= 0;
				end

			end //RX_BIT_STATE_IDLE

			//We're in the first half of a bit. Expect a toggle roughly one chip time later.
			RX_BIT_STATE_FIRST: begin

				//If we see a toggle, we're correctly synced! Emit the bit
				if(aux_in_edge) begin

					//If the toggle is really early, it's probably noise - drop the bit
					if(rx_bit_count < half_chip_time) begin
						rx_bit_state	<= RX_BIT_STATE_IDLE;
					end

					else begin
						rx_bit_valid	<= 1;
						rx_bit_value[1]	<= aux_in_sync;
						rx_bit_count	<= 0;
						rx_bit_state	<= RX_BIT_STATE_SECOND;
					end
				end

				//If it's been more than 1.5 chip times, there was no inter-bit toggle
				//For now, don't bitslip.
				//Assume we're sending a manchester-violation 0/0 or 1/1 symbol
				if(rx_bit_count > pulse_max_len) begin
					rx_bit_valid	<= 1;
					rx_bit_value[1]	<= aux_in_sync;
					rx_bit_count	<= rx_bit_count + 1 - chip_time;
					rx_bit_state	<= RX_BIT_STATE_SECOND;
				end

			end

			//We're in the second half of a bit. There may or may not be a toggle roughly one chip time later.
			RX_BIT_STATE_SECOND: begin

				//If it's been more than 1.5 chip times, there was no inter-bit toggle
				//and we're already into the start of the next bit
				if(rx_bit_count > pulse_max_len) begin
					rx_bit_value[0]	<= aux_in_sync;
					rx_bit_count	<= rx_bit_count + 1 - chip_time;
					rx_bit_state	<= RX_BIT_STATE_FIRST;
				end

				//If we see a toggle to the opposite value, we are at the end of the bit and the next is the same value
				if(aux_in_edge) begin

					//If the toggle is really early, it's probably noise - drop the bit
					if(rx_bit_count < half_chip_time) begin
						rx_bit_state	<= RX_BIT_STATE_IDLE;
					end

					else begin
						rx_bit_value[0]	<= aux_in_sync;
						rx_bit_count	<= 0;
						rx_bit_state	<= RX_BIT_STATE_FIRST;
					end

				end

			end //RX_BIT_STATE_SECOND

			default: begin
				rx_bit_state	<= RX_BIT_STATE_IDLE;
				rx_bit_count	<= 0;
			end

		endcase

		//Reset counter when we see a toggle
		if(aux_in_edge)
			rx_last_toggle	<= 0;

		//Time out if it's been a while
		else begin
			rx_last_toggle	<= rx_last_toggle + 1;

			if(rx_last_toggle > pulse_long_len) begin
				rx_last_toggle	<= 0;
				rx_bit_state	<= RX_BIT_STATE_IDLE;
				rx_bit_count	<= 0;
				rx_bit_value	<= 0;
			end
		end

		//If output is enabled, ignore anything we're seeing on the bus
		if(aux_oe) begin
			rx_last_toggle	<= 0;
			rx_bit_state	<= RX_BIT_STATE_IDLE;
			rx_bit_count	<= 0;
			rx_bit_value	<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Helpers for special bit flag decoding

	logic	rx_normal_bit_valid;
	logic	rx_normal_bit_value;

	logic	rx_double_bit_valid;
	logic	rx_double_bit_value;

	logic	rx_is_normal_bit;

	always_comb begin
		rx_is_normal_bit	= (rx_bit_value == BIT_PAIR_LOGIC_0) || (rx_bit_value == BIT_PAIR_LOGIC_1);

		rx_normal_bit_valid = rx_bit_valid && rx_is_normal_bit;
		rx_normal_bit_value = (rx_bit_value == BIT_PAIR_LOGIC_1);

		rx_double_bit_valid	= rx_bit_valid && !rx_is_normal_bit;
		rx_double_bit_value = (rx_bit_value == BIT_PAIR_DOUBLE_1);

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main RX state machine

	enum logic[3:0]
	{
		RX_STATE_IDLE 			= 'h0,
		RX_STATE_PREAMBLE		= 'h1,
		RX_STATE_SYNC_0			= 'h2,
		RX_STATE_SYNC_1			= 'h3,
		RX_STATE_SYNC_2			= 'h4,
		RX_STATE_COMMAND		= 'h5,
		RX_STATE_ADDRESS		= 'h6,
		RX_STATE_LEN			= 'h7,
		RX_STATE_HEADER_DONE	= 'h8,
		RX_STATE_STOP_0			= 'h9,
		RX_STATE_STOP_1			= 'ha,
		RX_STATE_STOP_2			= 'hb,
		RX_STATE_STOP_3			= 'hc,
		RX_STATE_DATA			= 'hd,

		RX_STATE_HANG			= 'hf
	} rx_state = RX_STATE_IDLE;

	logic[8:0]	ui_count = 0;
	logic[3:0]	idle_count = 0;
	logic[10:0]	rx_count = 0;

	logic[3:0]	rx_command = 0;
	logic[19:0]	rx_address = 0;
	logic[7:0]	rx_len = 0;
	logic[7:0]	rx_data_scratch	= 0;
	logic		rx_data_valid_adv = 0;

	logic[2:0]	rx_count_low;
	logic[7:0]	rx_count_high;
	always_comb begin
		rx_count_low	= rx_count[2:0];
		rx_count_high	= rx_count[10:3];
	end

	always_ff @(posedge clk) begin

		rx_header_valid_int	<= 0;
		rx_packet_done_int	<= 0;
		rx_data_valid_int	<= 0;
		rx_data_valid_adv	<= 0;

		if(rx_data_valid_adv) begin
			rx_data_int 		<= rx_data_scratch;
			rx_data_valid_int	<= 1;
		end

		case(rx_state)

			//Idle - nothing going on. Wait for preamble
			RX_STATE_IDLE: begin

				if(rx_normal_bit_valid && !rx_normal_bit_value) begin
					rx_state				<= RX_STATE_PREAMBLE;
					rx_count				<= 1;
					rx_header_has_len_int	<= 0;
				end

			end //end RX_STATE_IDLE

			//Preamble - expect at least 16 zero bits
			RX_STATE_PREAMBLE: begin

				//If it's a zero bit, continue
				if(rx_normal_bit_valid && !rx_normal_bit_value)
					rx_count <= rx_count + 1;

				//If it's a double-one and we've seen at least 16 preamble bits, we're synced
				else if( (rx_double_bit_valid && rx_double_bit_value) && (rx_count >= 16) ) begin
					rx_state	<= RX_STATE_SYNC_0;
					rx_count	<= 0;
				end

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //end RX_STATE_PREAMBLE

			//SYNC - expect a second double-one bit
			RX_STATE_SYNC_0: begin

				if(rx_double_bit_valid && rx_double_bit_value)
					rx_state	<= RX_STATE_SYNC_1;

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //end RX_STATE_SYNC_0

			//SYNC - Expect first double-zero bit
			RX_STATE_SYNC_1: begin

				if(rx_double_bit_valid && !rx_double_bit_value)
					rx_state	<= RX_STATE_SYNC_2;

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //end RX_STATE_SYNC_1

			//SYNC - expect second double-zero bit
			RX_STATE_SYNC_2: begin

				if(rx_double_bit_valid && !rx_double_bit_value) begin
					rx_state	<= RX_STATE_COMMAND;
					rx_count	<= 0;
					rx_command	<= 0;
					rx_address	<= 0;
				end

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //end RX_STATE_SYNC_2

			//COMMAND - expect four bit command ID
			RX_STATE_COMMAND: begin

				if(rx_normal_bit_valid) begin
					rx_command[3 - rx_count]	<= rx_normal_bit_value;
					rx_count					<= rx_count + 1;
					if(rx_count == 3) begin
						rx_count				<= 0;
						rx_state				<= RX_STATE_ADDRESS;
					end
				end

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //end RX_STATE_COMMAND

			//If this is a reply, we skip the address and length fields but send 4 bits of padding

			//ADDRESS - expect 20 bit address
			RX_STATE_ADDRESS: begin

				if(rx_normal_bit_valid) begin
					rx_address[19 - rx_count]	<= rx_normal_bit_value;
					rx_count					<= rx_count + 1;
					if(rx_count == 19) begin
						rx_count				<= 0;
						rx_state				<= RX_STATE_LEN;
					end
				end

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //end RX_STATE_ADDRESS

			//LEN - expect 8-bit length
			RX_STATE_LEN: begin

				if(rx_normal_bit_valid) begin
					rx_len[7 - rx_count]		<= rx_normal_bit_value;
					rx_count					<= rx_count + 1;
					if(rx_count == 7) begin
						rx_count				<= 0;
						rx_header_has_len_int	<= 1;
						rx_state				<= RX_STATE_HEADER_DONE;
					end
				end

				//I2C transactions are allowed to jump straight from length to stop bit
				else if(rx_double_bit_valid && rx_double_bit_value && (rx_count == 0) ) begin

					rx_header_valid_int		<= 1;
					rx_header_command_int	<= rx_command;
					rx_header_addr_int		<= rx_address;
					rx_header_len_int		<= rx_len;

					rx_state				<= RX_STATE_STOP_1;
					rx_len					<= 0;
				end

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //RX_STATE_LEN

			//Headers done, advertise them
			RX_STATE_HEADER_DONE: begin
				rx_header_valid_int			<= 1;
				rx_header_command_int		<= rx_command;
				rx_header_addr_int			<= rx_address;
				rx_header_len_int			<= rx_len;

				case(rx_command)

					//If this was a read, we don't expect any data
					//Length is number of *requested* bytes, not number being sent
					DP_AUX_REQ_NATIVE_READ: begin
						rx_state			<= RX_STATE_STOP_0;
						rx_count			<= 0;
					end
					DP_AUX_REQ_I2C_READ: begin
						rx_state			<= RX_STATE_STOP_0;
						rx_count			<= 0;
					end
					DP_AUX_REQ_I2C_READ_MOT: begin
						rx_state			<= RX_STATE_STOP_0;
						rx_count			<= 0;
					end

					//If this was a write, move on to the payload
					DP_AUX_REQ_NATIVE_WRITE: begin
						rx_state			<= RX_STATE_DATA;
						rx_count			<= 0;
					end

					//If this is an I2C write, it also has a payload (or might have one)
					DP_AUX_REQ_I2C_WRITE_MOT: begin
						rx_state			<= RX_STATE_DATA;
						rx_count			<= 0;
					end
					DP_AUX_REQ_I2C_WRITE: begin
						rx_state			<= RX_STATE_DATA;
						rx_count			<= 0;
					end

					//Anything else, don't know what to do yet, drop it
					default: begin
						rx_state			<= RX_STATE_HANG;
					end

				endcase

			end //end RX_STATE_HEADER_DONE

			//Write with data
			RX_STATE_DATA: begin

				if(rx_normal_bit_valid) begin
					rx_data_scratch[7 - rx_count_low]	<= rx_normal_bit_value;
					rx_count							<= rx_count + 1;

					//End of the byte?
					if(rx_count_low == 7) begin
						rx_data_valid_adv	<= 1;

						//End of the write burst?
						if(rx_count_high == rx_header_len)
							rx_state		<= RX_STATE_STOP_0;
					end

				end

				//I2C transactions are allowed to send a count of 0 but have no payload and jump straight to stop bit
				else if(rx_double_bit_valid && rx_double_bit_value && (rx_count == 0) ) begin

					rx_header_valid_int		<= 1;
					rx_header_command_int	<= rx_command;
					rx_header_addr_int		<= rx_address;
					rx_header_len_int		<= rx_len;

					rx_state				<= RX_STATE_STOP_1;
					rx_len					<= 0;
				end

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;

			end //end RX_STATE_DATA

			//STOP: expect two double 1, two double low
			RX_STATE_STOP_0: begin
				if(rx_double_bit_valid && rx_double_bit_value)
					rx_state	<= RX_STATE_STOP_1;

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;
			end //end RX_STATE_STOP_0

			RX_STATE_STOP_1: begin
				if(rx_double_bit_valid && rx_double_bit_value)
					rx_state	<= RX_STATE_STOP_2;

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;
			end //end RX_STATE_STOP_0

			RX_STATE_STOP_2: begin
				if(rx_double_bit_valid && !rx_double_bit_value)
					rx_state	<= RX_STATE_STOP_3;

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;
			end //end RX_STATE_STOP_0

			RX_STATE_STOP_3: begin

				//Normal happy end of packet
				if(rx_double_bit_valid && !rx_double_bit_value) begin
					rx_state			<= RX_STATE_IDLE;
					rx_packet_done_int	<= 1;
				end

				//Anything else is no good, reset
				else if(rx_bit_valid)
					rx_state	<= RX_STATE_IDLE;
			end //end RX_STATE_STOP_0

			//Debug state, block until end of transaction
			RX_STATE_HANG: begin
			end // RX_STATE_HANG

		endcase

		//Time out and reset if we haven't seen any bits in a while (transaction interrupted before completion)
		ui_count	<= ui_count + 1;
		if(ui_count >= baud_div) begin
			ui_count	<= 0;
			idle_count	<= idle_count + 1;

			if(idle_count == 'hf) begin
				rx_state	<= RX_STATE_IDLE;
				rx_count	<= 0;
				idle_count	<= 0;
			end
		end
		if(rx_bit_valid) begin
			ui_count	<= 0;
			idle_count	<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX bit generator

	logic[8:0]	tx_bit_count	= 0;

	logic		tx_bit_start	= 0;
	logic		tx_bit_next		= 0;
	bitpair_t	tx_bit_value	= BIT_PAIR_LOGIC_0;

	enum logic[1:0]
	{
		TX_BIT_STATE_IDLE 		= 'h0,
		TX_BIT_STATE_FIRST		= 'h1,
		TX_BIT_STATE_SECOND		= 'h2
	} tx_bit_state = TX_BIT_STATE_IDLE;

	always_ff @(posedge clk) begin

		tx_bit_next				<= 0;

		case(tx_bit_state)

			//Do nothing if not sending
			TX_BIT_STATE_IDLE: begin
				aux_out_int		<= 0;

				if(tx_bit_start) begin
					tx_bit_state	<= TX_BIT_STATE_FIRST;
					tx_bit_count	<= 0;
				end

			end //end TX_BIT_STATE_IDLE

			TX_BIT_STATE_FIRST: begin
				tx_bit_count	<= tx_bit_count + 1;
				aux_out_int		<= tx_bit_value[0];

				if( (tx_bit_count + 1) >= chip_time) begin
					tx_bit_count	<= 0;
					tx_bit_state	<= TX_BIT_STATE_SECOND;
				end

			end //end TX_BIT_STATE_FIRST

			TX_BIT_STATE_SECOND: begin
				tx_bit_count	<= tx_bit_count + 1;
				aux_out_int		<= tx_bit_value[1];

				if( (tx_bit_count + 2) == chip_time)
					tx_bit_next		<= 1;

				if( (tx_bit_count + 1) == chip_time) begin
					tx_bit_count	<= 0;
					tx_bit_state	<= TX_BIT_STATE_FIRST;
				end
			end

			default: begin
			end


		endcase

		//If not sending anything, reset to idle
		//if(!tx_oe) begin
		if(tx_state == TX_STATE_IDLE) begin
			tx_bit_state	<= TX_BIT_STATE_IDLE;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main TX state machine

	enum logic[3:0]
	{
		TX_STATE_IDLE 			= 'h0,
		TX_STATE_PREAMBLE		= 'h1,
		TX_STATE_SYNC			= 'h2,
		TX_STATE_COMMAND		= 'h3,
		TX_STATE_ADDRESS		= 'h4,
		TX_STATE_PAD			= 'h5,
		TX_STATE_DATA			= 'h6,
		TX_STATE_STOP			= 'h7,

		TX_STATE_HANG			= 'hf
	} tx_state = TX_STATE_IDLE;

	logic[10:0] tx_count		= 0;

	logic[2:0]	tx_count_low;
	logic[7:0]	tx_count_high;
	always_comb begin
		tx_count_low	= tx_count[2:0];
		tx_count_high	= tx_count[10:3];
	end

	always_ff @(posedge clk) begin

		tx_bit_start		<= 0;
		tx_next_byte_int	<= 0;
		tx_packet_done_int	<= 0;

		case(tx_state)

			//IDLE - wait for a transmit request to happen
			TX_STATE_IDLE: begin

				//Flatline the output and float
				tx_bit_value	<= BIT_PAIR_DOUBLE_0;
				aux_oe_int		<= 0;

				if(tx_start) begin
					tx_state		<= TX_STATE_PREAMBLE;
					tx_count		<= 0;

					tx_bit_start	<= 1;
					tx_bit_value	<= BIT_PAIR_LOGIC_0;

					//Start actually driving the output
					aux_oe_int		<= 1;
				end

			end //TX_STATE_IDLE

			//Send 26 to 32 zero bits, aim for 28
			TX_STATE_PREAMBLE: begin

				if(tx_bit_next) begin
					tx_bit_value	<= BIT_PAIR_LOGIC_0;
					tx_count		<= tx_count + 1;

					//Move on to the sync words
					if(tx_count >= 27) begin
						tx_state	<= TX_STATE_SYNC;
						tx_count	<= 0;
					end

				end

				//tx_state			<= TX_STATE_IDLE;
			end //TX_STATE_PREAMBLE

			//Send two double-1 and two double-0 sync
			TX_STATE_SYNC: begin

				if(tx_bit_next) begin

					if(tx_count <= 1)
						tx_bit_value	<= BIT_PAIR_DOUBLE_1;
					else
						tx_bit_value	<= BIT_PAIR_DOUBLE_0;

					tx_count			<= tx_count + 1;

					//Move on to the command etc
					if(tx_count >= 3) begin
						tx_state		<= TX_STATE_COMMAND;
						tx_count		<= 0;
					end
				end

			end //TX_STATE_SYNC

			//Send the command
			TX_STATE_COMMAND: begin

				if(tx_bit_next) begin

					if(tx_header_command[3 - tx_count])
						tx_bit_value	<= BIT_PAIR_LOGIC_1;
					else
						tx_bit_value	<= BIT_PAIR_LOGIC_0;

					tx_count			<= tx_count + 1;

					//Move on to the next packet field
					if(tx_count >= 3) begin

						tx_count		<= 0;

						case(tx_header_command)

							//If this is a reply, we skip the address and length fields but send 4 bits of padding
							DP_AUX_REPLY_AUX_ACK: 	tx_state	<= TX_STATE_PAD;
							DP_AUX_REPLY_AUX_NACK:	tx_state	<= TX_STATE_PAD;
							DP_AUX_REPLY_I2C_DEFER:	tx_state	<= TX_STATE_PAD;
							default: 				tx_state	<= TX_STATE_ADDRESS;

						endcase

					end
				end

			end //TX_STATE_COMMAND

			//Send four bits of padding
			TX_STATE_PAD: begin

				if(tx_bit_next) begin
					tx_bit_value	<= BIT_PAIR_LOGIC_0;
					tx_count		<= tx_count + 1;

					//Move on to the next packet field
					if(tx_count >= 3) begin
						tx_count			<= 0;

						case(rx_header_command)

							//If we are responding to a write, there's no data field to send
							DP_AUX_REQ_NATIVE_WRITE:	tx_state	<= TX_STATE_STOP;
							DP_AUX_REQ_I2C_WRITE_MOT: 	tx_state	<= TX_STATE_STOP;
							DP_AUX_REQ_I2C_WRITE:		tx_state	<= TX_STATE_STOP;

							//We're doing a read
							default: begin

								//If we are responding to a read that had no length field, stop
								if(!rx_header_has_len)
									tx_state			<= TX_STATE_STOP;

								else begin
									tx_next_byte_int	<= 1;
									tx_state			<= TX_STATE_DATA;
								end

							end

						endcase

					end
				end

			end //TX_STATE_PAD

			//Send data bytes
			TX_STATE_DATA: begin

				if(tx_bit_next) begin

					if(tx_data_byte[7 - tx_count_low])
						tx_bit_value	<= BIT_PAIR_LOGIC_1;
					else
						tx_bit_value	<= BIT_PAIR_LOGIC_0;

					tx_count			<= tx_count + 1;

					//End of a byte?
					if(tx_count_low == 7) begin

						//Send the stop after the last byte
						if(tx_count_high >= tx_header_len) begin
							tx_count			<= 0;
							tx_state			<= TX_STATE_STOP;
						end

						//Otherwise ask for more data
						else
							tx_next_byte_int	<= 1;

					end

				end

			end

			//Send two double-1 and two double-0 stop symbols
			TX_STATE_STOP: begin

				if(tx_bit_next) begin

					if(tx_count <= 1)
						tx_bit_value	<= BIT_PAIR_DOUBLE_1;
					else
						tx_bit_value	<= BIT_PAIR_DOUBLE_0;

					tx_count			<= tx_count + 1;

					//Done
					//Send an extra stop bit during bus turnaround to make sure
					if(tx_count >= 4) begin
						tx_state			<= TX_STATE_IDLE;
						tx_count			<= 0;
						tx_packet_done_int	<= 1;
					end
				end

			end //TX_STATE_SYNC

			//TODO
			TX_STATE_ADDRESS: begin
			end //TX_STATE_ADDRESS

			//Nothing to do, block
			TX_STATE_HANG: begin
				tx_bit_value	<= BIT_PAIR_DOUBLE_0;
			end //TX_STATE_HANG

		endcase

		//DEBUG: reset on tx_start, we should add timeouts or something?
		if(tx_start && (tx_state != TX_STATE_IDLE) ) begin
			tx_state	<= TX_STATE_IDLE;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	logic	aux_muxed;

	always_comb begin
		if(aux_oe)
			aux_muxed = aux_out;
		else
			aux_muxed = aux_in_sync;
	end

	APB_ILA #(
		.DEPTH(2048),
		.CLK_PERIOD(50000),
		.ROM_ADDR(32'h4010_0000),
		.DATA_BUF_ADDR(32'h4020_0000),

		.PROBE0_WIDTH(1),
		.PROBE0_NAME("aux_muxed"),

		.PROBE1_WIDTH(1),
		.PROBE1_NAME("rx_data_valid"),

		.PROBE2_WIDTH(8),
		.PROBE2_NAME("rx_data"),

		.PROBE3_WIDTH(8),
		.PROBE3_NAME("rx_count_high"),

		.PROBE4_WIDTH(3),
		.PROBE4_NAME("rx_count_low"),

		.PROBE5_WIDTH(2),
		.PROBE5_NAME("rx_bit_value"),

		.PROBE6_WIDTH(1),
		.PROBE6_NAME("aux_oe"),

		.PROBE7_WIDTH(9),
		.PROBE7_NAME("rx_bit_count"),

		.PROBE8_WIDTH(1),
		.PROBE8_NAME("rx_normal_bit_valid"),

		.PROBE9_WIDTH(1),
		.PROBE9_NAME("rx_normal_bit_value"),

		.PROBE10_WIDTH(1),
		.PROBE10_NAME("rx_double_bit_valid"),

		.PROBE11_WIDTH(1),
		.PROBE11_NAME("rx_double_bit_value"),

		.PROBE12_WIDTH(4),
		.PROBE12_NAME("rx_state"),

		.PROBE13_WIDTH(2),
		.PROBE13_NAME("tx_bit_value"),

		.PROBE14_WIDTH(1),
		.PROBE14_NAME("tx_bit_start"),

		.PROBE15_WIDTH(1),
		.PROBE15_NAME("tx_next_byte"),

		.PROBE16_WIDTH(8),
		.PROBE16_NAME("tx_data_byte"),

		.PROBE17_WIDTH(1),
		.PROBE17_NAME("tx_packet_done"),

		.PROBE18_WIDTH(8),
		.PROBE18_NAME("rx_len"),

		.PROBE19_WIDTH(1),
		.PROBE19_NAME("rx_packet_done"),

		.PROBE20_WIDTH(1),
		.PROBE20_NAME("rx_header_valid"),

		.PROBE21_WIDTH(4),
		.PROBE21_NAME("rx_header_command"),

		.PROBE22_WIDTH(20),
		.PROBE22_NAME("rx_header_addr"),

		.PROBE23_WIDTH(8),
		.PROBE23_NAME("rx_header_len"),

		.PROBE24_WIDTH(1),
		.PROBE24_NAME("tx_start"),

		.PROBE25_WIDTH(4),
		.PROBE25_NAME("tx_header_command"),

		.PROBE26_WIDTH(20),
		.PROBE26_NAME("tx_header_addr"),

		.PROBE27_WIDTH(8),
		.PROBE27_NAME("tx_header_len"),

		.PROBE28_WIDTH(4),
		.PROBE28_NAME("tx_state"),

		.PROBE29_WIDTH(11),
		.PROBE29_NAME("tx_count"),

		.PROBE30_WIDTH(9),
		.PROBE30_NAME("tx_bit_count"),

		.PROBE31_WIDTH(2),
		.PROBE31_NAME("tx_bit_state")

	) ila2 (
		.apbControl(ila_apb_control),
		.apbRom(ila_apb_rom),
		.apbData(ila_apb_data),

		.clk(clk),
		.probe0(aux_muxed),
		.probe1(rx_data_valid),
		.probe2(rx_data),
		.probe3(rx_count_high),
		.probe4(rx_count_low),
		.probe5(rx_bit_value),
		.probe6(aux_oe),
		.probe7(rx_bit_count),
		.probe8(rx_normal_bit_valid),
		.probe9(rx_normal_bit_value),
		.probe10(rx_double_bit_valid),
		.probe11(rx_double_bit_value),
		.probe12(rx_state),
		.probe13(tx_bit_value),
		.probe14(tx_bit_start),
		.probe15(tx_next_byte),
		.probe16(tx_data_byte),
		.probe17(tx_packet_done),
		.probe18(rx_len),
		.probe19(rx_packet_done),
		.probe20(rx_header_valid),
		.probe21(rx_header_command),
		.probe22(rx_header_addr),
		.probe23(rx_header_len),

		.probe24(tx_start),
		.probe25(tx_header_command),
		.probe26(tx_header_addr),
		.probe27(tx_header_len),
		.probe28(tx_state),
		.probe29(tx_count),
		.probe30(tx_bit_count),
		.probe31(tx_bit_state),

		//.trig_in(rx_header_valid && (rx_header_command == DP_AUX_REQ_NATIVE_WRITE) ),
		.trig_in(rx_header_valid && (rx_header_command == DP_AUX_REQ_I2C_WRITE_MOT) ),
		.trig_out()
	);

endmodule
