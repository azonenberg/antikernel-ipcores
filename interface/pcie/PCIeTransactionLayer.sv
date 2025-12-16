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

/**
	@brief PCI Express transaction layer
 */
module PCIeTransactionLayer(

	//Link status from data link layer and LTSSM
	input wire				dl_link_up,

	//Incoming TLPs from data link layer
	//TUSER[0] is CRC-error bit set concurrently with TLAST
	AXIStream.receiver		axi_tlp_rx,

	//Outgoing TLPs to data link layer
	AXIStream.transmitter	axi_tlp_tx,

	//Incoming memory read/write requests from the host

	//Incoming configuration register read/writes
	//No backpressure allowed, must assert PREADY immediately
	APB.requester			cfg_apb

	//TODO: outgoing memory read/write requests to the host (for DMA we initiate)
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Hook up AXI TX signals

	assign axi_tlp_tx.aclk		= axi_tlp_rx.aclk;
	assign axi_tlp_tx.areset_n	= axi_tlp_rx.areset_n;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB configuration

	assign cfg_apb.pclk		= axi_tlp_rx.aclk;
	assign cfg_apb.preset_n	= axi_tlp_rx.areset_n;
	assign cfg_apb.pauser	= 0;
	assign cfg_apb.pwuser	= 0;
	assign cfg_apb.pprot	= 0;
	assign cfg_apb.pwakeup	= 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Header field enums

	typedef enum logic[2:0]
	{
		TLP_FORMAT_3DW_NODATA	= 0,
		TLP_FORMAT_4DW_NODATA	= 1,
		TLP_FORMAT_3DW_DATA		= 2,
		TLP_FORMAT_4DW_DATA		= 3,
		TLP_FORMAT_PREFIX		= 4	//not currently implemented
	} tlp_format;

	typedef enum logic[4:0]
	{
		TLP_TYPE_MEM_				= 0,
		TLP_TYPE_MEM_LOCKED			= 1,
		TLP_TYPE_IO					= 2,
		TLP_TYPE_CONFIG_0			= 4,
		TLP_TYPE_CONFIG_1			= 5,
		TLP_TYPE_COMPLETION			= 10,
		TLP_TYPE_COMPLETION_LOCKED	= 11
	} tlp_type;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Incoming TLP parsing

	logic[2:0]	rx_tlp_format		= 0;
	logic[4:0]	rx_tlp_type			= 0;
	logic[9:0]	rx_tlp_len			= 0;
	logic[15:0]	rx_tlp_requester	= 0;
	logic[7:0]	rx_tlp_tag			= 0;
	logic[31:0]	rx_tlp_addr			= 0;

	//use zero prior to the first config write
	logic[7:0]	bus_num				= 0;
	logic[4:0]	device_num			= 0;

	enum logic[3:0]
	{
		RX_TLP_STATE_IDLE			= 0,
		RX_TLP_STATE_DROP			= 1,
		RX_TLP_STATE_HEADER_1		= 2,
		RX_TLP_STATE_HEADER_2		= 3,
		RX_TLP_STATE_CFG_WDATA		= 4,
		RX_TLP_STATE_MEM_WDATA		= 5,

		RX_TLP_STATE_CFG_COMP1		= 6,
		RX_TLP_STATE_CFG_COMP2		= 7
	} rx_tlp_state	= RX_TLP_STATE_IDLE;

	always_ff @(posedge axi_tlp_rx.aclk or negedge axi_tlp_rx.areset_n) begin
		if(!axi_tlp_rx.areset_n) begin
			cfg_apb.psel		<= 0;
			cfg_apb.penable		<= 0;
			cfg_apb.paddr		<= 0;
			cfg_apb.pwdata		<= 0;
			cfg_apb.pstrb		<= 0;
			rx_tlp_format		<= 0;
			rx_tlp_type			<= 0;
			axi_tlp_rx.tready	<= 1;
			axi_tlp_tx.tvalid	<= 0;
			axi_tlp_tx.tlast	<= 0;
			bus_num				<= 0;
			device_num			<= 0;
		end

		else begin

			//Clear single cycle flags
			axi_tlp_tx.tvalid	<= 0;
			axi_tlp_tx.tlast	<= 0;

			//APB control
			if(cfg_apb.psel)
				cfg_apb.penable	<= 1;
			if(cfg_apb.pready) begin
				cfg_apb.penable	<= 0;
				cfg_apb.psel	<= 0;
			end

			//Go to idle state whenever the current TLP ends by default
			if(axi_tlp_rx.tlast && axi_tlp_rx.tvalid)
				rx_tlp_state	<= RX_TLP_STATE_IDLE;

			case(rx_tlp_state)

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Generic RX path

				//Wait for TVALID on the TLP interface
				RX_TLP_STATE_IDLE: begin

					//Ready for new traffic by default
					axi_tlp_rx.tready	<= 1;

					//First word is TLP headers
					if(axi_tlp_rx.tvalid) begin

						//Core headers
						rx_tlp_format	<= axi_tlp_rx.tdata[7:5];
						rx_tlp_type		<= axi_tlp_rx.tdata[4:0];

						//Only alid if we have a payload
						rx_tlp_len		<= { axi_tlp_rx.tdata[17:16], axi_tlp_rx.tdata[31:24] };

						//Ignore traffic class
						//Ignore TLP digest
						//Ignore attributes
						//Ignore processing hints

						//If poisoned, drop it
						if(axi_tlp_rx.tdata[22])
							rx_tlp_state	<= RX_TLP_STATE_DROP;

						//Seems valid so far, figure out what's next
						else
							rx_tlp_state	<= RX_TLP_STATE_HEADER_1;

					end

				end	//end RX_TLP_STATE_IDLE

				//Requester, tag, byte mask
				RX_TLP_STATE_HEADER_1: begin
					if(axi_tlp_rx.tvalid) begin
						rx_tlp_requester	<= axi_tlp_rx.tdata[15:0];
						rx_tlp_tag			<= axi_tlp_rx.tdata[23:15];
						//byte mask in 31:24 ignore for now we only support full width transactions
						rx_tlp_state		<= RX_TLP_STATE_HEADER_2;
					end
				end	//end RX_TLP_STATE_HEADER_1

				//Low address if 3 word header
				//High address if 4 word header
				RX_TLP_STATE_HEADER_2: begin
					if(axi_tlp_rx.tvalid) begin
						case(rx_tlp_format)

							//This header is the only one, there's no payload
							//We expect TLAST now so go back to idle
							TLP_FORMAT_3DW_NODATA: begin
								rx_tlp_addr		<=
								{
									axi_tlp_rx.tdata[7:0],
									axi_tlp_rx.tdata[15:8],
									axi_tlp_rx.tdata[23:16],
									axi_tlp_rx.tdata[31:24]
								};
								rx_tlp_state	<= RX_TLP_STATE_IDLE;

								//TODO: valid flag, process it
							end

							//This header is the only one but we have payload
							TLP_FORMAT_3DW_DATA: begin
								rx_tlp_addr		<=
								{
									axi_tlp_rx.tdata[7:0],
									axi_tlp_rx.tdata[15:8],
									axi_tlp_rx.tdata[23:16],
									axi_tlp_rx.tdata[31:24]
								};

								//See what the format is
								case(rx_tlp_type)

									//Write to configuration space
									TLP_TYPE_CONFIG_0: begin
										rx_tlp_state	<= RX_TLP_STATE_CFG_WDATA;

										//Save bus and function number from the address
										//(must update each write per spec)
										bus_num			<= axi_tlp_rx.tdata[7:0];
										device_num		<= axi_tlp_rx.tdata[15:11];

									end

									//TODO: writes to normal memory

									//If we don't know, drop the TLP
									default: begin
										rx_tlp_state	<= RX_TLP_STATE_DROP;
									end

								endcase

							end

							//TODO: 4 word (64 bit address) support
							//For now, drop it
							TLP_FORMAT_4DW_NODATA: begin
								rx_tlp_state	<= RX_TLP_STATE_DROP;
							end

							TLP_FORMAT_4DW_DATA: begin
								rx_tlp_state	<= RX_TLP_STATE_DROP;
							end

						endcase
					end
				end	//end RX_TLP_STATE_HEADER_2

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Configuration write path

				//Configuration write data
				//Should only be a single data word
				RX_TLP_STATE_CFG_WDATA: begin
					if(axi_tlp_rx.tvalid) begin

						//If TLAST is not set, drop the TLP as malformed
						if(!axi_tlp_rx.tlast)
							rx_tlp_state	<= RX_TLP_STATE_DROP;

						//This is our single data word and the last cycle. If CRC is good, send the transaction out
						else if(!axi_tlp_rx.tuser[0]) begin
							cfg_apb.psel	<= 1;
							cfg_apb.paddr	<= rx_tlp_addr;
							cfg_apb.pstrb	<= 4'b1111;
							cfg_apb.pwrite	<= 1;
							cfg_apb.pwdata	<= axi_tlp_rx.tdata;

							//Prepare to send the completion
							axi_tlp_tx.tvalid	<= 1;
							axi_tlp_tx.tkeep	<= 4'b1111;
							axi_tlp_tx.tdata	<=
							{
								//Fourth byte: no payload data
								8'h00,

								//Third byte: flags we don't care about
								8'h00,

								//Second byte: flags we don't care about
								8'h00,

								//First byte: format / type
								TLP_FORMAT_3DW_NODATA,
								TLP_TYPE_COMPLETION
							};
							rx_tlp_state		<= RX_TLP_STATE_CFG_COMP1;

							//Not ready to accept any new data from the FIFO while sending completions
							axi_tlp_rx.tready	<= 0;

						end

						//If CRC is bad during the last cycle, just go to idle without further processing
						else
							rx_tlp_state	<= RX_TLP_STATE_IDLE;

					end
				end	//end RX_TLP_STATE_CFG_WDATA

				RX_TLP_STATE_CFG_COMP1: begin

					//Send the second word of the completion
					axi_tlp_tx.tvalid	<= 1;
					axi_tlp_tx.tkeep	<= 4'b1111;
					axi_tlp_tx.tdata	<=
					{
						//Fourth byte: low byte count plus two padding bits since word sized
						rx_tlp_len[5:0], 2'b0,

						//Third byte: status and upper byte count
						//For now always make this successful completion
						//TODO: should we delay the outbound TLP until we have the actual APB success/fail?
						4'h0, rx_tlp_len[9:6],

						//Second byte: device number and function number
						//function is always zero
						device_num, 3'b0,

						//First byte: bus ID
						bus_num
					};
					rx_tlp_state		<= RX_TLP_STATE_CFG_COMP2;

				end	//end RX_TLP_STATE_CFG_COMP1

				RX_TLP_STATE_CFG_COMP2: begin

					//Send the third and final word of the completion
					axi_tlp_tx.tvalid	<= 1;
					axi_tlp_tx.tlast	<= 1;
					axi_tlp_tx.tkeep	<= 4'b1111;
					axi_tlp_tx.tdata	<=
					{
						//Fourth byte: replay low bits of address if memory read
						//this is a config so zeroes
						8'h0,

						//Third byte: replay tag
						rx_tlp_tag,

						//First and second bytes: replay requester
						rx_tlp_requester
					};

					//We're done when we get to this point
					rx_tlp_state		<= RX_TLP_STATE_IDLE;

				end	//end RX_TLP_STATE_CFG_COMP2

				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Drop path

				//Wait until the TLP ends then go back to idle
				RX_TLP_STATE_DROP: begin
				end	//end RX_TLP_STATE_DROP

			endcase

		end
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_0 ila(
		.clk(axi_tlp_rx.aclk),

		//all new
		.probe0(dl_link_up),
		.probe1(axi_tlp_rx.tvalid),
		.probe2(axi_tlp_rx.tuser),
		.probe3(axi_tlp_rx.tkeep),
		.probe4(axi_tlp_rx.tlast),
		.probe5(axi_tlp_rx.tdata),
		.probe6(rx_tlp_format),
		.probe7(rx_tlp_type),
		.probe8(rx_tlp_state),
		.probe9(cfg_apb.psel),
		.probe10(cfg_apb.penable),
		.probe11(cfg_apb.paddr),
		.probe12(cfg_apb.pwdata),
		.probe13(cfg_apb.pstrb),
		.probe14(cfg_apb.pready),
		.probe15(cfg_apb.prdata),
		.probe16(cfg_apb.pslverr),
		.probe17(rx_tlp_len),
		.probe18(rx_tlp_requester),
		.probe19(rx_tlp_tag),
		.probe20(rx_tlp_addr),
		.probe21(axi_tlp_tx.tvalid),
		.probe22(axi_tlp_rx.tready),
		.probe23(axi_tlp_tx.tkeep),
		.probe24(axi_tlp_tx.tlast),
		.probe25(axi_tlp_tx.tdata),

		.probe26(bus_num),
		.probe27(device_num)
	);

endmodule
