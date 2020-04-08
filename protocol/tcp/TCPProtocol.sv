`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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

`include "EthernetBus.svh"
`include "IPv4Bus.svh"
`include "TCPv4Bus.svh"
`include "IPProtocols.svh"

/**
	@brief The TCP protocol implementation
 */
module TCPProtocol #(
	parameter		AGE_INTERVAL	= 125000000,		//clocks per aging tick (default is 1 Hz @ 125 MHz)
	parameter		MAX_AGE			= 60,				//close sockets after a minute of inactivity

	localparam		MAX_SOCKETS		= 2048,				//TODO: SocketManager config affects this
	localparam		SOCKET_BITS		= $clog2(MAX_SOCKETS)
)(

	//Clocks
	input wire				clk,

	//Address information
	input wire IPv4Config	ip_config,

	//Incoming data bus from IP stack
	//TODO: make this parameterizable for IPv4/IPv6, for now we only do v4
	input wire IPv4RxBus	rx_l3_bus,
	output IPv4TxBus		tx_l3_bus	=	{$bits(IPv4TxBus){1'b0}},

	//Outbound bus to applications
	output TCPv4RxBus		rx_l4_bus	=	{$bits(TCPv4RxBus){1'b0}},
	input wire TCPv4TxBus	tx_l4_bus,

	//Port opening/closing
	input wire				port_open_en,
	input wire				port_close_en,
	input wire portnum_t	port_num
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Table of open/closed port numbers

	localparam PORT_WAYS = 4;

	typedef portnum_t[PORT_WAYS-1:0]	port_row_t;
	port_row_t	open_port_table[255:0];

	initial begin
		for(integer i=0; i<256; i++)
			open_port_table[i] <= 0;
	end

	logic		port_check_en		= 0;
	portnum_t	port_check_num		= 0;
	logic		port_check_is_open	= 0;

	logic		found				= 0;
	always_ff @(posedge clk) begin

		//Open ports
		if(port_open_en) begin
			found	= 0;
			for(integer i=0; i<PORT_WAYS; i++) begin
				if(!found && (open_port_table[port_num[7:0]][i] == 0) ) begin
					open_port_table[port_num[7:0]][i]	<= port_num;
					found								= 1;
				end
			end
		end

		//Close ports
		else if(port_close_en) begin
			for(integer i=0; i<PORT_WAYS; i++) begin
				if(open_port_table[port_num[7:0]][i] == port_num)
					open_port_table[port_num[7:0]][i]	<= 0;
			end
		end

		//Port state lookup
		if(port_check_en) begin
			port_check_is_open	<= 0;

			for(integer i=0; i<PORT_WAYS; i++) begin
				if(open_port_table[port_check_num[7:0]][i] == port_check_num)
					port_check_is_open	<= 1;
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Outbound sequence number generation LFSR

	wire[31:0]	next_seq;

	PRBS31 #(
		.WIDTH(32),
		.INITIAL_SEED(31'h0eadbeef),
		.MSB_FIRST(0)
	) seq_prbs (
		.clk(clk),
		.update(1'b1),
		.init(1'b0),
		.seed(31'h0),
		.dout(next_seq)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Socket state

	//The SocketManager stores mappings from (sport, dport, client_ip) to socket handles.
	//We need to store other state ourselves.

	logic						lookup_en		= 0;
	socketstate_t				lookup_headers	= 0;
	wire						lookup_done;
	wire						lookup_hit;
	wire[SOCKET_BITS-1:0]		lookup_sockid;

	logic						insert_en		= 0;
	socketstate_t				insert_headers	= 0;
	wire						insert_done;
	wire						insert_fail;
	wire[SOCKET_BITS-1:0]		insert_sockid;

	logic						aging_tick		= 0;

	logic						remove_en		= 0;
	logic[SOCKET_BITS-1:0]		remove_sockid	= 0;

	SocketManager #(
		.WAYS(4),
		.LATENCY(2),	//must be 2 to allow for proper lookups of headers
		.BINS(256)
	) state_mgr (
		.clk(clk),

		.lookup_en(lookup_en),
		.lookup_headers(lookup_headers),
		.lookup_done(lookup_done),
		.lookup_hit(lookup_hit),
		.lookup_sockid(lookup_sockid),

		.insert_en(insert_en),
		.insert_headers(insert_headers),
		.insert_done(insert_done),
		.insert_fail(insert_fail),
		.insert_sockid(insert_sockid),

		.remove_en(),
		.remove_sockid(),
		.remove_done(),

		.aging_tick(aging_tick),
		.max_age(10'd30)
	);

	typedef enum logic[1:0]
	{
		TCP_STATE_HALF_OPEN	=	0,	//Sent SYN+ACK, no ACK yet
		TCP_STATE_OPEN		=	1,	//Socket is ready for data
		TCP_STATE_CLOSED	=	2	//Socket is closed, discard traffic from it
	} tcpstate_t;

	typedef struct packed
	{
		tcpstate_t	state;		//current state of the connection
		logic[31:0]	tx_seq;		//Starting sequence number for our next outbound segment
		logic[31:0]	rx_seq;		//Expected sequence number of the next inbound segment
		logic[15:0] tx_window;	//max window we're able to send (no scaling yet)
	} tcpsocket_t;

	logic						state_wr_en		= 0;
	logic[SOCKET_BITS-1:0]		state_wr_addr	= 0;
	tcpsocket_t					state_wr_data;

	tcpsocket_t					state_rd_data;

	MemoryMacro #(
		.WIDTH($bits(tcpsocket_t)),
		.DEPTH(MAX_SOCKETS),
		.USE_BLOCK(1),
		.OUT_REG(1),
		.DUAL_PORT(1),
		.TRUE_DUAL(0)
	) sockstate_mem (
		.porta_clk(clk),
		.porta_en(state_wr_en),
		.porta_addr(state_wr_addr),
		.porta_we(state_wr_en),
		.porta_din(state_wr_data),
		.porta_dout(),

		.portb_clk(clk),
		.portb_en(lookup_hit),
		.portb_addr(lookup_sockid),
		.portb_we(1'b0),
		.portb_din(),
		.portb_dout(state_rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Aging timer for sockets

	logic[31:0]	aging_count;

	always_ff @(posedge clk) begin

		aging_count	<= aging_count + 1'h1;
		aging_tick	<= 0;

		if(aging_count == (AGE_INTERVAL - 1)) begin
			aging_count	<= 0;
			aging_tick	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX checksum verification

	wire[15:0]	rx_checksum_expected;

	InternetChecksum32bit rx_csum(
		.clk(clk),
		.load(rx_l3_bus.start),
		.reset(1'b0),
		.process(rx_l3_bus.data_valid),
		.din(rx_l3_bus.data_valid ? rx_l3_bus.data : rx_l3_bus.pseudo_header_csum),
		.sumout(),
		.csumout(rx_checksum_expected)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Events (header-only packets) that the RX logic wants the TX side to send

	typedef struct packed
	{
		logic[31:0]	remote_ip;
		logic[15:0]	remote_port;
		logic[15:0]	our_port;

		logic		flag_syn;
		logic		flag_ack;
		logic		flag_fin;
		logic		flag_rst;

		logic[31:0]	seq;
		logic[31:0]	ack;

	} event_t;

	logic		event_wr_en	= 0;
	event_t		wr_event;

	wire[5:0]	event_rsize;
	logic		event_rd_en	= 0;
	event_t		rd_event;

	/*
		No need for overflow handling or size checks here.

		FIFO is well defined to discard writes when full.
		If a write is lost, this is no worse than the network dropping a packet, so the TCP protocol
		will recover just fine.
	 */
	SingleClockFifo #(
		.WIDTH($bits(event_t)),
		.DEPTH(32),
		.USE_BLOCK(0)
	) event_fifo (
		.clk(clk),

		.wr(event_wr_en),
		.din(wr_event),

		.overflow(),
		.underflow(),
		.empty(),
		.full(),
		.rsize(event_rsize),
		.wsize(),

		.reset(1'b0),

		.rd(event_rd_en),
		.dout(rd_event)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side

	enum logic[3:0]
	{
		RX_STATE_IDLE				= 4'h0,
		RX_STATE_PORT_HEADER		= 4'h1,
		RX_STATE_SEQ_HEADER			= 4'h2,
		RX_STATE_ACK_HEADER			= 4'h3,
		RX_STATE_FLAG_HEADER		= 4'h4,
		RX_STATE_CHECKSUM_HEADER	= 4'h5,
		RX_STATE_OPTION_HEADER		= 4'h6,
		RX_STATE_DATA				= 4'h7,
		RX_STATE_CHECKSUM			= 4'h8,
		RX_STATE_CREATE_SOCKET		= 4'h9
	} rx_state = RX_STATE_IDLE;

	logic[31:0]	rx_current_seq		= 0;
	logic[31:0]	rx_current_ack		= 0;
	logic[15:0]	rx_current_window	= 0;
	logic[3:0]	rx_data_offset		= 0;
	logic		rx_flag_ack			= 0;
	logic		rx_flag_syn			= 0;
	logic		rx_flag_fin			= 0;
	logic		rx_flag_rst			= 0;

	logic[3:0]	rx_current_option	= 0;

	always_ff @(posedge clk) begin

		lookup_en				<= 0;
		insert_en				<= 0;
		remove_en				<= 0;

		rx_l4_bus.start			<= 0;
		rx_l4_bus.data_valid	<= 0;
		rx_l4_bus.commit		<= 0;
		rx_l4_bus.drop			<= 0;

		event_wr_en				<= 0;
		state_wr_en				<= 0;

		port_check_en			<= 0;

		case(rx_state)

			//Start when we get a new packet
			RX_STATE_IDLE: begin

				if(rx_l3_bus.start && (rx_l3_bus.payload_len >= 20) && rx_l3_bus.protocol_is_tcp )
					rx_state			<= RX_STATE_PORT_HEADER;

			end	//end RX_STATE_IDLE

			RX_STATE_PORT_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state	<= RX_STATE_IDLE;

					else begin
						//rx_l4_bus.start				<= 1;
						rx_l4_bus.src_port			<= rx_l3_bus.data[31:16];
						rx_l4_bus.dst_port			<= rx_l3_bus.data[15:0];
						rx_state					<= RX_STATE_SEQ_HEADER;

						//At this point we have all of the info we need to look up the socket and see if
						//this is an active session.
						lookup_en					<= 1;
						lookup_headers.address		<= rx_l3_bus.src_ip;
						lookup_headers.client_port	<= rx_l3_bus.data[31:16];
						lookup_headers.server_port	<= rx_l3_bus.data[15:0];

						//See if the port is open as well.
						port_check_en				<= 1;
						port_check_num				<= rx_l3_bus.data[15:0];

					end

				end
			end	//end RX_STATE_PORT_HEADER

			RX_STATE_SEQ_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					else begin
						rx_current_seq	<= rx_l3_bus.data;
						rx_state		<= RX_STATE_ACK_HEADER;
					end

				end
			end	//end RX_STATE_SEQ_HEADER

			RX_STATE_ACK_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					else begin
						rx_current_ack	<= rx_l3_bus.data;
						rx_state		<= RX_STATE_FLAG_HEADER;
					end

				end
			end	//end RX_STATE_ACK_HEADER

			RX_STATE_FLAG_HEADER: begin
				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					else begin

						rx_current_window	<= rx_l3_bus.data[15:0];
						rx_data_offset		<= rx_l3_bus.data[31:28];

						//27:25 reserved
						//24 = NS (not supported)
						//23 = CWR (not supported)
						//22 = ECE (not supported)
						//21 = URG (TODO)
						rx_flag_ack			<= rx_l3_bus.data[20];
						//19 = PSH (ignored, we always push)
						rx_flag_rst			<= rx_l3_bus.data[18];
						rx_flag_syn			<= rx_l3_bus.data[17];
						rx_flag_fin			<= rx_l3_bus.data[16];

						rx_state			<= RX_STATE_CHECKSUM_HEADER;
					end

				end
			end	//end RX_STATE_FLAG_HEADER

			RX_STATE_CHECKSUM_HEADER: begin

				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					//valid data
					else begin

						//current data is checksum and urgent pointer, we don't care about either
						//so just ignore them

						//No options, jump right into data
						if(rx_data_offset <= 5)
							rx_state	<= RX_STATE_DATA;

						//We have options
						else begin
							rx_state			<= RX_STATE_OPTION_HEADER;
							rx_current_option	<= 6;
						end

					end
				end
			end	//end RX_STATE_CHECKSUM_HEADER

			RX_STATE_OPTION_HEADER: begin

				if(rx_l3_bus.data_valid) begin

					//Drop truncated packets
					if(rx_l3_bus.bytes_valid != 4)
						rx_state		<= RX_STATE_IDLE;

					//Keep count of options
					else begin
						rx_current_option	<= rx_current_option + 1;

						if(rx_current_option == rx_data_offset)
							rx_state	<= RX_STATE_DATA;
					end

				end

			end	//end RX_STATE_OPTION_HEADER

			RX_STATE_DATA: begin

				//TODO: If the ACK number of this packet is too high, we're missing stuff that came in between.
				//Drop this packet.

				if(rx_l3_bus.commit)
					rx_state		<= RX_STATE_CHECKSUM;
			end	//end RX_STATE_DATA

			//Verify checksum
			RX_STATE_CHECKSUM: begin

				if(rx_checksum_expected == 16'h0000) begin

					//TODO: commit data if any
					//rx_l4_bus.commit	<= 1;

					//SYN with no ACK: open new socket
					if(rx_flag_syn && !rx_flag_ack) begin

						//Get ready to send a reply packet
						wr_event.remote_ip		<= rx_l3_bus.src_ip;
						wr_event.remote_port	<= lookup_headers.client_port;
						wr_event.our_port		<= lookup_headers.server_port;
						wr_event.flag_syn		<= 0;
						wr_event.flag_ack		<= 0;
						wr_event.flag_fin		<= 0;
						wr_event.flag_rst		<= 0;
						wr_event.seq			<= next_seq;
						wr_event.ack			<= rx_current_seq + 1;

						//If the port is closed, send an immediate RST
						if(!port_check_is_open) begin
							wr_event.flag_rst	<= 1;
							wr_event.flag_ack	<= 1;
							event_wr_en			<= 1;
							rx_state			<= RX_STATE_IDLE;
						end

						//It's open. Create a new socket for this connection
						else begin
							insert_en			<= 1;
							insert_headers		<= lookup_headers;
							rx_state			<= RX_STATE_CREATE_SOCKET;
						end

					end

					//ACK, packet is half open? We're now fully open
					else if(rx_flag_ack && (state_rd_data.state == TCP_STATE_HALF_OPEN) ) begin

						state_wr_en				<= 1;
						state_wr_addr			<= lookup_sockid;
						state_wr_data.tx_seq	<= state_rd_data.tx_seq + 1;
						state_wr_data.rx_seq	<= rx_current_seq;
						state_wr_data.tx_window	<= state_rd_data.tx_window;
						state_wr_data.state		<= TCP_STATE_OPEN;

						rx_state				<= RX_STATE_IDLE;

					end

					//FIN, other end closed the socket.
					//Close the socket and send a FIN+ACK.
					//TODO: if we have queued transmit data or un-ACKed transmit traffic, send that first.
					else if(rx_flag_fin) begin

						//Send a reply packet
						wr_event.remote_ip		<= rx_l3_bus.src_ip;
						wr_event.remote_port	<= lookup_headers.client_port;
						wr_event.our_port		<= lookup_headers.server_port;
						wr_event.flag_syn		<= 0;
						wr_event.flag_ack		<= 1;
						wr_event.flag_fin		<= 1;
						wr_event.flag_rst		<= 0;
						wr_event.seq			<= state_rd_data.tx_seq;
						wr_event.ack			<= rx_current_seq + 1;
						event_wr_en				<= 1;

						//Mark the socket as closed
						state_wr_en				<= 1;
						state_wr_addr			<= lookup_sockid;
						state_wr_data.tx_seq	<= state_rd_data.tx_seq;
						state_wr_data.rx_seq	<= rx_current_seq + 1;
						state_wr_data.tx_window	<= state_rd_data.tx_window;
						state_wr_data.state		<= TCP_STATE_CLOSED;

						rx_state				<= RX_STATE_IDLE;

					end

					//RST, other end aborted the socket.
					//Close the socket without sending any traffic.
					//TODO: if we have un-ACK'd transmit data, discard it
					else if(rx_flag_rst) begin

						state_wr_en				<= 1;
						state_wr_addr			<= lookup_sockid;
						state_wr_data.tx_seq	<= state_rd_data.tx_seq;
						state_wr_data.rx_seq	<= rx_current_seq + 1;
						state_wr_data.tx_window	<= state_rd_data.tx_window;
						state_wr_data.state		<= TCP_STATE_CLOSED;

					end

					//Any other flags: discard for now
					else
						rx_state			<= RX_STATE_IDLE;

				end
				else begin
					rx_l4_bus.drop		<= 1;
					rx_state			<= RX_STATE_IDLE;
				end

			end	//end RX_STATE_CHECKSUM

			//Wait for a new socket table entry to be created
			RX_STATE_CREATE_SOCKET: begin

				if(insert_done) begin

					//Socket couldn't be created (not enough space in the table)
					//Send a RST
					if(insert_fail) begin
						wr_event.flag_syn	<= 0;
						wr_event.flag_rst	<= 1;
					end

					//Socket created successfully.
					//Send a SYN+ACK and write the state to the table
					else begin
						wr_event.flag_syn		<= 1;
						wr_event.flag_ack		<= 1;

						state_wr_en				<= 1;
						state_wr_addr			<= insert_sockid;
						state_wr_data.state		<= TCP_STATE_HALF_OPEN;
						state_wr_data.tx_seq	<= wr_event.seq;
						state_wr_data.rx_seq	<= wr_event.ack;
						state_wr_data.tx_window	<= 16'd1024;	//TODO: proper window size
					end

					event_wr_en				<= 1;
					rx_state				<= RX_STATE_IDLE;

				end

			end	//end RX_STATE_CREATE_SOCKET

		endcase

		//Handle drops
		if(rx_l3_bus.drop) begin
			rx_state		<= RX_STATE_IDLE;
			rx_l4_bus.drop	<= 1;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX checksum computation

	//TODO: we can precompute the IP pseudo-header for each socket, as it remains static

	logic		tx_checksum_reset	= 0;
	logic		tx_checksum_process	= 0;
	logic[31:0]	tx_checksum_din		= 0;
	wire[15:0]	tx_checksum;

	logic[2:0]	bytes_valid_adv		= 0;
	logic		data_valid_adv		= 0;

	InternetChecksum32bit tx_csum(
		.clk(clk),
		.load(1'b0),
		.reset(tx_checksum_reset),
		.process(tx_checksum_process),
		.din(tx_checksum_din),
		.sumout(),
		.csumout(tx_checksum)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TX side

	enum logic[3:0]
	{
		TX_STATE_IDLE					= 4'h0,
		TX_STATE_EVENT_POP				= 4'h1,
		TX_STATE_EVENT_HEADER_CHECKSUM	= 4'h2,
		TX_STATE_EVENT_PORT_HEADER		= 4'h3,
		TX_STATE_EVENT_SEQ				= 4'h4,
		TX_STATE_EVENT_ACK				= 4'h5,
		TX_STATE_EVENT_FLAGS			= 4'h6,
		TX_STATE_EVENT_CHECKSUM_1		= 4'h7,
		TX_STATE_EVENT_CHECKSUM_2		= 4'h8,
		TX_STATE_EVENT_COMMIT			= 4'h9
	} tx_state = TX_STATE_IDLE;

	logic[10:0]	tx_count;

	/*
		The TX logic has two jobs:

		1) Take events in rx_event_fifo, synthesize full TCP headers around them, and emit them as TCP segments
		2) Send application layer data (details TBD)
	 */
	always_ff @(posedge clk) begin

		event_rd_en			<= 0;
		tx_checksum_reset	<= 0;
		tx_checksum_process	<= 0;

		tx_l3_bus.start			<= 0;
		tx_l3_bus.data_valid	<= 0;
		tx_l3_bus.commit		<= 0;
		tx_l3_bus.drop			<= 0;

		bytes_valid_adv			<= 0;
		data_valid_adv			<= 0;

		//Pipeline the data
		tx_l3_bus.bytes_valid	<= bytes_valid_adv;
		tx_l3_bus.data_valid	<= data_valid_adv;
		tx_l3_bus.data			<= tx_checksum_din;

		case(tx_state)

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// IDLE - wait for events or transmit data

			TX_STATE_IDLE: begin

				if( (event_rsize > 1) || (event_rsize == 1 ** !event_rd_en) ) begin
					event_rd_en	<= 1;
					tx_state	<= TX_STATE_EVENT_POP;
				end

			end	//end TX_STATE_IDLE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// EVENT - send a header-only segment

			//POP - wait for the fifo to pop
			TX_STATE_EVENT_POP: begin
				if(!event_rd_en) begin
					tx_state			<= TX_STATE_EVENT_HEADER_CHECKSUM;
					tx_checksum_reset	<= 1;
					tx_count			<= 0;
				end
			end	//end TX_STATE_EVENT_POP

			//HEADER CHECKSUM - checksum the IP pseudo header
			TX_STATE_EVENT_HEADER_CHECKSUM: begin

				tx_checksum_process 	<= 1;
				tx_count				<= tx_count + 1;

				case(tx_count)

					0: 	tx_checksum_din	<= rd_event.remote_ip;
					1:	tx_checksum_din	<= ip_config.address;
					2: begin
						tx_checksum_din			<= {8'h0, IP_PROTO_TCP, 16'd20 };	//min size, no options
						tx_state				<= TX_STATE_EVENT_PORT_HEADER;
					end

				endcase

			end	//end TX_STATE_EVENT_HEADER_CHECKSUM

			//PORT_HEADER - send the port number header
			TX_STATE_EVENT_PORT_HEADER: begin

				//Start the outbound IP packet
				tx_l3_bus.start			<= 1;
				tx_l3_bus.payload_len	<= 16'd20;	//min size, no options
				tx_l3_bus.protocol		<= IP_PROTO_TCP;
				tx_l3_bus.dst_ip		<= rd_event.remote_ip;

				data_valid_adv					<= 1;
				bytes_valid_adv					<= 4;
				tx_checksum_process				<= 1;
				tx_checksum_din					<= { rd_event.our_port, rd_event.remote_port };
				tx_state						<= TX_STATE_EVENT_SEQ;
			end	//end TX_STATE_EVENT_PORT_HEADER

			TX_STATE_EVENT_SEQ: begin
				data_valid_adv					<= 1;
				bytes_valid_adv					<= 4;
				tx_checksum_process				<= 1;
				tx_checksum_din					<= rd_event.seq;
				tx_state						<= TX_STATE_EVENT_ACK;
			end	//end TX_STATE_EVENT_SEQ

			TX_STATE_EVENT_ACK: begin
				data_valid_adv					<= 1;
				bytes_valid_adv					<= 4;
				tx_checksum_process				<= 1;
				tx_checksum_din					<= rd_event.ack;
				tx_state						<= TX_STATE_EVENT_FLAGS;
			end	//end TX_STATE_EVENT_ACK

			TX_STATE_EVENT_FLAGS: begin
				data_valid_adv					<= 1;
				bytes_valid_adv					<= 4;
				tx_checksum_process				<= 1;
				tx_checksum_din					<=
				{
					4'd5,
					7'h0,
					rd_event.flag_ack,
					1'b0,
					rd_event.flag_rst,
					rd_event.flag_syn,
					rd_event.flag_fin,
					16'd1024					//TODO: proper window size!!!
				};
				tx_state						<= TX_STATE_EVENT_CHECKSUM_1;
			end	//end TX_STATE_EVENT_FLAGS

			TX_STATE_EVENT_CHECKSUM_1: begin
				//wait for checksum computation
				tx_state						<= TX_STATE_EVENT_CHECKSUM_2;
			end	//end TX_STATE_EVENT_CHECKSUM_1

			TX_STATE_EVENT_CHECKSUM_2: begin
				tx_l3_bus.data_valid			<= 1;
				tx_l3_bus.bytes_valid			<= 4;
				tx_l3_bus.data					<= { tx_checksum, 16'h0 };

				tx_state						<= TX_STATE_EVENT_COMMIT;
			end	//end TX_STATE_EVENT_CHECKSUM_2

			TX_STATE_EVENT_COMMIT: begin
				tx_l3_bus.commit				<= 1;
				tx_state						<= TX_STATE_IDLE;
			end	//end TX_STATE_EVENT_COMMIT

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug stuff

	ila_0 ila(
		.clk(clk),
		.probe0(rx_state),
		.probe1(rx_flag_ack),
		.probe2(rx_flag_rst),
		.probe3(rx_flag_syn),
		.probe4(rx_flag_fin),
		.probe5(event_wr_en),
		.probe6(wr_event),
		.probe7(lookup_en),
		.probe8(lookup_headers),
		.probe9(lookup_done),
		.probe10(lookup_hit),
		.probe11(lookup_sockid),
		.probe12(insert_en),
		.probe13(insert_headers),
		.probe14(insert_done),
		.probe15(insert_fail),
		.probe16(insert_sockid),
		.probe17(state_wr_en),
		.probe18(state_wr_addr),
		.probe19(state_wr_data),
		.probe20(state_rd_data),

		.probe21(port_check_en),
		.probe22(port_check_num),
		.probe23(port_check_is_open),

		.probe24(open_port_table[8'ha1]),
		.probe25(port_open_en),
		.probe26(port_num)
		);

endmodule
