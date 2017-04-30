/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright(c) 2012-2017 Andrew D. Zonenberg                                                                          *
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
*(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
*(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Andrew D. Zonenberg
	@brief Low-level wrapper for the Xilinx Zynq
 */
module XilinxZynq7CPU(

	//EMIO JTAG pins for the CPU
	//Useful for debugging the CPU from the FPGA or just breaking them out to GPIOs
	output wire cpu_jtag_tdo,
	input wire cpu_jtag_tdi,
	input wire cpu_jtag_tms,
	input wire cpu_jtag_tck,

	//Note that these pins are INPUTS.
	//The multiple-driver warning is a false positive (see AR#50430).
	//The __nowarn_528_ prefix will make Splash filter it out
	inout wire __nowarn_528_cpu_clk,
	inout wire __nowarn_528_cpu_por_n,
	inout wire __nowarn_528_cpu_srst_n
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Unused signals (tied off to default values now to avoid warnings, not yet brought out to top level)

	//Clocks from CPU PLL to FPGA, plus associated resets
	wire[3:0]	fabric_clock_unbuffered;
	wire[3:0]	async_cpu_reset_complete;

	//FPGA to CoreSight bridge signals
	wire[31:0]	cpu_to_fpga_debug_gpioreg;
	wire[31:0]	fpga_to_cpu_debug_gpioreg	= 32'h0;
	wire[3:0]	cpu_to_fpga_trace_trigger;
	wire[3:0]	cpu_to_fpga_trace_ack		= cpu_to_fpga_trace_trigger;
	wire[3:0]	fpga_to_cpu_trace_trigger	= 4'h0;
	wire[3:0]	fpga_to_cpu_trace_ack;
	wire		fpga_trace_clk				= 1'h0;
	wire		fpga_trace_valid			= 1'h0;
	wire[31:0]	fpga_trace_data				= 32'h0;
	wire[3:0]	fpga_trace_id				= 4'h0;

	//CPU trace signals
	wire		trace_clk			= 1'h0;
	wire		trace_ctl;
	wire[31:0]	trace_data;

	//System watchdog timer
	wire		watchdog_clk		= 1'h0;
	wire		watchdog_reset_out;

	//CPU events
	wire		cpu_event_req		= 1'h0;
	wire		cpu_event_ack;
	wire[1:0]	cpu_waiting_for_event;
	wire[1:0]	cpu_waiting_for_irq;

	//CPU interrupts
	wire[15:0]	fpga_to_cpu_irq		= 16'h0;
	wire[1:0]	cpu_fiq_n			= 2'h3;
	wire[1:0]	cpu_irq_n			= 2'h3;
	wire[28:0]	cpu_to_fpga_irq;			//copies of IRQs from various peripherals
											/*{
												IRQ_P2F_DMAC_ABORT,
												IRQ_P2F_DMAC7,
												IRQ_P2F_DMAC6,
												IRQ_P2F_DMAC5,
												IRQ_P2F_DMAC4,
												IRQ_P2F_DMAC3,
												IRQ_P2F_DMAC2,
												IRQ_P2F_DMAC1,
												IRQ_P2F_DMAC0,
												IRQ_P2F_SMC,
												IRQ_P2F_QSPI,
												IRQ_P2F_CTI,
												IRQ_P2F_GPIO,
												IRQ_P2F_USB0,
												IRQ_P2F_ENET0,
												IRQ_P2F_ENET_WAKE0,
												IRQ_P2F_SDIO0,
												IRQ_P2F_I2C0,
												IRQ_P2F_SPI0,
												IRQ_P2F_UART0,
												IRQ_P2F_CAN0,
												IRQ_P2F_USB1,
												IRQ_P2F_ENET1,
												IRQ_P2F_ENET_WAKE1,
												IRQ_P2F_SDIO1,
												IRQ_P2F_I2C1,
												IRQ_P2F_SPI1,
												IRQ_P2F_UART1,
												IRQ_P2F_CAN1
											} */

	//DMA channels
	wire[3:0]	dma_rst_n;
	wire[3:0]	dma_clk				= 4'h0;
	wire[3:0]	dma_req_ready;
	wire[3:0]	dma_req_valid		= 4'h0;
	wire[7:0] 	dma_req_type		= 8'h0;
	wire[3:0]	dma_req_last		= 4'h0;
	wire[3:0]	dma_ack_ready		= 4'h0;
	wire[3:0]	dma_ack_valid;
	wire[7:0]	dma_ack_type;

	//CAN interfaces
	wire[1:0]	can_phy_rx			= 2'h0;
	wire[1:0]	can_phy_tx;

	//Ethernet interfaces: data/control
	wire[1:0]	eth_ext_int			= 2'h0;

	wire[1:0]	eth_gmii_txc		= 2'h0;		//we have to externally mux the tx clock!
	wire[1:0]	eth_gmii_tx_en;
	wire[1:0]	eth_gmii_tx_er;
	wire[15:0]	eth_gmii_txd;

	wire[1:0]	eth_gmii_rxc		= 2'h0;
	wire[1:0]	eth_gmii_rx_dv		= 2'h0;
	wire[1:0]	eth_gmii_rx_er		= 2'h0;
	wire[15:0]	eth_gmii_rxd		= 16'h0;
	wire[1:0]	eth_gmii_rx_col		= 2'h0;
	wire[1:0]	eth_gmii_rx_crs		= 2'h0;

	wire[1:0]	eth_mgmt_mdio_out;
	wire[1:0]	eth_mgmt_mdio_tris;
	wire[1:0]	eth_mgmt_mdio_in	= 2'h0;
	wire[1:0]	eth_mgmt_mdc;

	//Ethernet interfaces: PTP timestamping
	wire[1:0]	eth_ptp_rx_sof;
	wire[1:0]	eth_ptp_rx_ptp_delay_req;
	wire[1:0]	eth_ptp_rx_ptp_peer_delay;
	wire[1:0]	eth_ptp_rx_ptp_peer_delay_resp;
	wire[1:0]	eth_ptp_rx_ptp_sync;

	wire[1:0]	eth_ptp_tx_sof;
	wire[1:0]	eth_ptp_tx_ptp_delay_req;
	wire[1:0]	eth_ptp_tx_ptp_peer_delay;
	wire[1:0]	eth_ptp_tx_ptp_peer_delay_resp;
	wire[1:0]	eth_ptp_tx_ptp_sync;

	//GPIO interfaces
	wire[63:0]	gpio_in				= 64'h0;
	wire[63:0]	gpio_out;
	wire[63:0]	gpio_tris;

	//I2C interfaces
	wire[1:0]	i2c_scl_in			= 2'h0;
	wire[1:0]	i2c_scl_out;
	wire[1:0]	i2c_scl_tris;

	wire[1:0]	i2c_sda_in			= 2'h0;
	wire[1:0]	i2c_sda_out;
	wire[1:0]	i2c_sda_tris;

	//SD/SDIO interfaces
	wire[1:0]	sdio_clk;
	wire[1:0]	sdio_clkfb			= sdio_clk;
	wire[1:0]	sdio_cmd_in			= 2'h0;
	wire[1:0]	sdio_cmd_out;
	wire[1:0]	sdio_cmd_tris;
	wire[7:0]	sdio_data_in		= 8'h0;
	wire[7:0]	sdio_data_out;
	wire[7:0]	sdio_data_tris;
	wire[1:0]	sdio_card_detect	= 2'h0;
	wire[1:0]	sdio_write_protect	= 2'h0;
	wire[1:0]	sdio_power_en;
	wire[1:0]	sdio_led_en;
	wire[5:0]	sdio_bus_voltage;

	//SPI interfaces
	wire[1:0]	spi_clk_in			= 2'h0;
	wire[1:0]	spi_clk_out;
	wire[1:0]	spi_clk_tris;
	wire[1:0]	spi_mosi_in			= 2'h0;
	wire[1:0]	spi_mosi_out;
	wire[1:0]	spi_mosi_tris;
	wire[1:0]	spi_miso_in			= 2'h0;
	wire[1:0]	spi_miso_out;
	wire[1:0]	spi_miso_tris;
	wire[1:0]	spi_csn_in			= 2'h3;
	wire[5:0]	spi_csn_out;
	wire[1:0]	spi_csn_tris;

	//Triple timer/counters
	wire[5:0]	tmrcnt_clk			= 6'h0;
	wire[5:0]	tmrcnt_wave_out;

	//UARTs
	wire[1:0]	uart_txd;
	wire[1:0]	uart_rxd			= 2'h3;
	wire[1:0]	uart_cts			= 2'h0;
	wire[1:0]	uart_rts;
	wire[1:0]	uart_dsr			= 2'h0;
	wire[1:0]	uart_dcd			= 2'h0;
	wire[1:0]	uart_ri				= 2'h0;
	wire[1:0]	uart_dtr			= 2'h0;

	//USB status signals. The actual ULPI data lines are hard IP and not routed to fabric.
	wire[3:0]	usb_indicator;
	wire[1:0]	usb_pwr_fault		= 2'h0;
	wire[1:0]	usb_pwr_select;

	//AXI bus power saving stuff
	wire		fpga_axi_buses_idle	= 1'h0;

	//Master AXI links (transactions initiated by CPU, 32 bits)
	wire[1:0]	master_axi_clk			= 2'h0;
	wire[1:0]	master_axi_rst_n;
	wire[63:0]	master_axi_rdreq_addr;
	wire[1:0]	master_axi_rdreq_valid;
	wire[1:0]	master_axi_rdreq_ready	= 2'h0;
	wire[23:0]	master_axi_rdreq_id;
	wire[3:0]	master_axi_rdreq_lock;
	wire[7:0]	master_axi_rdreq_cache;
	wire[5:0]	master_axi_rdreq_prot;
	wire[7:0]	master_axi_rdreq_len;
	wire[3:0]	master_axi_rdreq_size;
	wire[3:0]	master_axi_rdreq_burst;
	wire[7:0]	master_axi_rdreq_qos;
	wire[63:0]	master_axi_rdresp_data	= 64'h0;
	wire[1:0]	master_axi_rdresp_valid	= 2'h0;
	wire[1:0]	master_axi_rdresp_ready;
	wire[23:0]	master_axi_rdresp_id	= 24'h0;
	wire[1:0]	master_axi_rdresp_last	= 2'h0;
	wire[3:0]	master_axi_rdresp_resp	= 2'h0;

	wire[63:0]	master_axi_wrreq_addr;
	wire[1:0]	master_axi_wrreq_valid;
	wire[1:0]	master_axi_wrreq_ready	= 2'h0;
	wire[23:0]	master_axi_wrreq_id;
	wire[3:0]	master_axi_wrreq_lock;
	wire[7:0]	master_axi_wrreq_cache;
	wire[5:0]	master_axi_wrreq_prot;
	wire[7:0]	master_axi_wrreq_len;
	wire[3:0]	master_axi_wrreq_size;
	wire[3:0]	master_axi_wrreq_burst;
	wire[7:0]	master_axi_wrreq_qos;
	wire[63:0]	master_axi_wrdat_data;
	wire[1:0]	master_axi_wrdat_valid;
	wire[1:0]	master_axi_wrdat_ready	= 2'h0;
	wire[23:0]	master_axi_wrdat_id;
	wire[1:0]	master_axi_wrdat_last;
	wire[7:0]	master_axi_wrdat_mask;
	wire[1:0]	master_axi_wrresp_valid	= 2'h0;
	wire[1:0]	master_axi_wrresp_ready;
	wire[23:0]	master_axi_wrresp_id	= 24'h0;
	wire[3:0]	master_axi_wrresp_resp	= 4'h0;

	//Slave AXI ACP links (transactions initiated by FPGA, cache coherent, 64 bits)
	wire		acp_axi_clk				= 1'h0;
	wire		acp_axi_rst_n;
	wire[31:0]	acp_axi_rdreq_addr		= 32'h0;
	wire		acp_axi_rdreq_valid		= 1'h0;
	wire		acp_axi_rdreq_ready;
	wire[2:0]	acp_axi_rdreq_id		= 3'h0;
	wire[4:0]	acp_axi_rdreq_user		= 5'h0;
	wire[1:0]	acp_axi_rdreq_lock		= 2'h0;
	wire[3:0]	acp_axi_rdreq_cache		= 4'h0;
	wire[2:0]	acp_axi_rdreq_prot		= 3'h0;
	wire[3:0]	acp_axi_rdreq_len		= 4'h0;
	wire[1:0]	acp_axi_rdreq_size		= 2'h0;
	wire[1:0]	acp_axi_rdreq_burst		= 2'h0;
	wire[3:0]	acp_axi_rdreq_qos		= 4'h0;
	wire[63:0]	acp_axi_rdresp_data;
	wire		acp_axi_rdresp_valid;
	wire		acp_axi_rdresp_ready	= 1'h0;
	wire[2:0]	acp_axi_rdresp_id		= 3'h0;
	wire		acp_axi_rdresp_last;
	wire[1:0]	acp_axi_rdresp_resp;

	wire[31:0]	acp_axi_wrreq_addr		= 32'h0;
	wire		acp_axi_wrreq_valid		= 1'h0;
	wire		acp_axi_wrreq_ready;
	wire[2:0]	acp_axi_wrreq_id		= 3'h0;
	wire[4:0]	acp_axi_wrreq_user		= 5'h0;
	wire[1:0]	acp_axi_wrreq_lock		= 2'h0;
	wire[3:0]	acp_axi_wrreq_cache		= 4'h0;
	wire[2:0]	acp_axi_wrreq_prot		= 3'h0;
	wire[3:0]	acp_axi_wrreq_len		= 4'h0;
	wire[1:0]	acp_axi_wrreq_size		= 2'h0;
	wire[1:0]	acp_axi_wrreq_burst		= 2'h0;
	wire[3:0]	acp_axi_wrreq_qos		= 4'h0;
	wire[63:0]	acp_axi_wrdat_data		= 64'h0;
	wire		acp_axi_wrdat_valid		= 1'h0;
	wire		acp_axi_wrdat_ready		= 1'h0;
	wire[2:0]	acp_axi_wrdat_id		= 3'h0;
	wire		acp_axi_wrdat_last		= 1'h0;
	wire[7:0]	acp_axi_wrdat_mask		= 8'h0;
	wire		acp_axi_wrresp_valid;
	wire		acp_axi_wrresp_ready	= 1'h0;
	wire[2:0]	acp_axi_wrresp_id;
	wire[1:0]	acp_axi_wrresp_resp;

	//Slave AXI general purpose links (transactions initiated by FPGA, no buffering, 32 bits)
	wire[1:0]	slavegp_axi_clk				= 2'h0;
	wire[1:0]	slavegp_axi_rst_n;
	wire[63:0]	slavegp_axi_rdreq_addr		= 64'h0;
	wire[1:0]	slavegp_axi_rdreq_valid		= 2'h0;
	wire[1:0]	slavegp_axi_rdreq_ready;
	wire[11:0]	slavegp_axi_rdreq_id		= 12'h0;
	wire[3:0]	slavegp_axi_rdreq_lock		= 4'h0;
	wire[7:0]	slavegp_axi_rdreq_cache		= 8'h0;
	wire[5:0]	slavegp_axi_rdreq_prot		= 6'h0;
	wire[7:0]	slavegp_axi_rdreq_len		= 8'h0;
	wire[3:0]	slavegp_axi_rdreq_size		= 4'h0;
	wire[3:0]	slavegp_axi_rdreq_burst		= 4'h0;
	wire[7:0]	slavegp_axi_rdreq_qos		= 8'h0;
	wire[63:0]	slavegp_axi_rdresp_data;
	wire[1:0]	slavegp_axi_rdresp_valid;
	wire[1:0]	slavegp_axi_rdresp_ready	= 2'h0;
	wire[11:0]	slavegp_axi_rdresp_id;
	wire[1:0]	slavegp_axi_rdresp_last;
	wire[3:0]	slavegp_axi_rdresp_resp;

	wire[63:0]	slavegp_axi_wrreq_addr		= 64'h0;
	wire[1:0]	slavegp_axi_wrreq_valid		= 2'h0;
	wire[1:0]	slavegp_axi_wrreq_ready;
	wire[11:0]	slavegp_axi_wrreq_id		= 12'h0;
	wire[3:0]	slavegp_axi_wrreq_lock		= 4'h0;
	wire[7:0]	slavegp_axi_wrreq_cache		= 8'h0;
	wire[5:0]	slavegp_axi_wrreq_prot		= 6'h0;
	wire[7:0]	slavegp_axi_wrreq_len		= 8'h0;
	wire[3:0]	slavegp_axi_wrreq_size		= 4'h0;
	wire[3:0]	slavegp_axi_wrreq_burst		= 4'h0;
	wire[7:0]	slavegp_axi_wrreq_qos		= 8'h0;
	wire[63:0]	slavegp_axi_wrdat_data		= 64'h0;
	wire[1:0]	slavegp_axi_wrdat_valid		= 2'h0;
	wire[1:0]	slavegp_axi_wrdat_ready;
	wire[23:0]	slavegp_axi_wrdat_id		= 24'h0;
	wire[1:0]	slavegp_axi_wrdat_last		= 2'h0;
	wire[7:0]	slavegp_axi_wrdat_mask		= 8'h0;
	wire[1:0]	slavegp_axi_wrresp_valid;
	wire[1:0]	slavegp_axi_wrresp_ready	= 2'h0;
	wire[11:0]	slavegp_axi_wrresp_id;
	wire[3:0]	slavegp_axi_wrresp_resp;

	//Slave AXI high performance links (transactions initiated by FPGA, fifo buffering, 64 bits)
	wire[3:0]	slavehp_axi_clk				= 4'h0;
	wire[3:0]	slavehp_axi_rst_n;
	wire[127:0]	slavehp_axi_rdreq_addr		= 128'h0;
	wire[23:0]	slavehp_axi_rdreq_fifosize;
	wire[3:0]	slavehp_axi_rdreq_valid		= 4'h0;
	wire[3:0]	slavehp_axi_rdreq_ready;
	wire[23:0]	slavehp_axi_rdreq_id		= 24'h0;
	wire[7:0]	slavehp_axi_rdreq_lock		= 8'h0;
	wire[15:0]	slavehp_axi_rdreq_cache		= 16'h0;
	wire[11:0]	slavehp_axi_rdreq_prot		= 12'h0;
	wire[15:0]	slavehp_axi_rdreq_len		= 16'h0;
	wire[7:0]	slavehp_axi_rdreq_size		= 8'h0;
	wire[7:0]	slavehp_axi_rdreq_burst		= 8'h0;
	wire[15:0]	slavehp_axi_rdreq_qos		= 16'h0;
	wire[255:0]	slavehp_axi_rdresp_data;
	wire[31:0]	slavehp_axi_rdresp_fifosize;
	wire[3:0]	slavehp_axi_rdresp_valid;
	wire[3:0]	slavehp_axi_rdresp_ready	= 4'h0;
	wire[23:0]	slavehp_axi_rdresp_id;
	wire[3:0]	slavehp_axi_rdresp_last;
	wire[7:0]	slavehp_axi_rdresp_resp;
	wire[3:0]	slavehp_axi_rdissuecap1en	= 4'h0;		//wut is this?

	wire[127:0]	slavehp_axi_wrreq_addr		= 128'h0;
	wire[23:0]	slavehp_axi_wrreq_fifosize;
	wire[3:0]	slavehp_axi_wrreq_valid		= 4'h0;
	wire[3:0]	slavehp_axi_wrreq_ready;
	wire[23:0]	slavehp_axi_wrreq_id		= 24'h0;
	wire[7:0]	slavehp_axi_wrreq_lock		= 8'h0;
	wire[15:0]	slavehp_axi_wrreq_cache		= 16'h0;
	wire[11:0]	slavehp_axi_wrreq_prot		= 12'h0;
	wire[15:0]	slavehp_axi_wrreq_len		= 16'h0;
	wire[7:0]	slavehp_axi_wrreq_size		= 8'h0;
	wire[7:0]	slavehp_axi_wrreq_burst		= 8'h0;
	wire[15:0]	slavehp_axi_wrreq_qos		= 16'h0;
	wire[255:0]	slavehp_axi_wrdat_data		= 256'h0;
	wire[31:0]	slavehp_axi_wrdat_fifosize;
	wire[3:0]	slavehp_axi_wrdat_valid		= 4'h0;
	wire[3:0]	slavehp_axi_wrdat_ready;
	wire[23:0]	slavehp_axi_wrdat_id		= 24'h0;
	wire[3:0]	slavehp_axi_wrdat_last		= 4'h0;
	wire[31:0]	slavehp_axi_wrdat_mask		= 32'h0;
	wire[3:0]	slavehp_axi_wrresp_valid;
	wire[3:0]	slavehp_axi_wrresp_ready	= 4'h0;
	wire[23:0]	slavehp_axi_wrresp_id;
	wire[7:0]	slavehp_axi_wrresp_resp;
	wire[3:0]	slavehp_axi_wrissuecap1en	= 4'h0;		//wut is this?

	//DDR RAM signals (TODO: break these out to top level and hook them up)
	wire[14:0]	ddr_addr;
	wire[2:0]	ddr_bankaddr;
	wire		ddr_cas_n;
	wire		ddr_cke;
	wire		ddr_clk_n;
	wire		ddr_clk_p;
	wire		ddr_cs_n;
	wire[3:0]	ddr_dm;
	wire[31:0]	ddr_dq;
	wire[3:0]	ddr_dqs_n;
	wire[3:0]	ddr_dqs_p;
	wire		ddr_reset_n;
	wire		ddr_odt;
	wire		ddr_ras_n;
	wire		ddr_vr_n;
	wire		ddr_vr_p;
	wire		ddr_we_n;

	//Muxed GPIO signals
	wire[53:0]	muxed_gpio;

	//IRQ from external SRAM
	wire		sram_irq = 1'h0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual CPU

	PS7 cpu(

		//CPU clock and reset
		.PSCLK(__nowarn_528_cpu_clk),
		.PSPORB(__nowarn_528_cpu_por_n),
		.PSSRSTB(__nowarn_528_cpu_srst_n),

		//CPU JTAG
		.EMIOPJTAGTDO(cpu_jtag_tdo),
		.EMIOPJTAGTDTN(),				//JTAG TDO tristate enable (not used for now)
		.EMIOPJTAGTCK(cpu_jtag_tck),
		.EMIOPJTAGTDI(cpu_jtag_tdi),
		.EMIOPJTAGTMS(cpu_jtag_tms),

		//Clocks from CPU to FPGA
		.FCLKCLK(fabric_clock_unbuffered),
		.FCLKRESETN(async_cpu_reset_complete),
		.FCLKCLKTRIGN(4'h0),						//Unimplemented, must be tied to ground per Zynq TRM 2.7.1

		//AXI bus power management
		.FPGAIDLEN(fpga_axi_buses_idle),

		//Fabric Trace Monitor (FPGA to CoreSight bridge)
		.FTMTF2PDEBUG(fpga_to_cpu_debug_gpioreg),
		.FTMTP2FDEBUG(cpu_to_fpga_debug_gpioreg),
		.FTMTF2PTRIG(fpga_to_cpu_trace_trigger),
		.FTMTF2PTRIGACK(fpga_to_cpu_trace_ack),
		.FTMTP2FTRIG(cpu_to_fpga_trace_trigger),
		.FTMTP2FTRIGACK(cpu_to_fpga_trace_ack),
		.FTMDTRACEINVALID(fpga_trace_valid),
		.FTMDTRACEINCLOCK(fpga_trace_clk),
		.FTMDTRACEINDATA(fpga_trace_data),
		.FTMDTRACEINATID(fpga_trace_id),

		//EMIO trace port
		.EMIOTRACECLK(trace_clk),
		.EMIOTRACECTL(trace_ctl),
		.EMIOTRACEDATA(trace_data),

		//Watchdog timer (not yet used)
		.EMIOWDTRSTO(watchdog_reset_out),
		.EMIOWDTCLKI(watchdog_clk),

		//Event handling
		.EVENTEVENTI(cpu_event_req),
		.EVENTEVENTO(cpu_event_ack),
		.EVENTSTANDBYWFE(cpu_waiting_for_event),
		.EVENTSTANDBYWFI(cpu_waiting_for_irq),

		//Interrupts
		//NOTE: Zynq TRM 2.7.2 specifies "both CPUs" for nFIQ, nIRQ as bits 19:16
		//but does not specify the ordering within those four bits!
		.IRQF2P({cpu_fiq_n, cpu_irq_n, fpga_to_cpu_irq}),
		.IRQP2F(cpu_to_fpga_irq),

		//Muxed I/O signals from hard IP
		.MIO(muxed_gpio),

		//SRAM external interrupt signal
		.EMIOSRAMINTIN(sram_irq),

		//DDR RAM
		.DDRDRSTB(ddr_reset_n),
		.DDRCKE(ddr_cke),
		.DDRODT(ddr_odt),
		.DDRCSB(ddr_cs_n),
		.DDRVRP(ddr_vr_p),
		.DDRVRN(ddr_vr_n),
		.DDRCKP(ddr_clk_p),
		.DDRCKN(ddr_clk_n),
		.DDRA(ddr_addr),
		.DDRBA(ddr_bankaddr),
		.DDRARB(),			//what does this do? doesn't seem to be doc'd anywhere!
		.DDRCASB(ddr_cas_n),
		.DDRRASB(ddr_ras_n),
		.DDRWEB(ddr_we_n),
		.DDRDM(ddr_dm),
		.DDRDQ(ddr_dq),
		.DDRDQSP(ddr_dqs_p),
		.DDRDQSN(ddr_dqs_n),

		//CPU DMA ports
		.DMA0ACLK(dma_clk[0]),
		.DMA0DRREADY(dma_req_ready[0]),
		.DMA0DRVALID(dma_req_valid[0]),
		.DMA0DRTYPE(dma_req_type[0*2 +: 2]),
		.DMA0DRLAST(dma_req_last[0]),
		.DMA0DAREADY(dma_ack_ready[0]),
		.DMA0DAVALID(dma_ack_valid[0]),
		.DMA0DATYPE(dma_ack_type[0*2 +: 2]),
		.DMA0RSTN(dma_rst_n[0]),

		.DMA1ACLK(dma_clk[1]),
		.DMA1DRREADY(dma_req_ready[1]),
		.DMA1DRVALID(dma_req_valid[1]),
		.DMA1DRTYPE(dma_req_type[1*2 +: 2]),
		.DMA1DRLAST(dma_req_last[1]),
		.DMA1DAREADY(dma_ack_ready[1]),
		.DMA1DAVALID(dma_ack_valid[1]),
		.DMA1DATYPE(dma_ack_type[1*2 +: 2]),
		.DMA1RSTN(dma_rst_n[1]),

		.DMA2ACLK(dma_clk[2]),
		.DMA2DRREADY(dma_req_ready[2]),
		.DMA2DRVALID(dma_req_valid[2]),
		.DMA2DRTYPE(dma_req_type[2*2 +: 2]),
		.DMA2DRLAST(dma_req_last[2]),
		.DMA2DAREADY(dma_ack_ready[2]),
		.DMA2DAVALID(dma_ack_valid[2]),
		.DMA2DATYPE(dma_ack_type[2*2 +: 2]),
		.DMA2RSTN(dma_rst_n[2]),

		.DMA3ACLK(dma_clk[3]),
		.DMA3DRREADY(dma_req_ready[3]),
		.DMA3DRVALID(dma_req_valid[3]),
		.DMA3DRTYPE(dma_req_type[3*2 +: 2]),
		.DMA3DRLAST(dma_req_last[3]),
		.DMA3DAREADY(dma_ack_ready[3]),
		.DMA3DAVALID(dma_ack_valid[3]),
		.DMA3DATYPE(dma_ack_type[3*2 +: 2]),
		.DMA3RSTN(dma_rst_n[3]),

		//EMIO CAN
		.EMIOCAN0PHYTX(can_phy_tx[0]),
		.EMIOCAN0PHYRX(can_phy_rx[0]),
		.EMIOCAN1PHYTX(can_phy_tx[1]),
		.EMIOCAN1PHYRX(can_phy_rx[1]),

		//EMIO Ethernet 0
		.EMIOENET0MDIOMDC(eth_mgmt_mdc[0]),
		.EMIOENET0MDIOO(eth_mgmt_mdio_out[0]  ),
		.EMIOENET0MDIOTN(eth_mgmt_mdio_tris[0]  ),
		.EMIOENET0MDIOI(eth_mgmt_mdio_in[0]),

		.EMIOENET0GMIITXCLK(eth_gmii_txc[0]),
		.EMIOENET0GMIITXD(eth_gmii_txd[7:0] ),
		.EMIOENET0GMIITXEN(eth_gmii_tx_en[0]),
		.EMIOENET0GMIITXER(eth_gmii_tx_er[0]),

		.EMIOENET0GMIIRXCLK(eth_gmii_rxc[0]),
		.EMIOENET0GMIICOL(eth_gmii_rx_col[0]),
		.EMIOENET0GMIICRS(eth_gmii_rx_crs[0]),
		.EMIOENET0GMIIRXER(eth_gmii_rx_er[0]),
		.EMIOENET0GMIIRXDV(eth_gmii_rx_dv[0]),
		.EMIOENET0GMIIRXD(eth_gmii_rxd[7:0]),

		.EMIOENET0SOFRX(eth_ptp_rx_sof[0]),
		.EMIOENET0PTPDELAYREQRX(eth_ptp_rx_ptp_delay_req[0]),
		.EMIOENET0PTPPDELAYREQRX(eth_ptp_rx_ptp_peer_delay[0]),
		.EMIOENET0PTPPDELAYRESPRX(eth_ptp_rx_ptp_peer_delay_resp[0]),
		.EMIOENET0PTPSYNCFRAMERX(eth_ptp_rx_ptp_sync[0]),

		.EMIOENET0SOFTX(eth_ptp_tx_sof[0]),
		.EMIOENET0PTPDELAYREQTX(eth_ptp_tx_ptp_delay_req[0]),
		.EMIOENET0PTPPDELAYREQTX(eth_ptp_tx_ptp_peer_delay[0]),
		.EMIOENET0PTPPDELAYRESPTX(eth_ptp_tx_ptp_peer_delay_resp[0]),
		.EMIOENET0PTPSYNCFRAMETX(eth_ptp_tx_ptp_sync[0]),

		.EMIOENET0EXTINTIN(eth_ext_int[0]),

		//EMIO Ethernet 1
		.EMIOENET1MDIOMDC(eth_mgmt_mdc[1]),
		.EMIOENET1MDIOO(eth_mgmt_mdio_out[1]  ),
		.EMIOENET1MDIOTN(eth_mgmt_mdio_tris[1]  ),
		.EMIOENET1MDIOI(eth_mgmt_mdio_in[1]),

		.EMIOENET1GMIITXCLK(eth_gmii_txc[1]),
		.EMIOENET1GMIITXD(eth_gmii_txd[15:8] ),
		.EMIOENET1GMIITXEN(eth_gmii_tx_en[1]),
		.EMIOENET1GMIITXER(eth_gmii_tx_er[1]),

		.EMIOENET1GMIIRXCLK(eth_gmii_rxc[1]),
		.EMIOENET1GMIICOL(eth_gmii_rx_col[1]),
		.EMIOENET1GMIICRS(eth_gmii_rx_crs[1]),
		.EMIOENET1GMIIRXER(eth_gmii_rx_er[1]),
		.EMIOENET1GMIIRXDV(eth_gmii_rx_dv[1]),
		.EMIOENET1GMIIRXD(eth_gmii_rxd[15:8]),

		.EMIOENET1SOFRX(eth_ptp_rx_sof[1]),
		.EMIOENET1PTPDELAYREQRX(eth_ptp_rx_ptp_delay_req[1]),
		.EMIOENET1PTPPDELAYREQRX(eth_ptp_rx_ptp_peer_delay[1]),
		.EMIOENET1PTPPDELAYRESPRX(eth_ptp_rx_ptp_peer_delay_resp[1]),
		.EMIOENET1PTPSYNCFRAMERX(eth_ptp_rx_ptp_sync[1]),

		.EMIOENET1SOFTX(eth_ptp_tx_sof[1]),
		.EMIOENET1PTPDELAYREQTX(eth_ptp_tx_ptp_delay_req[1]),
		.EMIOENET1PTPPDELAYREQTX(eth_ptp_tx_ptp_peer_delay[1]),
		.EMIOENET1PTPPDELAYRESPTX(eth_ptp_tx_ptp_peer_delay_resp[1]),
		.EMIOENET1PTPSYNCFRAMETX(eth_ptp_tx_ptp_sync[1]),

		.EMIOENET1EXTINTIN(eth_ext_int[1]),

		//EMIO GPIO
		.EMIOGPIOO(gpio_out),
		.EMIOGPIOTN(gpio_tris),
		.EMIOGPIOI(gpio_in),

		//EMIO I2C 0
		.EMIOI2C0SCLO(i2c_scl_out[0]),
		.EMIOI2C0SCLTN(i2c_scl_tris[0]),
		.EMIOI2C0SCLI(i2c_scl_in[0]),

		.EMIOI2C0SDAO(i2c_sda_out[0]),
		.EMIOI2C0SDATN(i2c_sda_tris[0]),
		.EMIOI2C0SDAI(i2c_sda_in[0]),

		//EMIO I2C 1
		.EMIOI2C1SCLO(i2c_scl_out[1]),
		.EMIOI2C1SCLTN(i2c_scl_tris[1]),
		.EMIOI2C1SCLI(i2c_scl_in[1]),

		.EMIOI2C1SDAO(i2c_sda_out[1]),
		.EMIOI2C1SDATN(i2c_sda_tris[1]),
		.EMIOI2C1SDAI(i2c_sda_in[1]),

		//EMIO SD 0
		.EMIOSDIO0CLK(sdio_clk[0]),
		.EMIOSDIO0CLKFB(sdio_clkfb[0]),
		.EMIOSDIO0CMDO(sdio_cmd_out[0]),
		.EMIOSDIO0CMDTN(sdio_cmd_tris[0]),
		.EMIOSDIO0CMDI(sdio_cmd_in[0]),
		.EMIOSDIO0DATAO(sdio_data_out[3:0]),
		.EMIOSDIO0DATATN(sdio_data_tris[3:0]),
		.EMIOSDIO0DATAI(sdio_data_in[3:0]),
		.EMIOSDIO0BUSPOW(sdio_power_en[0]),
		.EMIOSDIO0LED(sdio_led_en[0]),
		.EMIOSDIO0BUSVOLT(sdio_bus_voltage[2:0]),
		.EMIOSDIO0CDN(sdio_card_detect[0]),
		.EMIOSDIO0WP(sdio_write_protect[0]),

		//EMIO SD 1
		.EMIOSDIO1CLK(sdio_clk[1]),
		.EMIOSDIO1CLKFB(sdio_clkfb[1]),
		.EMIOSDIO1CMDO(sdio_cmd_out[1]),
		.EMIOSDIO1CMDTN(sdio_cmd_tris[1]),
		.EMIOSDIO1CMDI(sdio_cmd_in[1]),
		.EMIOSDIO1DATAO(sdio_data_out[7:4]),
		.EMIOSDIO1DATATN(sdio_data_tris[7:4]),
		.EMIOSDIO1DATAI(sdio_data_in[7:4]),
		.EMIOSDIO1BUSPOW(sdio_power_en[1]),
		.EMIOSDIO1LED(sdio_led_en[1]),
		.EMIOSDIO1BUSVOLT(sdio_bus_voltage[5:3]),
		.EMIOSDIO1CDN(sdio_card_detect[1]),
		.EMIOSDIO1WP(sdio_write_protect[1]),

		//EMIO SPI 0 (not yet used)
		.EMIOSPI0SCLKO(spi_clk_out[0]),
		.EMIOSPI0SCLKTN(spi_clk_tris[0]),
		.EMIOSPI0SCLKI(spi_clk_in[0]),
		.EMIOSPI0MO(spi_mosi_out[0]),
		.EMIOSPI0MOTN(spi_mosi_tris[0]),
		.EMIOSPI0SI(spi_mosi_in[0]),
		.EMIOSPI0SO(spi_miso_out[0]),
		.EMIOSPI0STN(spi_miso_tris[0]),
		.EMIOSPI0MI(spi_miso_in[0]),
		.EMIOSPI0SSON(spi_csn_out[2:0]),
		.EMIOSPI0SSNTN(spi_csn_tris[0]),
		.EMIOSPI0SSIN(spi_csn_in[0]),

		//EMIO SPI 1 (not yet used)
		.EMIOSPI1SCLKO(spi_clk_out[1]),
		.EMIOSPI1SCLKTN(spi_clk_tris[1]),
		.EMIOSPI1SCLKI(spi_clk_in[1]),
		.EMIOSPI1MO(spi_mosi_out[1]),
		.EMIOSPI1MOTN(spi_mosi_tris[1]),
		.EMIOSPI1SI(spi_mosi_in[1]),
		.EMIOSPI1SO(spi_miso_out[1]),
		.EMIOSPI1STN(spi_miso_tris[1]),
		.EMIOSPI1MI(spi_miso_in[1]),
		.EMIOSPI1SSON(spi_csn_out[5:3]),
		.EMIOSPI1SSNTN(spi_csn_tris[1]),
		.EMIOSPI1SSIN(spi_csn_in[1]),

		//Triple timer/counters
		.EMIOTTC0CLKI(tmrcnt_clk[2:0]),
		.EMIOTTC0WAVEO(tmrcnt_wave_out[2:0]),
		.EMIOTTC1CLKI(tmrcnt_clk[5:3]),
		.EMIOTTC1WAVEO(tmrcnt_wave_out[2:0]),

		//EMIO UART 0
		.EMIOUART0TX(uart_txd[0]),
		.EMIOUART0RX(uart_rxd[0]),
		.EMIOUART0DTRN(uart_dtr[0]),
		.EMIOUART0DSRN(uart_dsr[0]),
		.EMIOUART0RTSN(uart_rts[0]),
		.EMIOUART0DCDN(uart_dcd[0]),
		.EMIOUART0CTSN(uart_cts[0]),
		.EMIOUART0RIN(uart_ri[0]),

		//EMIO UART 1
		.EMIOUART1TX(uart_txd[1]),
		.EMIOUART1RX(uart_rxd[1]),
		.EMIOUART1DTRN(uart_dtr[1]),
		.EMIOUART1DSRN(uart_dsr[1]),
		.EMIOUART1RTSN(uart_rts[1]),
		.EMIOUART1DCDN(uart_dcd[1]),
		.EMIOUART1CTSN(uart_cts[1]),
		.EMIOUART1RIN(uart_ri[1]),

		//EMIO USB 0
		.EMIOUSB0PORTINDCTL(usb_indicator[1:0]),
		.EMIOUSB0VBUSPWRSELECT(usb_pwr_select[0]),
		.EMIOUSB0VBUSPWRFAULT(usb_pwr_fault[0]),

		//EMIO USB 1
		.EMIOUSB1PORTINDCTL(usb_indicator[3:2]),
		.EMIOUSB1VBUSPWRSELECT(usb_pwr_select[1]),
		.EMIOUSB1VBUSPWRFAULT(usb_pwr_fault[1]),

		//Master AXI GP 0
		.MAXIGP0ACLK(master_axi_clk[0]),
		.MAXIGP0ARESETN(master_axi_rst_n[0]),
		.MAXIGP0ARADDR(master_axi_rdreq_addr[31:0]),
		.MAXIGP0ARVALID(master_axi_rdreq_valid[0]),
		.MAXIGP0ARREADY(master_axi_rdreq_ready[0]),
		.MAXIGP0ARID(master_axi_rdreq_id[11:0]),
		.MAXIGP0ARLOCK(master_axi_rdreq_lock[1:0]),
		.MAXIGP0ARCACHE(master_axi_rdreq_cache[3:0]),
		.MAXIGP0ARPROT(master_axi_rdreq_prot[2:0]),
		.MAXIGP0ARLEN(master_axi_rdreq_len[3:0]),
		.MAXIGP0ARSIZE(master_axi_rdreq_size[1:0]),
		.MAXIGP0ARBURST(master_axi_rdreq_burst[1:0]),
		.MAXIGP0ARQOS(master_axi_rdreq_qos[3:0]),
		.MAXIGP0RDATA(master_axi_rdresp_data[31:0]),
		.MAXIGP0RVALID(master_axi_rdresp_valid[0]),
		.MAXIGP0RREADY(master_axi_rdresp_ready[0]),
		.MAXIGP0RID(master_axi_rdresp_id[11:0]),
		.MAXIGP0RLAST(master_axi_rdresp_last[0]),
		.MAXIGP0RRESP(master_axi_rdresp_resp[1:0]),

		.MAXIGP0AWADDR(master_axi_wrreq_addr[31:0]),
		.MAXIGP0AWVALID(master_axi_wrreq_valid[0]),
		.MAXIGP0AWREADY(master_axi_wrreq_ready[0]),
		.MAXIGP0AWID(master_axi_wrreq_id[11:0]),
		.MAXIGP0AWLOCK(master_axi_wrreq_lock[1:0]),
		.MAXIGP0AWCACHE(master_axi_wrreq_cache[3:0]),
		.MAXIGP0AWPROT(master_axi_wrreq_prot[2:0]),
		.MAXIGP0AWLEN(master_axi_wrreq_len[3:0]),
		.MAXIGP0AWSIZE(master_axi_wrreq_size[1:0]),
		.MAXIGP0AWBURST(master_axi_wrreq_burst[1:0]),
		.MAXIGP0AWQOS(master_axi_wrreq_qos[3:0]),
		.MAXIGP0WDATA(master_axi_wrdat_data[31:0]),
		.MAXIGP0WVALID(master_axi_wrdat_valid[0]),
		.MAXIGP0WREADY(master_axi_wrdat_ready[0]),
		.MAXIGP0WID(master_axi_wrdat_id[11:0]),
		.MAXIGP0WLAST(master_axi_wrdat_last[0]),
		.MAXIGP0WSTRB(master_axi_wrdat_mask[3:0]),
		.MAXIGP0BVALID(master_axi_wrresp_valid[0]),
		.MAXIGP0BREADY(master_axi_wrresp_ready[0]),
		.MAXIGP0BID(master_axi_wrresp_id[11:0]),
		.MAXIGP0BRESP(master_axi_wrresp_resp[1:0]),

		//Master AXI GP 1
		.MAXIGP1ACLK(master_axi_clk[1]),
		.MAXIGP1ARESETN(master_axi_rst_n[1]),
		.MAXIGP1ARADDR(master_axi_rdreq_addr[63:32]),
		.MAXIGP1ARVALID(master_axi_rdreq_valid[1]),
		.MAXIGP1ARREADY(master_axi_rdreq_ready[1]),
		.MAXIGP1ARID(master_axi_rdreq_id[23:12]),
		.MAXIGP1ARLOCK(master_axi_rdreq_lock[3:2]),
		.MAXIGP1ARCACHE(master_axi_rdreq_cache[7:4]),
		.MAXIGP1ARPROT(master_axi_rdreq_prot[5:3]),
		.MAXIGP1ARLEN(master_axi_rdreq_len[7:4]),
		.MAXIGP1ARSIZE(master_axi_rdreq_size[3:2]),
		.MAXIGP1ARBURST(master_axi_rdreq_burst[3:2]),
		.MAXIGP1ARQOS(master_axi_rdreq_qos[7:4]),
		.MAXIGP1RDATA(master_axi_rdresp_data[63:32]),
		.MAXIGP1RVALID(master_axi_rdresp_valid[1]),
		.MAXIGP1RREADY(master_axi_rdresp_ready[1]),
		.MAXIGP1RID(master_axi_rdresp_id[23:12]),
		.MAXIGP1RLAST(master_axi_rdresp_last[1]),
		.MAXIGP1RRESP(master_axi_rdresp_resp[3:2]),

		.MAXIGP1AWADDR(master_axi_wrreq_addr[63:32]),
		.MAXIGP1AWVALID(master_axi_wrreq_valid[1]),
		.MAXIGP1AWREADY(master_axi_wrreq_ready[1]),
		.MAXIGP1AWID(master_axi_wrreq_id[23:12]),
		.MAXIGP1AWLOCK(master_axi_wrreq_lock[3:2]),
		.MAXIGP1AWCACHE(master_axi_wrreq_cache[7:4]),
		.MAXIGP1AWPROT(master_axi_wrreq_prot[5:3]),
		.MAXIGP1AWLEN(master_axi_wrreq_len[7:4]),
		.MAXIGP1AWSIZE(master_axi_wrreq_size[3:2]),
		.MAXIGP1AWBURST(master_axi_wrreq_burst[3:2]),
		.MAXIGP1AWQOS(master_axi_wrreq_qos[7:4]),
		.MAXIGP1WDATA(master_axi_wrdat_data[63:32]),
		.MAXIGP1WVALID(master_axi_wrdat_valid[1]),
		.MAXIGP1WREADY(master_axi_wrdat_ready[1]),
		.MAXIGP1WID(master_axi_wrdat_id[23:12]),
		.MAXIGP1WLAST(master_axi_wrdat_last[1]),
		.MAXIGP1WSTRB(master_axi_wrdat_mask[7:4]),
		.MAXIGP1BVALID(master_axi_wrresp_valid[1]),
		.MAXIGP1BREADY(master_axi_wrresp_ready[1]),
		.MAXIGP1BID(master_axi_wrresp_id[23:12]),
		.MAXIGP1BRESP(master_axi_wrresp_resp[3:2]),

		//Slave AXI ACP (not yet used)
		.SAXIACPACLK(acp_axi_clk),
		.SAXIACPARESETN(acp_axi_rst_n),
		.SAXIACPARADDR(acp_axi_rdreq_addr),
		.SAXIACPARVALID(acp_axi_rdreq_valid),
		.SAXIACPARREADY(acp_axi_rdreq_ready),
		.SAXIACPRID(acp_axi_rdreq_id),
		.SAXIACPARUSER(acp_axi_rdreq_user ),
		.SAXIACPARLOCK(acp_axi_rdreq_lock),
		.SAXIACPARCACHE(acp_axi_rdreq_cache),
		.SAXIACPARPROT(acp_axi_rdreq_prot),
		.SAXIACPARLEN(acp_axi_rdreq_len),
		.SAXIACPARSIZE(acp_axi_rdreq_size),
		.SAXIACPARBURST(acp_axi_rdreq_burst),
		.SAXIACPARQOS(acp_axi_rdreq_qos),
		.SAXIACPRDATA(acp_axi_rdresp_data),
		.SAXIACPRVALID(acp_axi_rdresp_valid),
		.SAXIACPRREADY(acp_axi_rdresp_ready),
		.SAXIACPARID(acp_axi_rdresp_id),
		.SAXIACPRLAST(acp_axi_rdresp_last),
		.SAXIACPRRESP(acp_axi_rdresp_resp),

		.SAXIACPAWADDR(acp_axi_wrreq_addr ),
		.SAXIACPAWVALID(acp_axi_wrreq_valid),
		.SAXIACPAWREADY(acp_axi_wrreq_ready),
		.SAXIACPAWID(acp_axi_wrreq_id),
		.SAXIACPAWUSER(acp_axi_wrreq_user),
		.SAXIACPAWLOCK(acp_axi_wrreq_lock),
		.SAXIACPAWCACHE(acp_axi_wrreq_cache),
		.SAXIACPAWPROT(acp_axi_wrreq_prot),
		.SAXIACPAWLEN(acp_axi_wrreq_len),
		.SAXIACPAWSIZE(acp_axi_wrreq_size),
		.SAXIACPAWBURST(acp_axi_wrreq_burst),
		.SAXIACPAWQOS(acp_axi_wrreq_qos),
		.SAXIACPWDATA(acp_axi_wrdat_data),
		.SAXIACPWVALID(acp_axi_wrdat_valid ),
		.SAXIACPWREADY(acp_axi_wrdat_ready),
		.SAXIACPWID(acp_axi_wrdat_id),
		.SAXIACPWLAST(acp_axi_wrdat_last),
		.SAXIACPWSTRB(acp_axi_wrdat_mask),
		.SAXIACPBVALID(acp_axi_wrresp_valid ),
		.SAXIACPBREADY(acp_axi_wrresp_ready),
		.SAXIACPBID(acp_axi_wrresp_id),
		.SAXIACPBRESP(acp_axi_wrresp_resp),

		//Slave AXI GP 0
		.SAXIGP0ACLK(slavegp_axi_clk[0]),
		.SAXIGP0ARESETN(slavegp_axi_rst_n[0]),
		.SAXIGP0ARADDR(slavegp_axi_rdreq_addr[31:0]),
		.SAXIGP0ARVALID(slavegp_axi_rdreq_valid[0]),
		.SAXIGP0ARREADY(slavegp_axi_rdreq_ready[0]),
		.SAXIGP0ARID(slavegp_axi_rdreq_id[5:0]),
		.SAXIGP0ARLOCK(slavegp_axi_rdreq_lock[1:0]),
		.SAXIGP0ARCACHE(slavegp_axi_rdreq_cache[3:0]),
		.SAXIGP0ARPROT(slavegp_axi_rdreq_prot[2:0]),
		.SAXIGP0ARLEN(slavegp_axi_rdreq_len[3:0]),
		.SAXIGP0ARSIZE(slavegp_axi_rdreq_size[1:0]),
		.SAXIGP0ARBURST(slavegp_axi_rdreq_burst[1:0]),
		.SAXIGP0ARQOS(slavegp_axi_rdreq_qos[3:0]),
		.SAXIGP0RDATA(slavegp_axi_rdresp_data[31:0]),
		.SAXIGP0RVALID(slavegp_axi_rdresp_valid[0]),
		.SAXIGP0RREADY(slavegp_axi_rdresp_ready[0]),
		.SAXIGP0RID(slavegp_axi_rdresp_id[5:0]),
		.SAXIGP0RLAST(slavegp_axi_rdresp_last[0]),
		.SAXIGP0RRESP(slavegp_axi_rdresp_resp[1:0]),

		.SAXIGP0AWADDR(slavegp_axi_wrreq_addr[31:0]),
		.SAXIGP0AWVALID(slavegp_axi_wrreq_valid[0]),
		.SAXIGP0AWREADY(slavegp_axi_wrreq_ready[0]),
		.SAXIGP0AWID(slavegp_axi_wrreq_id[5:0]),
		.SAXIGP0AWLOCK(slavegp_axi_wrreq_lock[1:0]),
		.SAXIGP0AWCACHE(slavegp_axi_wrreq_cache[3:0]),
		.SAXIGP0AWPROT(slavegp_axi_wrreq_prot[2:0]),
		.SAXIGP0AWLEN(slavegp_axi_wrreq_len[3:0]),
		.SAXIGP0AWSIZE(slavegp_axi_wrreq_size[1:0]),
		.SAXIGP0AWBURST(slavegp_axi_wrreq_burst[1:0]),
		.SAXIGP0AWQOS(slavegp_axi_wrreq_qos[3:0]),
		.SAXIGP0WDATA(slavegp_axi_wrdat_data[31:0]),
		.SAXIGP0WVALID(slavegp_axi_wrdat_valid[0]),
		.SAXIGP0WREADY(slavegp_axi_wrdat_ready[0]),
		.SAXIGP0WID(slavegp_axi_wrdat_id[5:0]),
		.SAXIGP0WLAST(slavegp_axi_wrdat_last[0]),
		.SAXIGP0WSTRB(slavegp_axi_wrdat_mask[3:0]),
		.SAXIGP0BVALID(slavegp_axi_wrresp_valid[0]),
		.SAXIGP0BREADY(slavegp_axi_wrresp_ready[0]),
		.SAXIGP0BID(slavegp_axi_wrresp_id[5:0]),
		.SAXIGP0BRESP(slavegp_axi_wrresp_resp[1:0]),

		//Slave AXI GP 1
		.SAXIGP1ACLK(slavegp_axi_clk[1]),
		.SAXIGP1ARESETN(slavegp_axi_rst_n[1]),
		.SAXIGP1ARADDR(slavegp_axi_rdreq_addr[63:32]),
		.SAXIGP1ARVALID(slavegp_axi_rdreq_valid[1]),
		.SAXIGP1ARREADY(slavegp_axi_rdreq_ready[1]),
		.SAXIGP1ARID(slavegp_axi_rdreq_id[11:6]),
		.SAXIGP1ARLOCK(slavegp_axi_rdreq_lock[3:2]),
		.SAXIGP1ARCACHE(slavegp_axi_rdreq_cache[7:4]),
		.SAXIGP1ARPROT(slavegp_axi_rdreq_prot[5:3]),
		.SAXIGP1ARLEN(slavegp_axi_rdreq_len[7:4]),
		.SAXIGP1ARSIZE(slavegp_axi_rdreq_size[3:2]),
		.SAXIGP1ARBURST(slavegp_axi_rdreq_burst[3:2]),
		.SAXIGP1ARQOS(slavegp_axi_rdreq_qos[7:4]),
		.SAXIGP1RDATA(slavegp_axi_rdresp_data[63:32]),
		.SAXIGP1RVALID(slavegp_axi_rdresp_valid[1]),
		.SAXIGP1RREADY(slavegp_axi_rdresp_ready[1]),
		.SAXIGP1RID(slavegp_axi_rdresp_id[11:6]),
		.SAXIGP1RLAST(slavegp_axi_rdresp_last[1]),
		.SAXIGP1RRESP(slavegp_axi_rdresp_resp[3:2]),

		.SAXIGP1AWADDR(slavegp_axi_wrreq_addr[63:32]),
		.SAXIGP1AWVALID(slavegp_axi_wrreq_valid[1]),
		.SAXIGP1AWREADY(slavegp_axi_wrreq_ready[1]),
		.SAXIGP1AWID(slavegp_axi_wrreq_id[11:6]),
		.SAXIGP1AWLOCK(slavegp_axi_wrreq_lock[3:2]),
		.SAXIGP1AWCACHE(slavegp_axi_wrreq_cache[7:4]),
		.SAXIGP1AWPROT(slavegp_axi_wrreq_prot[5:3]),
		.SAXIGP1AWLEN(slavegp_axi_wrreq_len[7:4]),
		.SAXIGP1AWSIZE(slavegp_axi_wrreq_size[3:2]),
		.SAXIGP1AWBURST(slavegp_axi_wrreq_burst[3:2]),
		.SAXIGP1AWQOS(slavegp_axi_wrreq_qos[7:4]),
		.SAXIGP1WDATA(slavegp_axi_wrdat_data[63:32]),
		.SAXIGP1WVALID(slavegp_axi_wrdat_valid[1]),
		.SAXIGP1WREADY(slavegp_axi_wrdat_ready[1]),
		.SAXIGP1WID(slavegp_axi_wrdat_id[11:6]),
		.SAXIGP1WLAST(slavegp_axi_wrdat_last[1]),
		.SAXIGP1WSTRB(slavegp_axi_wrdat_mask[7:4]),
		.SAXIGP1BVALID(slavegp_axi_wrresp_valid[1]),
		.SAXIGP1BREADY(slavegp_axi_wrresp_ready[1]),
		.SAXIGP1BID(slavegp_axi_wrresp_id[11:6]),
		.SAXIGP1BRESP(slavegp_axi_wrresp_resp[3:2]),

		//Slave AXI HP 0
		.SAXIHP0ACLK(slavehp_axi_clk[0]),
		.SAXIHP0RDISSUECAP1EN(slavehp_axi_rdissuecap1en[0]),
		.SAXIHP0ARESETN(slavehp_axi_rst_n[0]),
		.SAXIHP0ARADDR(slavehp_axi_rdreq_addr[31:0]),
		.SAXIHP0RACOUNT(slavehp_axi_rdreq_fifosize[2:0]),
		.SAXIHP0ARVALID(slavehp_axi_rdreq_valid[0]),
		.SAXIHP0ARREADY(slavehp_axi_rdreq_ready[0]),
		.SAXIHP0ARID(slavehp_axi_rdreq_id[5:0]),
		.SAXIHP0ARLOCK(slavehp_axi_rdreq_lock[1:0]),
		.SAXIHP0ARCACHE(slavehp_axi_rdreq_cache[3:0]),
		.SAXIHP0ARPROT(slavehp_axi_rdreq_prot[2:0]),
		.SAXIHP0ARLEN(slavehp_axi_rdreq_len[3:0]),
		.SAXIHP0ARSIZE(slavehp_axi_rdreq_size[1:0]),
		.SAXIHP0ARBURST(slavehp_axi_rdreq_burst[1:0]),
		.SAXIHP0ARQOS(slavehp_axi_rdreq_qos[3:0]),
		.SAXIHP0RDATA(slavehp_axi_rdresp_data[63:0]),
		.SAXIHP0RCOUNT(slavehp_axi_rdresp_fifosize[7:0]),
		.SAXIHP0RVALID(slavehp_axi_rdresp_valid[0]),
		.SAXIHP0RREADY(slavehp_axi_rdresp_ready[0]),
		.SAXIHP0RID(slavehp_axi_rdresp_id[5:0]),
		.SAXIHP0RLAST(slavehp_axi_rdresp_last[0]),
		.SAXIHP0RRESP(slavehp_axi_rdresp_resp[1:0]),

		.SAXIHP0WRISSUECAP1EN(slavehp_axi_wrissuecap1en[0]),
		.SAXIHP0AWADDR(slavehp_axi_wrreq_addr[31:0]),
		.SAXIHP0WACOUNT(slavehp_axi_wrreq_fifosize[5:0]),
		.SAXIHP0AWVALID(slavehp_axi_wrreq_valid[0]),
		.SAXIHP0AWREADY(slavehp_axi_wrreq_ready[0]),
		.SAXIHP0AWID(slavehp_axi_wrreq_id[5:0]),
		.SAXIHP0AWLOCK(slavehp_axi_wrreq_lock[1:0]),
		.SAXIHP0AWCACHE(slavehp_axi_wrreq_cache[3:0]),
		.SAXIHP0AWPROT(slavehp_axi_wrreq_prot[2:0]),
		.SAXIHP0AWLEN(slavehp_axi_wrreq_len[3:0]),
		.SAXIHP0AWSIZE(slavehp_axi_wrreq_size[1:0]),
		.SAXIHP0AWBURST(slavehp_axi_wrreq_burst[1:0]),
		.SAXIHP0AWQOS(slavehp_axi_wrreq_qos[3:0]),
		.SAXIHP0WDATA(slavehp_axi_wrdat_data[63:0]),
		.SAXIHP0WCOUNT(slavehp_axi_wrdat_fifosize[7:0]),
		.SAXIHP0WVALID(slavehp_axi_wrdat_valid[0]),
		.SAXIHP0WREADY(slavehp_axi_wrdat_ready[0]),
		.SAXIHP0WID(slavehp_axi_wrdat_id[5:0]),
		.SAXIHP0WLAST(slavehp_axi_wrdat_last[0]),
		.SAXIHP0WSTRB(slavehp_axi_wrdat_mask[7:0]),
		.SAXIHP0BVALID(slavehp_axi_wrresp_valid[0]),
		.SAXIHP0BREADY(slavehp_axi_wrresp_ready[0]),
		.SAXIHP0BID(slavehp_axi_wrresp_id[5:0]),
		.SAXIHP0BRESP(slavehp_axi_wrresp_resp[1:0]),

		//Slave AXI HP 1
		.SAXIHP1ACLK(slavehp_axi_clk[1]),
		.SAXIHP1RDISSUECAP1EN(slavehp_axi_rdissuecap1en[1]),
		.SAXIHP1ARESETN(slavehp_axi_rst_n[1]),
		.SAXIHP1ARADDR(slavehp_axi_rdreq_addr[63:32]),
		.SAXIHP1RACOUNT(slavehp_axi_rdreq_fifosize[5:3]),
		.SAXIHP1ARVALID(slavehp_axi_rdreq_valid[1]),
		.SAXIHP1ARREADY(slavehp_axi_rdreq_ready[1]),
		.SAXIHP1ARID(slavehp_axi_rdreq_id[11:6]),
		.SAXIHP1ARLOCK(slavehp_axi_rdreq_lock[3:2]),
		.SAXIHP1ARCACHE(slavehp_axi_rdreq_cache[7:4]),
		.SAXIHP1ARPROT(slavehp_axi_rdreq_prot[5:3]),
		.SAXIHP1ARLEN(slavehp_axi_rdreq_len[7:4]),
		.SAXIHP1ARSIZE(slavehp_axi_rdreq_size[3:2]),
		.SAXIHP1ARBURST(slavehp_axi_rdreq_burst[3:2]),
		.SAXIHP1ARQOS(slavehp_axi_rdreq_qos[7:4]),
		.SAXIHP1RDATA(slavehp_axi_rdresp_data[127:64]),
		.SAXIHP1RCOUNT(slavehp_axi_rdresp_fifosize[15:8]),
		.SAXIHP1RVALID(slavehp_axi_rdresp_valid[1]),
		.SAXIHP1RREADY(slavehp_axi_rdresp_ready[1]),
		.SAXIHP1RID(slavehp_axi_rdresp_id[11:6]),
		.SAXIHP1RLAST(slavehp_axi_rdresp_last[1]),
		.SAXIHP1RRESP(slavehp_axi_rdresp_resp[3:2]),

		.SAXIHP1WRISSUECAP1EN(slavehp_axi_wrissuecap1en[1]),
		.SAXIHP1AWADDR(slavehp_axi_wrreq_addr[63:32]),
		.SAXIHP1WACOUNT(slavehp_axi_wrreq_fifosize[11:6]),
		.SAXIHP1AWVALID(slavehp_axi_wrreq_valid[1]),
		.SAXIHP1AWREADY(slavehp_axi_wrreq_ready[1]),
		.SAXIHP1AWID(slavehp_axi_wrreq_id[11:6]),
		.SAXIHP1AWLOCK(slavehp_axi_wrreq_lock[3:2]),
		.SAXIHP1AWCACHE(slavehp_axi_wrreq_cache[7:4]),
		.SAXIHP1AWPROT(slavehp_axi_wrreq_prot[5:3]),
		.SAXIHP1AWLEN(slavehp_axi_wrreq_len[7:4]),
		.SAXIHP1AWSIZE(slavehp_axi_wrreq_size[3:2]),
		.SAXIHP1AWBURST(slavehp_axi_wrreq_burst[3:2]),
		.SAXIHP1AWQOS(slavehp_axi_wrreq_qos[7:4]),
		.SAXIHP1WDATA(slavehp_axi_wrdat_data[127:64]),
		.SAXIHP1WCOUNT(slavehp_axi_wrdat_fifosize[15:8]),
		.SAXIHP1WVALID(slavehp_axi_wrdat_valid[1]),
		.SAXIHP1WREADY(slavehp_axi_wrdat_ready[1]),
		.SAXIHP1WID(slavehp_axi_wrdat_id[11:6]),
		.SAXIHP1WLAST(slavehp_axi_wrdat_last[1]),
		.SAXIHP1WSTRB(slavehp_axi_wrdat_mask[15:8]),
		.SAXIHP1BVALID(slavehp_axi_wrresp_valid[1]),
		.SAXIHP1BREADY(slavehp_axi_wrresp_ready[1]),
		.SAXIHP1BID(slavehp_axi_wrresp_id[11:6]),
		.SAXIHP1BRESP(slavehp_axi_wrresp_resp[3:2]),

		//Slave AXI HP 2
		.SAXIHP2ACLK(slavehp_axi_clk[2]),
		.SAXIHP2RDISSUECAP1EN(slavehp_axi_rdissuecap1en[2]),
		.SAXIHP2ARESETN(slavehp_axi_rst_n[2]),
		.SAXIHP2ARADDR(slavehp_axi_rdreq_addr[95:64]),
		.SAXIHP2RACOUNT(slavehp_axi_rdreq_fifosize[8:6]),
		.SAXIHP2ARVALID(slavehp_axi_rdreq_valid[2]),
		.SAXIHP2ARREADY(slavehp_axi_rdreq_ready[2]),
		.SAXIHP2ARID(slavehp_axi_rdreq_id[17:12]),
		.SAXIHP2ARLOCK(slavehp_axi_rdreq_lock[5:4]),
		.SAXIHP2ARCACHE(slavehp_axi_rdreq_cache[11:8]),
		.SAXIHP2ARPROT(slavehp_axi_rdreq_prot[8:6]),
		.SAXIHP2ARLEN(slavehp_axi_rdreq_len[11:8]),
		.SAXIHP2ARSIZE(slavehp_axi_rdreq_size[5:4]),
		.SAXIHP2ARBURST(slavehp_axi_rdreq_burst[5:4]),
		.SAXIHP2ARQOS(slavehp_axi_rdreq_qos[11:8]),
		.SAXIHP2RDATA(slavehp_axi_rdresp_data[191:128]),
		.SAXIHP2RCOUNT(slavehp_axi_rdresp_fifosize[23:16]),
		.SAXIHP2RVALID(slavehp_axi_rdresp_valid[2]),
		.SAXIHP2RREADY(slavehp_axi_rdresp_ready[2]),
		.SAXIHP2RID(slavehp_axi_rdresp_id[17:12]),
		.SAXIHP2RLAST(slavehp_axi_rdresp_last[2]),
		.SAXIHP2RRESP(slavehp_axi_rdresp_resp[5:4]),

		.SAXIHP2WRISSUECAP1EN(slavehp_axi_wrissuecap1en[2]),
		.SAXIHP2AWADDR(slavehp_axi_wrreq_addr[95:64]),
		.SAXIHP2WACOUNT(slavehp_axi_wrreq_fifosize[17:12]),
		.SAXIHP2AWVALID(slavehp_axi_wrreq_valid[2]),
		.SAXIHP2AWREADY(slavehp_axi_wrreq_ready[2]),
		.SAXIHP2AWID(slavehp_axi_wrreq_id[17:12]),
		.SAXIHP2AWLOCK(slavehp_axi_wrreq_lock[5:4]),
		.SAXIHP2AWCACHE(slavehp_axi_wrreq_cache[11:8]),
		.SAXIHP2AWPROT(slavehp_axi_wrreq_prot[8:6]),
		.SAXIHP2AWLEN(slavehp_axi_wrreq_len[11:8]),
		.SAXIHP2AWSIZE(slavehp_axi_wrreq_size[5:4]),
		.SAXIHP2AWBURST(slavehp_axi_wrreq_burst[5:4]),
		.SAXIHP2AWQOS(slavehp_axi_wrreq_qos[11:8]),
		.SAXIHP2WDATA(slavehp_axi_wrdat_data[191:128]),
		.SAXIHP2WCOUNT(slavehp_axi_wrdat_fifosize[23:16]),
		.SAXIHP2WVALID(slavehp_axi_wrdat_valid[2]),
		.SAXIHP2WREADY(slavehp_axi_wrdat_ready[2]),
		.SAXIHP2WID(slavehp_axi_wrdat_id[17:12]),
		.SAXIHP2WLAST(slavehp_axi_wrdat_last[2]),
		.SAXIHP2WSTRB(slavehp_axi_wrdat_mask[23:16]),
		.SAXIHP2BVALID(slavehp_axi_wrresp_valid[2]),
		.SAXIHP2BREADY(slavehp_axi_wrresp_ready[2]),
		.SAXIHP2BID(slavehp_axi_wrresp_id[17:12]),
		.SAXIHP2BRESP(slavehp_axi_wrresp_resp[5:4]),

		//Slave AXI HP 3
		.SAXIHP3ACLK(slavehp_axi_clk[3]),
		.SAXIHP3RDISSUECAP1EN(slavehp_axi_rdissuecap1en[3]),
		.SAXIHP3ARESETN(slavehp_axi_rst_n[3]),
		.SAXIHP3ARADDR(slavehp_axi_rdreq_addr[127:96]),
		.SAXIHP3RACOUNT(slavehp_axi_rdreq_fifosize[11:9]),
		.SAXIHP3ARVALID(slavehp_axi_rdreq_valid[3]),
		.SAXIHP3ARREADY(slavehp_axi_rdreq_ready[3]),
		.SAXIHP3ARID(slavehp_axi_rdreq_id[23:18]),
		.SAXIHP3ARLOCK(slavehp_axi_rdreq_lock[7:6]),
		.SAXIHP3ARCACHE(slavehp_axi_rdreq_cache[15:12]),
		.SAXIHP3ARPROT(slavehp_axi_rdreq_prot[11:9]),
		.SAXIHP3ARLEN(slavehp_axi_rdreq_len[15:12]),
		.SAXIHP3ARSIZE(slavehp_axi_rdreq_size[7:6]),
		.SAXIHP3ARBURST(slavehp_axi_rdreq_burst[7:6]),
		.SAXIHP3ARQOS(slavehp_axi_rdreq_qos[15:12]),
		.SAXIHP3RDATA(slavehp_axi_rdresp_data[255:192]),
		.SAXIHP3RCOUNT(slavehp_axi_rdresp_fifosize[31:24]),
		.SAXIHP3RVALID(slavehp_axi_rdresp_valid[3]),
		.SAXIHP3RREADY(slavehp_axi_rdresp_ready[3]),
		.SAXIHP3RID(slavehp_axi_rdresp_id[23:18]),
		.SAXIHP3RLAST(slavehp_axi_rdresp_last[3]),
		.SAXIHP3RRESP(slavehp_axi_rdresp_resp[7:6]),

		.SAXIHP3WRISSUECAP1EN(slavehp_axi_wrissuecap1en[3]),
		.SAXIHP3AWADDR(slavehp_axi_wrreq_addr[127:96]),
		.SAXIHP3WACOUNT(slavehp_axi_wrreq_fifosize[23:18]),
		.SAXIHP3AWVALID(slavehp_axi_wrreq_valid[3]),
		.SAXIHP3AWREADY(slavehp_axi_wrreq_ready[3]),
		.SAXIHP3AWID(slavehp_axi_wrreq_id[23:18]),
		.SAXIHP3AWLOCK(slavehp_axi_wrreq_lock[7:6]),
		.SAXIHP3AWCACHE(slavehp_axi_wrreq_cache[15:12]),
		.SAXIHP3AWPROT(slavehp_axi_wrreq_prot[11:9]),
		.SAXIHP3AWLEN(slavehp_axi_wrreq_len[15:12]),
		.SAXIHP3AWSIZE(slavehp_axi_wrreq_size[7:6]),
		.SAXIHP3AWBURST(slavehp_axi_wrreq_burst[7:6]),
		.SAXIHP3AWQOS(slavehp_axi_wrreq_qos[15:12]),
		.SAXIHP3WDATA(slavehp_axi_wrdat_data[255:192]),
		.SAXIHP3WCOUNT(slavehp_axi_wrdat_fifosize[31:24]),
		.SAXIHP3WVALID(slavehp_axi_wrdat_valid[3]),
		.SAXIHP3WREADY(slavehp_axi_wrdat_ready[3]),
		.SAXIHP3WID(slavehp_axi_wrdat_id[23:18]),
		.SAXIHP3WLAST(slavehp_axi_wrdat_last[3]),
		.SAXIHP3WSTRB(slavehp_axi_wrdat_mask[31:24]),
		.SAXIHP3BVALID(slavehp_axi_wrresp_valid[3]),
		.SAXIHP3BREADY(slavehp_axi_wrresp_ready[3]),
		.SAXIHP3BID(slavehp_axi_wrresp_id[23:18]),
		.SAXIHP3BRESP(slavehp_axi_wrresp_resp[7:6])
	);

endmodule
