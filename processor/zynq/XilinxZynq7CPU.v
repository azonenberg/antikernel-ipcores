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

	//Slave AXI ACP link (transactions initiated by FPGA, cache coherent, 64 bits)
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

		//MIO pins (why are these brought out when we have everything else?)
		//.MIO(MIO),

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

		//.EMIOSRAMINTIN(SRAM_INTIN),

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
		.SAXIACPBRESP(acp_axi_wrresp_resp)//,

		//Slave AXI GP 0 (not yet used)
		/*
		.SAXIGP0ARESETN(S_AXI_GP0_ARESETN),
		.SAXIGP0ARREADY(S_AXI_GP0_ARREADY),
		.SAXIGP0AWREADY(S_AXI_GP0_AWREADY),
		.SAXIGP0BID(S_AXI_GP0_BID_out),
		.SAXIGP0BRESP(S_AXI_GP0_BRESP  ),
		.SAXIGP0BVALID(S_AXI_GP0_BVALID ),
		.SAXIGP0RDATA(S_AXI_GP0_RDATA  ),
		.SAXIGP0RID(S_AXI_GP0_RID_out ),
		.SAXIGP0RLAST(S_AXI_GP0_RLAST  ),
		.SAXIGP0RRESP(S_AXI_GP0_RRESP  ),
		.SAXIGP0RVALID(S_AXI_GP0_RVALID ),
		.SAXIGP0WREADY(S_AXI_GP0_WREADY ),
		.SAXIGP0ACLK(S_AXI_GP0_ACLK   ),
		.SAXIGP0ARADDR(S_AXI_GP0_ARADDR ),
		.SAXIGP0ARBURST(S_AXI_GP0_ARBURST),
		.SAXIGP0ARCACHE(S_AXI_GP0_ARCACHE),
		.SAXIGP0ARID(S_AXI_GP0_ARID_in   ),
		.SAXIGP0ARLEN(S_AXI_GP0_ARLEN  ),
		.SAXIGP0ARLOCK(S_AXI_GP0_ARLOCK ),
		.SAXIGP0ARPROT(S_AXI_GP0_ARPROT ),
		.SAXIGP0ARQOS(S_AXI_GP0_ARQOS  ),
		.SAXIGP0ARSIZE(S_AXI_GP0_ARSIZE[1:0] ),
		.SAXIGP0ARVALID(S_AXI_GP0_ARVALID),
		.SAXIGP0AWADDR(S_AXI_GP0_AWADDR ),
		.SAXIGP0AWBURST(S_AXI_GP0_AWBURST),
		.SAXIGP0AWCACHE(S_AXI_GP0_AWCACHE),
		.SAXIGP0AWID(S_AXI_GP0_AWID_in   ),
		.SAXIGP0AWLEN(S_AXI_GP0_AWLEN  ),
		.SAXIGP0AWLOCK(S_AXI_GP0_AWLOCK ),
		.SAXIGP0AWPROT(S_AXI_GP0_AWPROT ),
		.SAXIGP0AWQOS(S_AXI_GP0_AWQOS  ),
		.SAXIGP0AWSIZE(S_AXI_GP0_AWSIZE[1:0] ),
		.SAXIGP0AWVALID(S_AXI_GP0_AWVALID),
		.SAXIGP0BREADY(S_AXI_GP0_BREADY ),
		.SAXIGP0RREADY(S_AXI_GP0_RREADY ),
		.SAXIGP0WDATA(S_AXI_GP0_WDATA  ),
		.SAXIGP0WID(S_AXI_GP0_WID_in ),
		.SAXIGP0WLAST(S_AXI_GP0_WLAST  ),
		.SAXIGP0WSTRB(S_AXI_GP0_WSTRB  ),
		.SAXIGP0WVALID(S_AXI_GP0_WVALID ),
		*/

		//Slave AXI GP 1 (not yet used)
		/*
		.SAXIGP1ARESETN(S_AXI_GP1_ARESETN),
		.SAXIGP1ARREADY(S_AXI_GP1_ARREADY),
		.SAXIGP1AWREADY(S_AXI_GP1_AWREADY),
		.SAXIGP1BID(S_AXI_GP1_BID_out    ),
		.SAXIGP1BRESP(S_AXI_GP1_BRESP  ),
		.SAXIGP1BVALID(S_AXI_GP1_BVALID ),
		.SAXIGP1RDATA(S_AXI_GP1_RDATA  ),
		.SAXIGP1RID(S_AXI_GP1_RID_out    ),
		.SAXIGP1RLAST(S_AXI_GP1_RLAST  ),
		.SAXIGP1RRESP(S_AXI_GP1_RRESP  ),
		.SAXIGP1RVALID(S_AXI_GP1_RVALID ),
		.SAXIGP1WREADY(S_AXI_GP1_WREADY ),
		.SAXIGP1ACLK(S_AXI_GP1_ACLK   ),
		.SAXIGP1ARADDR(S_AXI_GP1_ARADDR ),
		.SAXIGP1ARBURST(S_AXI_GP1_ARBURST),
		.SAXIGP1ARCACHE(S_AXI_GP1_ARCACHE),
		.SAXIGP1ARID(S_AXI_GP1_ARID_in   ),
		.SAXIGP1ARLEN(S_AXI_GP1_ARLEN  ),
		.SAXIGP1ARLOCK(S_AXI_GP1_ARLOCK ),
		.SAXIGP1ARPROT(S_AXI_GP1_ARPROT ),
		.SAXIGP1ARQOS(S_AXI_GP1_ARQOS  ),
		.SAXIGP1ARSIZE(S_AXI_GP1_ARSIZE[1:0] ),
		.SAXIGP1ARVALID(S_AXI_GP1_ARVALID),
		.SAXIGP1AWADDR(S_AXI_GP1_AWADDR ),
		.SAXIGP1AWBURST(S_AXI_GP1_AWBURST),
		.SAXIGP1AWCACHE(S_AXI_GP1_AWCACHE),
		.SAXIGP1AWID(S_AXI_GP1_AWID_in   ),
		.SAXIGP1AWLEN(S_AXI_GP1_AWLEN  ),
		.SAXIGP1AWLOCK(S_AXI_GP1_AWLOCK ),
		.SAXIGP1AWPROT(S_AXI_GP1_AWPROT ),
		.SAXIGP1AWQOS(S_AXI_GP1_AWQOS  ),
		.SAXIGP1AWSIZE(S_AXI_GP1_AWSIZE[1:0] ),
		.SAXIGP1AWVALID(S_AXI_GP1_AWVALID),
		.SAXIGP1BREADY(S_AXI_GP1_BREADY ),
		.SAXIGP1RREADY(S_AXI_GP1_RREADY ),
		.SAXIGP1WDATA(S_AXI_GP1_WDATA  ),
		.SAXIGP1WID(S_AXI_GP1_WID_in    ),
		.SAXIGP1WLAST(S_AXI_GP1_WLAST  ),
		.SAXIGP1WSTRB(S_AXI_GP1_WSTRB  ),
		.SAXIGP1WVALID(S_AXI_GP1_WVALID ),
		 */

		//Slave AXI HP 0 (not yet used)
		/*
		.SAXIHP0ARESETN(S_AXI_HP0_ARESETN),
		.SAXIHP0ARREADY(S_AXI_HP0_ARREADY),
		.SAXIHP0AWREADY(S_AXI_HP0_AWREADY),
		.SAXIHP0BID(S_AXI_HP0_BID_out    ),
		.SAXIHP0BRESP(S_AXI_HP0_BRESP  ),
		.SAXIHP0BVALID(S_AXI_HP0_BVALID ),
		.SAXIHP0RACOUNT(S_AXI_HP0_RACOUNT),
		.SAXIHP0RCOUNT(S_AXI_HP0_RCOUNT),
		.SAXIHP0RDATA(S_AXI_HP0_RDATA_out),
		.SAXIHP0RID(S_AXI_HP0_RID_out ),
		.SAXIHP0RLAST(S_AXI_HP0_RLAST),
		.SAXIHP0RRESP(S_AXI_HP0_RRESP),
		.SAXIHP0RVALID(S_AXI_HP0_RVALID),
		.SAXIHP0WCOUNT(S_AXI_HP0_WCOUNT),
		.SAXIHP0WACOUNT(S_AXI_HP0_WACOUNT),
		.SAXIHP0WREADY(S_AXI_HP0_WREADY),
		.SAXIHP0ACLK(S_AXI_HP0_ACLK   ),
		.SAXIHP0ARADDR(S_AXI_HP0_ARADDR),
		.SAXIHP0ARBURST(S_AXI_HP0_ARBURST),
		.SAXIHP0ARCACHE(S_AXI_HP0_ARCACHE),
		.SAXIHP0ARID(S_AXI_HP0_ARID_in),
		.SAXIHP0ARLEN(S_AXI_HP0_ARLEN),
		.SAXIHP0ARLOCK(S_AXI_HP0_ARLOCK),
		.SAXIHP0ARPROT(S_AXI_HP0_ARPROT),
		.SAXIHP0ARQOS(S_AXI_HP0_ARQOS),
		.SAXIHP0ARSIZE(S_AXI_HP0_ARSIZE[1:0]),
		.SAXIHP0ARVALID(S_AXI_HP0_ARVALID),
		.SAXIHP0AWADDR(S_AXI_HP0_AWADDR),
		.SAXIHP0AWBURST(S_AXI_HP0_AWBURST),
		.SAXIHP0AWCACHE(S_AXI_HP0_AWCACHE),
		.SAXIHP0AWID(S_AXI_HP0_AWID_in),
		.SAXIHP0AWLEN(S_AXI_HP0_AWLEN),
		.SAXIHP0AWLOCK(S_AXI_HP0_AWLOCK),
		.SAXIHP0AWPROT(S_AXI_HP0_AWPROT),
		.SAXIHP0AWQOS(S_AXI_HP0_AWQOS),
		.SAXIHP0AWSIZE(S_AXI_HP0_AWSIZE[1:0]),
		.SAXIHP0AWVALID(S_AXI_HP0_AWVALID),
		.SAXIHP0BREADY(S_AXI_HP0_BREADY),
		.SAXIHP0RDISSUECAP1EN(S_AXI_HP0_RDISSUECAP1_EN),
		.SAXIHP0RREADY(S_AXI_HP0_RREADY),
		.SAXIHP0WDATA(S_AXI_HP0_WDATA_in),
		.SAXIHP0WID(S_AXI_HP0_WID_in),
		.SAXIHP0WLAST(S_AXI_HP0_WLAST),
		.SAXIHP0WRISSUECAP1EN(S_AXI_HP0_WRISSUECAP1_EN),
		.SAXIHP0WSTRB(S_AXI_HP0_WSTRB_in),
		.SAXIHP0WVALID(S_AXI_HP0_WVALID),
		 */

		//Slave AXI HP 1 (not yet used)
		/*
		.SAXIHP1ARESETN(S_AXI_HP1_ARESETN),
		.SAXIHP1ARREADY(S_AXI_HP1_ARREADY),
		.SAXIHP1AWREADY(S_AXI_HP1_AWREADY),
		.SAXIHP1BID(S_AXI_HP1_BID_out    ),
		.SAXIHP1BRESP(S_AXI_HP1_BRESP  ),
		.SAXIHP1BVALID(S_AXI_HP1_BVALID ),
		.SAXIHP1RACOUNT(S_AXI_HP1_RACOUNT ),
		.SAXIHP1RCOUNT(S_AXI_HP1_RCOUNT ),
		.SAXIHP1RDATA(S_AXI_HP1_RDATA_out),
		.SAXIHP1RID(S_AXI_HP1_RID_out    ),
		.SAXIHP1RLAST(S_AXI_HP1_RLAST  ),
		.SAXIHP1RRESP(S_AXI_HP1_RRESP  ),
		.SAXIHP1RVALID(S_AXI_HP1_RVALID),
		.SAXIHP1WACOUNT(S_AXI_HP1_WACOUNT),
		.SAXIHP1WCOUNT(S_AXI_HP1_WCOUNT),
		.SAXIHP1WREADY(S_AXI_HP1_WREADY),
		.SAXIHP1ACLK(S_AXI_HP1_ACLK),
		.SAXIHP1ARADDR(S_AXI_HP1_ARADDR),
		.SAXIHP1ARBURST(S_AXI_HP1_ARBURST),
		.SAXIHP1ARCACHE(S_AXI_HP1_ARCACHE),
		.SAXIHP1ARID(S_AXI_HP1_ARID_in),
		.SAXIHP1ARLEN(S_AXI_HP1_ARLEN),
		.SAXIHP1ARLOCK(S_AXI_HP1_ARLOCK),
		.SAXIHP1ARPROT(S_AXI_HP1_ARPROT),
		.SAXIHP1ARQOS(S_AXI_HP1_ARQOS),
		.SAXIHP1ARSIZE(S_AXI_HP1_ARSIZE[1:0]),
		.SAXIHP1ARVALID(S_AXI_HP1_ARVALID),
		.SAXIHP1AWADDR(S_AXI_HP1_AWADDR),
		.SAXIHP1AWBURST(S_AXI_HP1_AWBURST),
		.SAXIHP1AWCACHE(S_AXI_HP1_AWCACHE),
		.SAXIHP1AWID(S_AXI_HP1_AWID_in),
		.SAXIHP1AWLEN(S_AXI_HP1_AWLEN),
		.SAXIHP1AWLOCK(S_AXI_HP1_AWLOCK),
		.SAXIHP1AWPROT(S_AXI_HP1_AWPROT),
		.SAXIHP1AWQOS(S_AXI_HP1_AWQOS),
		.SAXIHP1AWSIZE(S_AXI_HP1_AWSIZE[1:0]),
		.SAXIHP1AWVALID(S_AXI_HP1_AWVALID),
		.SAXIHP1BREADY(S_AXI_HP1_BREADY),
		.SAXIHP1RDISSUECAP1EN(S_AXI_HP1_RDISSUECAP1_EN),
		.SAXIHP1RREADY(S_AXI_HP1_RREADY),
		.SAXIHP1WDATA(S_AXI_HP1_WDATA_in),
		.SAXIHP1WID(S_AXI_HP1_WID_in),
		.SAXIHP1WLAST(S_AXI_HP1_WLAST),
		.SAXIHP1WRISSUECAP1EN(S_AXI_HP1_WRISSUECAP1_EN),
		.SAXIHP1WSTRB(S_AXI_HP1_WSTRB_in),
		.SAXIHP1WVALID(S_AXI_HP1_WVALID),
		*/

		//Slave AXI HP 2 (not yet used)
		/*
		.SAXIHP2ARESETN(S_AXI_HP2_ARESETN),
		.SAXIHP2ARREADY(S_AXI_HP2_ARREADY),
		.SAXIHP2AWREADY(S_AXI_HP2_AWREADY),
		.SAXIHP2BID(S_AXI_HP2_BID_out ),
		.SAXIHP2BRESP(S_AXI_HP2_BRESP),
		.SAXIHP2BVALID(S_AXI_HP2_BVALID),
		.SAXIHP2RACOUNT(S_AXI_HP2_RACOUNT),
		.SAXIHP2RCOUNT(S_AXI_HP2_RCOUNT),
		.SAXIHP2RDATA(S_AXI_HP2_RDATA_out),
		.SAXIHP2RID(S_AXI_HP2_RID_out ),
		.SAXIHP2RLAST(S_AXI_HP2_RLAST),
		.SAXIHP2RRESP(S_AXI_HP2_RRESP),
		.SAXIHP2RVALID(S_AXI_HP2_RVALID),
		.SAXIHP2WACOUNT(S_AXI_HP2_WACOUNT),
		.SAXIHP2WCOUNT(S_AXI_HP2_WCOUNT),
		.SAXIHP2WREADY(S_AXI_HP2_WREADY),
		.SAXIHP2ACLK(S_AXI_HP2_ACLK),
		.SAXIHP2ARADDR(S_AXI_HP2_ARADDR),
		.SAXIHP2ARBURST(S_AXI_HP2_ARBURST),
		.SAXIHP2ARCACHE(S_AXI_HP2_ARCACHE),
		.SAXIHP2ARID(S_AXI_HP2_ARID_in),
		.SAXIHP2ARLEN(S_AXI_HP2_ARLEN),
		.SAXIHP2ARLOCK(S_AXI_HP2_ARLOCK),
		.SAXIHP2ARPROT(S_AXI_HP2_ARPROT),
		.SAXIHP2ARQOS(S_AXI_HP2_ARQOS),
		.SAXIHP2ARSIZE(S_AXI_HP2_ARSIZE[1:0]),
		.SAXIHP2ARVALID(S_AXI_HP2_ARVALID),
		.SAXIHP2AWADDR(S_AXI_HP2_AWADDR),
		.SAXIHP2AWBURST(S_AXI_HP2_AWBURST),
		.SAXIHP2AWCACHE(S_AXI_HP2_AWCACHE),
		.SAXIHP2AWID(S_AXI_HP2_AWID_in),
		.SAXIHP2AWLEN(S_AXI_HP2_AWLEN),
		.SAXIHP2AWLOCK(S_AXI_HP2_AWLOCK),
		.SAXIHP2AWPROT(S_AXI_HP2_AWPROT),
		.SAXIHP2AWQOS(S_AXI_HP2_AWQOS),
		.SAXIHP2AWSIZE(S_AXI_HP2_AWSIZE[1:0]),
		.SAXIHP2AWVALID(S_AXI_HP2_AWVALID),
		.SAXIHP2BREADY(S_AXI_HP2_BREADY),
		.SAXIHP2RDISSUECAP1EN(S_AXI_HP2_RDISSUECAP1_EN),
		.SAXIHP2RREADY(S_AXI_HP2_RREADY),
		.SAXIHP2WDATA(S_AXI_HP2_WDATA_in),
		.SAXIHP2WID(S_AXI_HP2_WID_in),
		.SAXIHP2WLAST(S_AXI_HP2_WLAST),
		.SAXIHP2WRISSUECAP1EN(S_AXI_HP2_WRISSUECAP1_EN),
		.SAXIHP2WSTRB(S_AXI_HP2_WSTRB_in),
		.SAXIHP2WVALID(S_AXI_HP2_WVALID),
		 */

		//Slave AXI HP 3 (not yet used)
		/*
		.SAXIHP3ARESETN(S_AXI_HP3_ARESETN),
		.SAXIHP3ARREADY(S_AXI_HP3_ARREADY),
		.SAXIHP3AWREADY(S_AXI_HP3_AWREADY),
		.SAXIHP3BID(S_AXI_HP3_BID_out),
		.SAXIHP3BRESP(S_AXI_HP3_BRESP),
		.SAXIHP3BVALID(S_AXI_HP3_BVALID),
		.SAXIHP3RACOUNT(S_AXI_HP3_RACOUNT),
		.SAXIHP3RCOUNT(S_AXI_HP3_RCOUNT),
		.SAXIHP3RDATA(S_AXI_HP3_RDATA_out),
		.SAXIHP3RID(S_AXI_HP3_RID_out),
		.SAXIHP3RLAST(S_AXI_HP3_RLAST),
		.SAXIHP3RRESP(S_AXI_HP3_RRESP),
		.SAXIHP3RVALID(S_AXI_HP3_RVALID),
		.SAXIHP3WCOUNT(S_AXI_HP3_WCOUNT),
		.SAXIHP3WACOUNT(S_AXI_HP3_WACOUNT),
		.SAXIHP3WREADY(S_AXI_HP3_WREADY),
		.SAXIHP3ACLK(S_AXI_HP3_ACLK),
		.SAXIHP3ARADDR(S_AXI_HP3_ARADDR ),
		.SAXIHP3ARBURST(S_AXI_HP3_ARBURST),
		.SAXIHP3ARCACHE(S_AXI_HP3_ARCACHE),
		.SAXIHP3ARID(S_AXI_HP3_ARID_in   ),
		.SAXIHP3ARLEN(S_AXI_HP3_ARLEN),
		.SAXIHP3ARLOCK(S_AXI_HP3_ARLOCK),
		.SAXIHP3ARPROT(S_AXI_HP3_ARPROT),
		.SAXIHP3ARQOS(S_AXI_HP3_ARQOS),
		.SAXIHP3ARSIZE(S_AXI_HP3_ARSIZE[1:0]),
		.SAXIHP3ARVALID(S_AXI_HP3_ARVALID),
		.SAXIHP3AWADDR(S_AXI_HP3_AWADDR),
		.SAXIHP3AWBURST(S_AXI_HP3_AWBURST),
		.SAXIHP3AWCACHE(S_AXI_HP3_AWCACHE),
		.SAXIHP3AWID(S_AXI_HP3_AWID_in),
		.SAXIHP3AWLEN(S_AXI_HP3_AWLEN),
		.SAXIHP3AWLOCK(S_AXI_HP3_AWLOCK),
		.SAXIHP3AWPROT(S_AXI_HP3_AWPROT),
		.SAXIHP3AWQOS(S_AXI_HP3_AWQOS),
		.SAXIHP3AWSIZE(S_AXI_HP3_AWSIZE[1:0]),
		.SAXIHP3AWVALID(S_AXI_HP3_AWVALID),
		.SAXIHP3BREADY(S_AXI_HP3_BREADY),
		.SAXIHP3RDISSUECAP1EN(S_AXI_HP3_RDISSUECAP1_EN),
		.SAXIHP3RREADY(S_AXI_HP3_RREADY),
		.SAXIHP3WDATA(S_AXI_HP3_WDATA_in),
		.SAXIHP3WID(S_AXI_HP3_WID_in),
		.SAXIHP3WLAST(S_AXI_HP3_WLAST),
		.SAXIHP3WRISSUECAP1EN(S_AXI_HP3_WRISSUECAP1_EN),
		.SAXIHP3WSTRB(S_AXI_HP3_WSTRB_in),
		.SAXIHP3WVALID(S_AXI_HP3_WVALID),
		 */

		//DDR RAM (not yet used)
		/*
		.DDRARB(DDR_ARB),
		.DDRA(DDR_Addr),
		.DDRBA(DDR_BankAddr),
		.DDRCASB(DDR_CAS_n),
		.DDRCKE(DDR_CKE),
		.DDRCKN(DDR_Clk_n),
		.DDRCKP(DDR_Clk),
		.DDRCSB(DDR_CS_n),
		.DDRDM(DDR_DM),
		.DDRDQ(DDR_DQ),
		.DDRDQSN(DDR_DQS_n),
		.DDRDQSP(DDR_DQS),
		.DDRDRSTB(DDR_DRSTB),
		.DDRODT(DDR_ODT),
		.DDRRASB(DDR_RAS_n),
		.DDRVRN(DDR_VRN),
		.DDRVRP(DDR_VRP),
		.DDRWEB(DDR_WEB),
		 */
	);

endmodule
