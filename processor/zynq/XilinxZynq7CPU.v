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
	// Unused signals (not yet implemented)

	//DMA channels
	wire[3:0] dma_rst_n;
	wire[3:0] dma_clk			= 4'h0;
	wire[3:0] dma_req_ready;
	wire[3:0] dma_req_valid		= 4'h0;
	wire[7:0] dma_req_type		= 8'h0;
	wire[3:0] dma_req_last		= 4'h0;
	wire[3:0] dma_ack_ready		= 4'h0;
	wire[3:0] dma_ack_valid;
	wire[7:0] dma_ack_type;

	//CAN interfaces
	wire[1:0] can_phy_rx		= 2'h0;
	wire[1:0] can_phy_tx;

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

		/*
		.FCLKCLK(FCLK_CLK_unbuffered),
		.FCLKRESETN({FCLK_RESET3_N,FCLK_RESET2_N,FCLK_RESET1_N,FCLK_RESET0_N}),
		.FCLKCLKTRIGN(fclk_clktrig_gnd),
		.FPGAIDLEN(FPGA_IDLE_N),
		.FTMDTRACEINATID(FTMD_TRACEIN_ATID_i),
		.FTMDTRACEINCLOCK(FTMD_TRACEIN_CLK),
		.FTMDTRACEINDATA(FTMD_TRACEIN_DATA_i),
		.FTMDTRACEINVALID(FTMD_TRACEIN_VALID_i),
		.FTMTF2PDEBUG(FTMT_F2P_DEBUG  ),
		.FTMTF2PTRIG(FTMT_F2P_TRIG   ),
		.FTMTP2FTRIGACK(FTMT_P2F_TRIGACK),
		.EMIOSRAMINTIN(SRAM_INTIN),
		.EVENTEVENTI(EVENT_EVENTI),
		*/

		//EMIO trace port (not yet used)
		/*
		.EMIOTRACECTL(TRACE_CTL),
		.EMIOTRACEDATA(TRACE_DATA),
		.EMIOTRACECLK(TRACE_CLK),
		.EMIOTTC0CLKI({TTC0_CLK2_IN, TTC0_CLK1_IN, TTC0_CLK0_IN}),
		.EMIOTTC1CLKI({TTC1_CLK2_IN, TTC1_CLK1_IN, TTC1_CLK0_IN}),
		.EMIOTTC0WAVEO({TTC0_WAVE2_OUT,TTC0_WAVE1_OUT,TTC0_WAVE0_OUT}),
		.EMIOTTC1WAVEO({TTC1_WAVE2_OUT,TTC1_WAVE1_OUT,TTC1_WAVE0_OUT}),
		*/

		//Watchdog timer (not yet used)
		/*
		.EMIOWDTRSTO(WDT_RST_OUT),
		.EMIOWDTCLKI(WDT_CLK_IN),
		*/

		//Event stuff? What's this for?
		/*
		.EVENTEVENTO(EVENT_EVENTO),
		.EVENTSTANDBYWFE(EVENT_STANDBYWFE),
		.EVENTSTANDBYWFI(EVENT_STANDBYWFI),
		*/

		//IRQ stuff (not yet used)
		/*
		.IRQF2P(irq_f2p_i),
		.IRQP2F({IRQ_P2F_DMAC_ABORT, IRQ_P2F_DMAC7, IRQ_P2F_DMAC6, IRQ_P2F_DMAC5, IRQ_P2F_DMAC4, IRQ_P2F_DMAC3, IRQ_P2F_DMAC2, IRQ_P2F_DMAC1, IRQ_P2F_DMAC0, IRQ_P2F_SMC, IRQ_P2F_QSPI, IRQ_P2F_CTI, IRQ_P2F_GPIO, IRQ_P2F_USB0, IRQ_P2F_ENET0, IRQ_P2F_ENET_WAKE0, IRQ_P2F_SDIO0, IRQ_P2F_I2C0, IRQ_P2F_SPI0, IRQ_P2F_UART0, IRQ_P2F_CAN0, IRQ_P2F_USB1, IRQ_P2F_ENET1, IRQ_P2F_ENET_WAKE1, IRQ_P2F_SDIO1, IRQ_P2F_I2C1, IRQ_P2F_SPI1, IRQ_P2F_UART1, IRQ_P2F_CAN1}),
		*/

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
		.EMIOSDIO1WP(sdio_write_protect[1])//,

		//EMIO SPI 0 (not yet used)
		/*
		.EMIOSPI0MO(SPI0_MOSI_O),
		.EMIOSPI0MOTN(SPI0_MOSI_T_n),
		.EMIOSPI0SCLKO(SPI0_SCLK_O),
		.EMIOSPI0SCLKTN(SPI0_SCLK_T_n),
		.EMIOSPI0SO(SPI0_MISO_O),
		.EMIOSPI0STN(SPI0_MISO_T_n),
		.EMIOSPI0SSON({SPI0_SS2_O,SPI0_SS1_O,SPI0_SS_O}),
		.EMIOSPI0SSNTN(SPI0_SS_T_n),
		.EMIOSPI0MI(SPI0_MISO_I),
		.EMIOSPI0SCLKI(SPI0_SCLK_I),
		.EMIOSPI0SI(SPI0_MOSI_I),
		.EMIOSPI0SSIN(SPI0_SS_I),
		*/

		//EMIO SPI 1 (not yet used)
		/*
		.EMIOSPI1MO(SPI1_MOSI_O),
		.EMIOSPI1MOTN(SPI1_MOSI_T_n),
		.EMIOSPI1SCLKO(SPI1_SCLK_O),
		.EMIOSPI1SCLKTN(SPI1_SCLK_T_n),
		.EMIOSPI1SO(SPI1_MISO_O),
		.EMIOSPI1STN(SPI1_MISO_T_n),
		.EMIOSPI1SSON({SPI1_SS2_O,SPI1_SS1_O,SPI1_SS_O}),
		.EMIOSPI1SSNTN(SPI1_SS_T_n),
		.EMIOSPI1MI(SPI1_MISO_I),
		.EMIOSPI1SCLKI(SPI1_SCLK_I),
		.EMIOSPI1SI(SPI1_MOSI_I),
		.EMIOSPI1SSIN(SPI1_SS_I),
		*/

		//EMIO UART 0 (not yet used)
		/*
		.EMIOUART0DTRN(UART0_DTRN),
		.EMIOUART0RTSN(UART0_RTSN),
		.EMIOUART0TX(UART0_TX  ),
		.EMIOUART0CTSN(UART0_CTSN),
		.EMIOUART0DCDN(UART0_DCDN),
		.EMIOUART0DSRN(UART0_DSRN),
		.EMIOUART0RIN(UART0_RIN ),
		.EMIOUART0RX(UART0_RX  ),
		*/

		//EMIO UART 1 (not yet used)
		/*
		.EMIOUART1DTRN(UART1_DTRN),
		.EMIOUART1RTSN(UART1_RTSN),
		.EMIOUART1TX(UART1_TX  ),
		.EMIOUART1CTSN(UART1_CTSN),
		.EMIOUART1DCDN(UART1_DCDN),
		.EMIOUART1DSRN(UART1_DSRN),
		.EMIOUART1RIN(UART1_RIN ),
		.EMIOUART1RX(UART1_RX  ),
		*/

		//EMIO USB 0 (not yet used)
		/*
		.EMIOUSB0PORTINDCTL(USB0_PORT_INDCTL),
		.EMIOUSB0VBUSPWRSELECT(USB0_VBUS_PWRSELECT),
		.EMIOUSB0VBUSPWRFAULT(USB0_VBUS_PWRFAULT),
		*/

		//EMIO USB 1 (not yet used)
		/*
		.EMIOUSB1PORTINDCTL(USB1_PORT_INDCTL),
		.EMIOUSB1VBUSPWRSELECT(USB1_VBUS_PWRSELECT),
		.EMIOUSB1VBUSPWRFAULT(USB1_VBUS_PWRFAULT),
		*/

		//What's this?
		/*
		.FTMTF2PTRIGACK(FTMT_F2P_TRIGACK),
		.FTMTP2FDEBUG(FTMT_P2F_DEBUG  ),
		.FTMTP2FTRIG(FTMT_P2F_TRIG   ),
		*/

		//Master AXI GP 0 (not yet used)
		/*
		.MAXIGP0ARADDR(M_AXI_GP0_ARADDR),
		.MAXIGP0ARBURST(M_AXI_GP0_ARBURST),
		.MAXIGP0ARCACHE(M_AXI_GP0_ARCACHE),
		.MAXIGP0ARESETN(M_AXI_GP0_ARESETN),
		.MAXIGP0ARID(M_AXI_GP0_ARID_FULL   ),
		.MAXIGP0ARLEN(M_AXI_GP0_ARLEN  ),
		.MAXIGP0ARLOCK(M_AXI_GP0_ARLOCK ),
		.MAXIGP0ARPROT(M_AXI_GP0_ARPROT ),
		.MAXIGP0ARQOS(M_AXI_GP0_ARQOS  ),
		.MAXIGP0ARSIZE(M_AXI_GP0_ARSIZE_i ),
		.MAXIGP0ARVALID(M_AXI_GP0_ARVALID),
		.MAXIGP0AWADDR(M_AXI_GP0_AWADDR ),
		.MAXIGP0AWBURST(M_AXI_GP0_AWBURST),
		.MAXIGP0AWCACHE(M_AXI_GP0_AWCACHE),
		.MAXIGP0AWID(M_AXI_GP0_AWID_FULL   ),
		.MAXIGP0AWLEN(M_AXI_GP0_AWLEN  ),
		.MAXIGP0AWLOCK(M_AXI_GP0_AWLOCK ),
		.MAXIGP0AWPROT(M_AXI_GP0_AWPROT ),
		.MAXIGP0AWQOS(M_AXI_GP0_AWQOS  ),
		.MAXIGP0AWSIZE(M_AXI_GP0_AWSIZE_i ),
		.MAXIGP0AWVALID(M_AXI_GP0_AWVALID),
		.MAXIGP0BREADY(M_AXI_GP0_BREADY ),
		.MAXIGP0RREADY(M_AXI_GP0_RREADY ),
		.MAXIGP0WDATA(M_AXI_GP0_WDATA  ),
		.MAXIGP0WID(M_AXI_GP0_WID_FULL    ),
		.MAXIGP0WLAST(M_AXI_GP0_WLAST  ),
		.MAXIGP0WSTRB(M_AXI_GP0_WSTRB  ),
		.MAXIGP0WVALID(M_AXI_GP0_WVALID ),
		.MAXIGP0ACLK(M_AXI_GP0_ACLK),
		.MAXIGP0ARREADY(M_AXI_GP0_ARREADY),
		.MAXIGP0AWREADY(M_AXI_GP0_AWREADY),
		.MAXIGP0BID(M_AXI_GP0_BID_FULL    ),
		.MAXIGP0BRESP(M_AXI_GP0_BRESP  ),
		.MAXIGP0BVALID(M_AXI_GP0_BVALID ),
		.MAXIGP0RDATA(M_AXI_GP0_RDATA  ),
		.MAXIGP0RID(M_AXI_GP0_RID_FULL    ),
		.MAXIGP0RLAST(M_AXI_GP0_RLAST  ),
		.MAXIGP0RRESP(M_AXI_GP0_RRESP  ),
		.MAXIGP0RVALID(M_AXI_GP0_RVALID ),
		.MAXIGP0WREADY(M_AXI_GP0_WREADY ),
		*/

		//Master AXI GP 1 (not yet used)
		/*
		.MAXIGP1ARADDR(M_AXI_GP1_ARADDR ),
		.MAXIGP1ARBURST(M_AXI_GP1_ARBURST),
		.MAXIGP1ARCACHE(M_AXI_GP1_ARCACHE),
		.MAXIGP1ARESETN(M_AXI_GP1_ARESETN),
		.MAXIGP1ARID(M_AXI_GP1_ARID_FULL   ),
		.MAXIGP1ARLEN(M_AXI_GP1_ARLEN  ),
		.MAXIGP1ARLOCK(M_AXI_GP1_ARLOCK ),
		.MAXIGP1ARPROT(M_AXI_GP1_ARPROT ),
		.MAXIGP1ARQOS(M_AXI_GP1_ARQOS  ),
		.MAXIGP1ARSIZE(M_AXI_GP1_ARSIZE_i ),
		.MAXIGP1ARVALID(M_AXI_GP1_ARVALID),
		.MAXIGP1AWADDR(M_AXI_GP1_AWADDR ),
		.MAXIGP1AWBURST(M_AXI_GP1_AWBURST),
		.MAXIGP1AWCACHE(M_AXI_GP1_AWCACHE),
		.MAXIGP1AWID(M_AXI_GP1_AWID_FULL   ),
		.MAXIGP1AWLEN(M_AXI_GP1_AWLEN  ),
		.MAXIGP1AWLOCK(M_AXI_GP1_AWLOCK ),
		.MAXIGP1AWPROT(M_AXI_GP1_AWPROT ),
		.MAXIGP1AWQOS(M_AXI_GP1_AWQOS  ),
		.MAXIGP1AWSIZE(M_AXI_GP1_AWSIZE_i ),
		.MAXIGP1AWVALID(M_AXI_GP1_AWVALID),
		.MAXIGP1BREADY(M_AXI_GP1_BREADY ),
		.MAXIGP1RREADY(M_AXI_GP1_RREADY ),
		.MAXIGP1WDATA(M_AXI_GP1_WDATA  ),
		.MAXIGP1WID(M_AXI_GP1_WID_FULL    ),
		.MAXIGP1WLAST(M_AXI_GP1_WLAST  ),
		.MAXIGP1WSTRB(M_AXI_GP1_WSTRB  ),
		.MAXIGP1WVALID(M_AXI_GP1_WVALID ),
		.MAXIGP1ACLK(M_AXI_GP1_ACLK   ),
		.MAXIGP1ARREADY(M_AXI_GP1_ARREADY),
		.MAXIGP1AWREADY(M_AXI_GP1_AWREADY),
		.MAXIGP1BID(M_AXI_GP1_BID_FULL ),
		.MAXIGP1BRESP(M_AXI_GP1_BRESP  ),
		.MAXIGP1BVALID(M_AXI_GP1_BVALID ),
		.MAXIGP1RDATA(M_AXI_GP1_RDATA  ),
		.MAXIGP1RID(M_AXI_GP1_RID_FULL    ),
		.MAXIGP1RLAST(M_AXI_GP1_RLAST  ),
		.MAXIGP1RRESP(M_AXI_GP1_RRESP  ),
		.MAXIGP1RVALID(M_AXI_GP1_RVALID ),
		.MAXIGP1WREADY(M_AXI_GP1_WREADY ),
		*/

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

		//Slave AXI ACP (not yet used)
		/*
		.SAXIACPARESETN(S_AXI_ACP_ARESETN),
		.SAXIACPARREADY(SAXIACPARREADY_W),
		.SAXIACPAWREADY(SAXIACPAWREADY_W),
		.SAXIACPBID(S_AXI_ACP_BID_out    ),
		.SAXIACPBRESP(SAXIACPBRESP_W  ),
		.SAXIACPBVALID(SAXIACPBVALID_W ),
		.SAXIACPRDATA(SAXIACPRDATA_W  ),
		.SAXIACPRID(S_AXI_ACP_RID_out),
		.SAXIACPRLAST(SAXIACPRLAST_W  ),
		.SAXIACPRRESP(SAXIACPRRESP_W  ),
		.SAXIACPRVALID(SAXIACPRVALID_W ),
		.SAXIACPWREADY(SAXIACPWREADY_W ),
		.SAXIACPACLK(S_AXI_ACP_ACLK   ),
		.SAXIACPARADDR(SAXIACPARADDR_W ),
		.SAXIACPARBURST(SAXIACPARBURST_W),
		.SAXIACPARCACHE(SAXIACPARCACHE_W),
		.SAXIACPARID(S_AXI_ACP_ARID_in   ),
		.SAXIACPARLEN(SAXIACPARLEN_W  ),
		.SAXIACPARLOCK(SAXIACPARLOCK_W ),
		.SAXIACPARPROT(SAXIACPARPROT_W ),
		.SAXIACPARQOS(S_AXI_ACP_ARQOS  ),
		.SAXIACPARSIZE(SAXIACPARSIZE_W[1:0] ),
		.SAXIACPARUSER(SAXIACPARUSER_W ),
		.SAXIACPARVALID(SAXIACPARVALID_W),
		.SAXIACPAWADDR(SAXIACPAWADDR_W ),
		.SAXIACPAWBURST(SAXIACPAWBURST_W),
		.SAXIACPAWCACHE(SAXIACPAWCACHE_W),
		.SAXIACPAWID(S_AXI_ACP_AWID_in   ),
		.SAXIACPAWLEN(SAXIACPAWLEN_W  ),
		.SAXIACPAWLOCK(SAXIACPAWLOCK_W ),
		.SAXIACPAWPROT(SAXIACPAWPROT_W ),
		.SAXIACPAWQOS(S_AXI_ACP_AWQOS  ),
		.SAXIACPAWSIZE(SAXIACPAWSIZE_W[1:0] ),
		.SAXIACPAWUSER(SAXIACPAWUSER_W ),
		.SAXIACPAWVALID(SAXIACPAWVALID_W),
		.SAXIACPBREADY(SAXIACPBREADY_W ),
		.SAXIACPRREADY(SAXIACPRREADY_W ),
		.SAXIACPWDATA(SAXIACPWDATA_W  ),
		.SAXIACPWID(S_AXI_ACP_WID_in    ),
		.SAXIACPWLAST(SAXIACPWLAST_W  ),
		.SAXIACPWSTRB(SAXIACPWSTRB_W  ),
		.SAXIACPWVALID(SAXIACPWVALID_W ),
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
