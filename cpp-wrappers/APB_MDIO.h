/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL                                                                                                           *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg                                                                          *
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

#ifndef APB_MDIO_h
#define APB_MDIO_h

/**
	@brief Registers for MDIO transceiver
 */
struct APB_MDIO
{
	uint32_t	cmd_addr;
	uint32_t	field_04[7];
	uint32_t	data;
	uint32_t	field_14[7];
	uint32_t	status;
	uint32_t	field_24[7];
	uint32_t	status2;
};

enum base_mdioreg_t
{
	//IEEE defined registers
	REG_BASIC_CONTROL			= 0x0000,
	REG_BASIC_STATUS			= 0x0001,
	REG_PHY_ID_1				= 0x0002,
	REG_PHY_ID_2				= 0x0003,
	REG_AN_ADVERT				= 0x0004,
	REG_GIG_CONTROL				= 0x0009,

	//Extended register access
	REG_PHY_REGCR				= 0x000d,
	REG_PHY_ADDAR				= 0x000e,

	//KSZ9031 specific
	REG_KSZ9031_MDIX			= 0x001c,

	//KSZ9031 MMD 2
	REG_KSZ9031_MMD2_CLKSKEW	= 0x0008
};

/**
	@brief A single device on an APB_MDIO bus
 */
class MDIODevice
{
public:
	MDIODevice(volatile APB_MDIO* mdio, uint8_t phyaddr)
	: m_mdio(mdio)
	, m_phyaddr(phyaddr)
	{}

	/**
		@brief Write a value to a MDIO register
	 */
	void WriteRegister(uint8_t regid, uint16_t value)
	{
		m_mdio->cmd_addr = (regid << 8) | 0x8000 | m_phyaddr;
		m_mdio->data = value;
		WaitUntilIdle();
	}

	/**
		@brief Read a value from an MDIO register
	 */
	uint16_t ReadRegister(uint8_t regid)
	{
		m_mdio->cmd_addr = (regid << 8) | m_phyaddr;
		WaitUntilIdle();
		return m_mdio->data;
	}

	/**
		@brief Write an extended register
	 */
	void __attribute__((noinline)) WriteExtendedRegister(uint8_t mmd, uint8_t regid, uint16_t regval)
	{
		WriteRegister(REG_PHY_REGCR, mmd);			//set address
		WriteRegister(REG_PHY_ADDAR, regid);
		WriteRegister(REG_PHY_REGCR, mmd | 0x4000);	//data, no post inc
		WriteRegister(REG_PHY_ADDAR, regval);
	}

	/**
		@brief Read an extended register
	 */
	uint16_t __attribute__((noinline)) ReadExtendedRegister(uint8_t mmd, uint8_t regid)
	{
		WriteRegister(REG_PHY_REGCR, mmd);			//set address
		WriteRegister(REG_PHY_ADDAR, regid);
		WriteRegister(REG_PHY_REGCR, mmd | 0x4000);	//data, no post inc
		return ReadRegister(REG_PHY_ADDAR);
	}

protected:
	/**
		@brief Wait for the ongoing transaction to complete
	 */
	void WaitUntilIdle()
	{
		#ifdef QSPI_CACHE_WORKAROUND
			asm("dmb st");
			while(true)
			{
				uint32_t a = m_mdio->status;
				uint32_t b = m_mdio->status2;
				if(!a && !b)
					break;
			}
		#else
			while(m_mdio->status)
			{}
		#endif
	}


protected:
	volatile APB_MDIO* m_mdio;
	uint8_t m_phyaddr;
};

#endif
