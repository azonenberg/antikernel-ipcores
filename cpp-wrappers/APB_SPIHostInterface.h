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

#ifndef APB_APB_SPIHostInterface_h
#define APB_APB_SPIHostInterface_h

#include <embedded-utils/CharacterDevice.h>

/**
	@brief Registers for a APB_SPIHostInterface channel
 */
struct APB_SPIHostInterface
{
	uint32_t clkdiv;
	uint32_t field_04[7];
	uint32_t data;
	uint32_t field_24[7];
	uint32_t cs_n;
	uint32_t field_44[7];
	uint32_t status;
	uint32_t field_64[7];
	uint32_t status2;
};

/**
	@brief Wrapper class for APB_SPIHostInterface

	Not currently compatible with stm32-cpp SPI class
 */
class APB_SPIHostInterfaceDriver : public CharacterDevice
{
public:

	APB_SPIHostInterfaceDriver(volatile APB_SPIHostInterface* lane, uint32_t baud_div = 181)
		: m_lane(lane)
	{
		m_lane->clkdiv = baud_div;
		#ifdef __arm__
			asm("dmb st");
		#endif
	}

	virtual void WaitUntilIdle()
	{
		#ifdef QSPI_CACHE_WORKAROUND
			asm("dmb st");
			while(true)
			{
				uint32_t a = m_lane->status;
				uint32_t b = m_lane->status2;
				if(!a && !b)
					break;
			}
		#else
			while(m_lane->status)
			{}
		#endif
	}

	//TX side
	virtual void PrintBinary(char ch) override
	{
		WaitUntilIdle();
		m_lane->data = ch;
	}

	virtual char BlockingRead()
	{
		WaitUntilIdle();
		return m_lane->data;
	}

	void SetCS(bool b)
	{ m_lane->cs_n = b; }

protected:
	volatile APB_SPIHostInterface* m_lane;
};

#endif
