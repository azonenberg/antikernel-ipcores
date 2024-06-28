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

#ifndef APB_UART_h
#define APB_UART_h

#include <embedded-utils/CharacterDevice.h>

/**
	@brief Registers for a UART channel
 */
struct APB_UART
{
	uint32_t status;
	uint32_t clkdiv;
	uint32_t tx_data;
	uint32_t rx_data;
};

/**
	@brief Wrapper class for APB_UART

	Semantics largely identical to the stm32-cpp UART class but derived from CharacterDevice rather than
	BufferedCharacterDevice, since our buffering is implemented in hardware
 */
class UART : public CharacterDevice
{
public:

	UART(volatile APB_UART* lane, uint32_t baud_div = 181)
	 : UART(lane, lane, baud_div)
	{}

	UART(volatile APB_UART* txlane, volatile APB_UART* rxlane, uint32_t baud_div)
		: m_txlane(txlane)
		, m_rxlane(rxlane)
	{
		m_txlane->clkdiv = baud_div;
		m_rxlane->clkdiv = baud_div;
	}

	//TX side
	virtual void PrintBinary(char ch) override
	{
		//Block if the FIFO has no free space
		while(m_txlane->status & 1)
		{}

		m_txlane->tx_data = ch;
	}

	bool DataPending()
	{ return (m_rxlane->status & 2) == 2; }

	virtual char BlockingRead()
	{
		//Block if the FIFO has nothing to read
		while( (m_rxlane->status & 2) == 0)
		{}

		return m_rxlane->rx_data;
	}

protected:
	volatile APB_UART* m_txlane;
	volatile APB_UART* m_rxlane;
};

#endif
