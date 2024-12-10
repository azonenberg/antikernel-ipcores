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

#ifndef APB_GPIO_h
#define APB_GPIO_h

/**
	@brief Registers for a GPIO pin
 */
struct APB_GPIO
{
public:
	uint32_t		out;
	uint32_t		in;
	uint32_t		tris;
};

/**
	@brief Wrapper class for APB_GPIO

	Semantics largely identical to the stm32-cpp APB_GPIOPin class

	TODO: open drain support?
 */
class APB_GPIOPin
{
public:

	enum gpiomode_t
	{
		MODE_INPUT		= 0,
		MODE_OUTPUT		= 1,
		//no peripheral/analog mode supported
	};

	enum initmode_t
	{
		INIT_NOW		= 0,
		INIT_DEFERRED	= 1
	};

	//no slew rate control supported

	/**
		@brief Initializes the pin

		@param imode	If set to INIT_NOW, behaves normally
						If set to INIT_DEFERRED, doesn't set mode register until DeferredInit() is called.
						This allows the object to be declared at global scope before the FPGA has been brought up.
	 */
	APB_GPIOPin(
		volatile APB_GPIO* gpio,
		uint8_t pin,
		gpiomode_t mode,
		initmode_t imode = INIT_NOW)
	: m_gpio(gpio)
	, m_pin(pin)
	, m_setmask(1 << pin)
	, m_clearmask(~m_setmask)
	, m_mode(mode)
	{
		//Configure the pin
		if(imode != INIT_DEFERRED)
			SetMode(mode);
	}

	void DeferredInit()
	{ SetMode(m_mode); }

	/**
		@brief Set the pin to input or output mode
	 */
	void SetMode(gpiomode_t mode)
	{
		if(mode == MODE_OUTPUT)
			m_gpio->tris &= m_clearmask;
		else
			m_gpio->tris |= m_setmask;
	}

	/**
		@brief Drives a value out the pin
	 */
	void Set(bool b)
	{
		if(b)
			m_gpio->out |= m_setmask;
		else
			m_gpio->out &= m_clearmask;
	}

	//Convenience helper for assigning GPIOs
	void operator=(bool b)
	{ Set(b); }

	//Convenience helper for reading GPIOs
	operator bool() const
	{ return Get(); }

	/**
		@brief Reads the current value of the pin
	 */
	bool Get() const
	{
		if(m_gpio->in & m_setmask)
			return true;
		else
			return false;
	}

protected:
	volatile APB_GPIO* 	m_gpio;
	uint8_t				m_pin;
	uint32_t			m_setmask;
	uint32_t			m_clearmask;
	gpiomode_t			m_mode;
};

#endif
