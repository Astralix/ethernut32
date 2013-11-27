#ifndef _STM32_CLK_H_
#define _STM32_CLK_H_

/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*
 * \verbatim
 * $Id: stm32_gpio.c 3182 2010-10-17 21:46:04Z Astralix $
 * \endverbatim
 */
#include <stdint.h>

/* STM32F Clock source selectors */
#if defined(MCU_STM32F1) || defined(MCU_STM32F2) || defined(MCU_STM32F3) || defined(MCU_STM32F4)
#define SYSCLK_HSI  0		/* SYSCLK fed by internal high speed RC oscillator */
#define SYSCLK_PLL  1		/* SYSCLK derived from PLL */
#define SYSCLK_HSE  2		/* SYSCLK fed by external oscillator */
#define PLLCLK_HSI  10		/* PLL clocked by HSI clock */
#define PLLCLK_HSE  11		/* PLL clocked by HSE clock */

#elif defined(MCU_STM32L1)
#define SYSCLK_MSI  0		/* SYSCLK fed by internal medium speed RC oscillator */
#define SYSCLK_HSI  1		/* SYSCLK fed by internal high speed RC oscillator */
#define SYSCLK_HSE  2		/* SYSCLK fed by external oscillator */
#define SYSCLK_PLL  3		/* SYSCLK fed by internal PLL */

#define PLLCLK_HSI  10		/* PLL clocked by HSI clock */
#define PLLCLK_HSE  11		/* PLL clocked by HSE clock */
#endif

/* RTC clock sources */
#if defined(MCU_STM32F1) || defined(MCU_STM32F2) || defined(MCU_STM32F3) || defined(MCU_STM32F4)
#define RTCCLK_NONE   0
#define RTCCLK_LSE    1
#define RTCCLK_LSI    2
#define RTCCLK_HSE    3
#define RTCCLK_LSEBYP 4
#elif defined(MCU_STM32L1)
#define RTCCLK_NONE   0
#define RTCCLK_HSE    3
#define RTCCLK_LSE    1
#define RTCCLK_LSI    2
#endif

/* Retrieve CPU/AHB/APBx clock*/
extern uint32_t STM_ClockGet(int idx);

/* Control functions for the separate clocks */
extern int CtlHseClock( uint8_t ena);
extern int CtlHsiClock( uint8_t ena);
extern int CtlPllClock( uint8_t ena);
#if defined(MCU_STM32L1)
extern int CtlMsiClock( uint8_t ena);
#endif

/* Selection functions for the clock sources */
extern int SetPllClock( int src);
extern int SetPllClockSource( int src);
extern int SetSysClock(void);
extern uint32_t SysCtlClockGet(void);

#if defined(MCU_STM32L1)
extern int SetRTCClock(int source);
#endif
#endif /* _STM32_CLK_H_ */
