#ifndef _STM32_MCO_H_
#define _STM32_MCO_H_

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
 * $Id: stm32_mco.c 3182 2010-10-17 21:46:04Z Astralix $
 * \endverbatim
 */
#include <stdint.h>

/* STM32 MCU Clock Out source selectors */
#define MCO_Source_OFF      ((uint8_t)0x00)
#define MCO_Source_SYSCLK   ((uint8_t)0x01)
#define MCO_Source_HSI      ((uint8_t)0x02)
#define MCO_Source_MSI      ((uint8_t)0x03)
#define MCO_Source_HSE      ((uint8_t)0x04)
#define MCO_Source_PLLCLK   ((uint8_t)0x05)
#define MCO_Source_LSI      ((uint8_t)0x06)
#define MCO_Source_LSE      ((uint8_t)0x07)

/* MCU Clock Out Dividers */
#define MCODiv_1            ((uint8_t)0x00)
#define MCODiv_2            ((uint8_t)0x10)
#define MCODiv_4            ((uint8_t)0x20)
#define MCODiv_8            ((uint8_t)0x30)
#define MCODiv_16           ((uint8_t)0x40)

/* Setup MCO */
extern int Stm32McoSetup( uint8_t cs, uint8_t cdiv);

#endif /* _STM32_MCO_H_ */
