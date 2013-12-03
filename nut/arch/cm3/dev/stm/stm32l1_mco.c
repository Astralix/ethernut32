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

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/gpio.h>

#include <cfg/clock.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_gpio.h>

#define MCO_PORT    NUTGPIO_PORTA
#define MCO_PIN     8

#if 0
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOSEL                     ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOSEL_0                   ((uint32_t)0x01000000)        /*!< Bit 0 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOSEL_1                   ((uint32_t)0x02000000)        /*!< Bit 1 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOSEL_2                   ((uint32_t)0x04000000)        /*!< Bit 2 */
include/arch/cm3/stm/vendor/stm32l1xx.h:/*!< MCO configuration */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_NOCLOCK                ((uint32_t)0x00000000)        /*!< No clock */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_SYSCLK                 ((uint32_t)0x01000000)        /*!< System clock selected */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_HSI                    ((uint32_t)0x02000000)        /*!< Internal 16 MHz RC oscillator clock selected */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_MSI                    ((uint32_t)0x03000000)        /*!< Internal Medium Speed RC oscillator clock selected */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_HSE                    ((uint32_t)0x04000000)        /*!< External 1-25 MHz oscillator clock selected */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_PLL                    ((uint32_t)0x05000000)        /*!< PLL clock divided */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_LSI                    ((uint32_t)0x06000000)        /*!< LSI selected */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_LSE                    ((uint32_t)0x07000000)        /*!< LSE selected */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOPRE                     ((uint32_t)0x70000000)        /*!< MCOPRE[2:0] bits (Microcontroller Clock Output Prescaler) */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOPRE_0                   ((uint32_t)0x10000000)        /*!< Bit 0 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOPRE_1                   ((uint32_t)0x20000000)        /*!< Bit 1 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCOPRE_2                   ((uint32_t)0x40000000)        /*!< Bit 2 */
include/arch/cm3/stm/vendor/stm32l1xx.h:/*!< MCO Prescaler configuration */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_DIV1                   ((uint32_t)0x00000000)        /*!< MCO Clock divided by 1 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_DIV2                   ((uint32_t)0x10000000)        /*!< MCO Clock divided by 2 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_DIV4                   ((uint32_t)0x20000000)        /*!< MCO Clock divided by 4 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_DIV8                   ((uint32_t)0x30000000)        /*!< MCO Clock divided by 8 */
include/arch/cm3/stm/vendor/stm32l1xx.h:#define  RCC_CFGR_MCO_DIV16                  ((uint32_t)0x40000000)        /*!< MCO Clock divided by 16 */
#endif

/*!
 * \brief Initialize and start MCU clock output.
 *
 * \param cs    Clock source.
 * \param cdic  Divider.
 */
int Stm32McoSetup( uint8_t cs, uint8_t cdiv)
{
    uint32_t mcoset;

    /* Setup Related GPIO. */
	// GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
    GpioPinConfigSet( MCO_PORT, MCO_PIN,
                    GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL 
                    | GPIO_CFG_SPEED_FAST);
    GPIO_PinAFConfig( MCO_PORT, MCO_PIN, GPIO_AF_MCO);

    /* assemble bits */
    mcoset = (((uint32_t)(cs|cdiv))<<24);
    /* limit to valid values */
    mcoset &= (RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE);

    /* Select MCO clock source and divider */
    RCC->CFGR &= ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE);
    RCC->CFGR |= mcoset;

    return 0;
}

