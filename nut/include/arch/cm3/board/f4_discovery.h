/*
 * Copyright 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/board/f4_discovery.h
 * \brief STM32f4_Discovery board specific settings.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */
/* Orange LED*/
#ifndef LED1_PORT
#define LED1_PORT NUTGPIO_PORTD
#endif
#ifndef LED1_PIN
#define LED1_PIN 13
#endif
/* GREEN LED*/
#ifndef LED2_PORT
#define LED2_PORT NUTGPIO_PORTD
#endif
#ifndef LED2_PIN
#define LED2_PIN 12
#endif
/* RED LED*/
#ifndef LED3_PORT
#define LED3_PORT NUTGPIO_PORTD
#endif
#ifndef LED3_PIN
#define LED3_PIN 14
#endif
/* BLUE LED*/
#ifndef LED4_PORT
#define LED4_PORT NUTGPIO_PORTD
#endif
#ifndef LED4_PIN
#define LED4_PIN 15
#endif

#ifndef DEF_OWIBUS
#if 1
#define DEF_OWIBUS owiBus0Stm32Tim
#include <arch/cm3/stm/owibus_stm32tim.h>
#else
#define DEF_OWIBUS owiBus0Gpio
#include <dev/owibus_gpio.h>
#endif
#endif

/* Some extension*/
#ifndef OWI_PORT
#define OWI_PORT NUTGPIO_PORTB
#endif
#ifndef OWI_PIN
#define OWI_PIN  1
#endif
#ifndef OWI_TIMER
#define OWI_TIMER NUTTIMER3
#endif
#ifndef OWI_CHANNEL
#define OWI_CHANNEL 4
#endif

#ifndef DEV_UART
#define DEV_UART devUsartStm32_2
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME  devUsartStm32_2.dev_name
#endif

#ifndef SBBI0_MISO_BIT
#define SBBI0_MISO_BIT 6
#endif
#ifndef SBBI0_MISO_PORT
#define SBBI0_MISO_PORT NUTGPIO_PORTA
#endif
#ifndef SBBI0_MOSI_BIT
#define SBBI0_MOSI_BIT 7
#endif
#ifndef SBBI0_MOSI_PORT
#define SBBI0_MOSI_PORT NUTGPIO_PORTA
#endif
#ifndef SBBI0_SCK_BIT
#define SBBI0_SCK_BIT 5
#endif
#ifndef SBBI0_SCK_PORT
#define SBBI0_SCK_PORT NUTGPIO_PORTA
#endif
#ifndef SBBI0_CS0_BIT
#define SBBI0_CS0_BIT 3
#endif
#ifndef SBBI0_CS0_PORT
#define SBBI0_CS0_PORT NUTGPIO_PORTE
#endif

#define AUDIO_RST_PORT NUTGPIO_PORTD
#define AUDIO_RST_PIN  4

#ifndef DEV_CONSOLE
#define DEV_CONSOLE devUsartStm32_2
#endif

#ifndef DEV_CONSOLE_NAME
#define DEV_CONSOLE_NAME devUsartStm32_2.dev_name
#endif

#ifndef I2CBUS1_SDA_PIN
#define I2CBUS1_SDA_PIN 9
#endif

#ifndef I2CBUS1_SCL_PIN
#define I2CBUS1_SCL_PIN 6
#endif

#ifndef DEF_TWIBUS
#define  DEF_TWIBUS Stm32TwiBus_1
#endif

#ifndef DEF_I2CBUS
#if 0
#define DEF_I2CBUS i2cBus1Stm32
#include <dev/i2cbus_stm32.h>
#else
#define DEF_I2CBUS i2cBus0Gpio
#include <dev/i2cbus_gpio.h>
#endif
#endif

#ifndef DEF_CANBUS
#define DEF_CANBUS Stm32CanBus1
#endif

#ifndef DEF_CANBUS_SLAVE
#define  DEF_CANBUS_SLAVE Stm32CanBus1C
#endif
