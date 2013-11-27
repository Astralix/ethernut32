/*
 * Copyright 2013 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file arch/cm3/board/f3_discovery.h
 * \brief STM32F3_Discovery board specific settings.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */
/* LD4 NW BLUE*/
#ifndef HEARTBEAT_IDLE_PORT
#define HEARTBEAT_IDLE_PORT NUTGPIO_PORTE
#endif
#ifndef HEARTBEAT_IDLE_PIN
#define HEARTBEAT_IDLE_PIN 12
#endif
/* LD3 North RED*/
#ifndef LED1_PORT
#define LED1_PORT NUTGPIO_PORTE
#endif
#ifndef LED1_PIN
#define LED1_PIN 9
#endif
/* LD6 WEST BLUE*/
#ifndef LED2_PORT
#define LED2_PORT NUTGPIO_PORTE
#endif
#ifndef LED2_PIN
#define LED2_PIN 15
#endif

#ifndef DEV_UART
#define DEV_UART devUsartStm32_1
#endif
#ifndef DEV_UART_NAME
#define DEV_UART_NAME  devUsartStm32_1.dev_name
#endif

#ifndef DEF_CANBUS
#define DEF_CANBUS Stm32CanBus1
#endif

#ifndef DEF_CANBUS_SLAVE
#define  DEF_CANBUS_SLAVE Stm32CanBus1C
#endif

#ifndef DEF_I2CBUS
#if 1
#define DEF_I2CBUS i2cBus1Stm32
#include <dev/i2cbus_stm32.h>
#else
#define DEF_I2CBUS i2cBus0Gpio
#include <dev/i2cbus_gpio.h>
#endif
#endif

/* define RTC */

#ifndef RTC_CHIP
#include <dev/stm32_rtc.h>
#define RTC_CHIP rtcStm32
#endif
