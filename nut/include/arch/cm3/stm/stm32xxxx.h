/*
 * Copyright (C) 2012-2013 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 *
 */

/*!
 * \file arch/cm3/stm/atm32xxxx.h
 * \brief Wrapper for the device dependant stm32XZxx.h files.
 * $Id: stm32_flash.h 3220 2010-11-12 13:04:17Z astralix $
 * \verbatim
 */

#include <cfg/arch.h>
#if defined(MCU_STM32F1)
#include <arch/cm3/stm/vendor/stm32f10x.h>
#elif defined(MCU_STM32L1)
#define MCU_STM32L1XX
#include <arch/cm3/stm/vendor/stm32l1xx.h>
#elif defined(MCU_STM32F2)
#include <arch/cm3/stm/vendor/stm32f2xx.h>
#elif defined(STM32F30X)
#include <arch/cm3/stm/vendor/stm32f30x.h>
#elif defined(STM32F37X)
#include <arch/cm3/stm/vendor/stm32f37x.h>
#elif defined(MCU_STM32F401)
#define STM32F401xx
#include <arch/cm3/stm/vendor/stm32f4xx.h>
#elif defined(MCU_STM32F40X)
#define STM32F40_41xxx
#include <arch/cm3/stm/vendor/stm32f4xx.h>
#elif defined(MCU_STM32F42X)
#define STM32F427_437xx
#include <arch/cm3/stm/vendor/stm32f4xx.h>
#else
#warning "Unknown STM32 family"
#endif
