#ifndef _SYS_TIMER_H
#define _SYS_TIMER_H

/*
 * Copyright (C) 2001-2006 by egnite Software GmbH. All rights reserved.
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
 * -
 * Portions Copyright (C) 2000 David J. Hudson <dave@humbug.demon.co.uk>
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You can redistribute this file and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software Foundation;
 * either version 2 of the License, or (at your discretion) any later version.
 * See the accompanying file "copying-gpl.txt" for more details.
 *
 * As a special exception to the GPL, permission is granted for additional
 * uses of the text contained in this file.  See the accompanying file
 * "copying-liquorice.txt" for details.
 */

#include <sys/types.h>
#include <arch/timer.h>

#include <time.h>
#include <stdint.h>

/*!
 * \file sys/timer.h
 * \brief Timer management definitions.
 */

/*!
 * \brief Timer type.
 */
typedef struct _NUTTIMERINFO NUTTIMERINFO;

/*!
 * \struct _NUTTIMERINFO timer.h sys/timer.h
 * \brief Timer information structure.
 */
struct _NUTTIMERINFO {
    /*! \brief Link to next timer.
     */
    NUTTIMERINFO *tn_next;
    /*! \brief Link to previous timer.
     */
    NUTTIMERINFO *tn_prev;
    /*! \brief Number of system ticks.
     *  Set to zero on one-shot timers.
     */
    uint32_t tn_ticks;
    /*! \brief Decremented by one on each system tick intervall.
     */
    uint32_t tn_ticks_left;
    /*! \brief Callback function.
     */
    void (*tn_callback)(HANDLE, void *);
    /*! \brief Argument pointer passed to callback function.
     */
    volatile void *tn_arg;
};

extern NUTTIMERINFO* nutTimerList;

/*! \brief The system_time struct holds the seconds and microseconds since
 *         system startup
 */
extern volatile struct timeval system_time;


#define TM_ONESHOT  0x01

#define NUT_CACHE_LVALID        0x80000000UL

/* Set defaults. */
#ifndef NUT_HWCLK_CPU
#define NUT_HWCLK_CPU           0
#endif

#ifndef NUT_HWCLK_PERIPHERAL
#define NUT_HWCLK_PERIPHERAL    NUT_HWCLK_CPU
#endif

//TODO: Implement this via configuration tools?
#ifndef NUT_HWCLK_MAX
  #if defined(MCU_STM32F10X)
    #define NUT_HWCLK_MAX           NUT_HWCLK_PCLK2
  #elif defined(MCU_STM32L1XX)
    #define NUT_HWCLK_MAX           NUT_HWCLOCK_ADC
  #else
    #define NUT_HWCLK_MAX           NUT_HWCLK_PERIPHERAL
  #endif
#endif

/*
 * Functions used by the kernel.
 */
extern void NutTimerIntr(void *arg);
extern void NutTimerInit(void);
extern NUTTIMERINFO * NutTimerCreate(uint32_t ticks, void (*callback) (HANDLE, void *), void *arg, uint8_t flags);
extern void NutTimerInsert(NUTTIMERINFO * tn);
extern void NutTimerProcessElapsed(void);

/*
 * API declarations.
 */
extern void NutSleep(uint32_t ms);
extern void NutDelay(uint8_t ms);
extern void NutMicroDelay(uint32_t us);

extern uint32_t NutGetTickCount(void);
extern uint32_t NutGetSeconds(void);
extern uint32_t NutGetMillis(void);

extern HANDLE NutTimerStart(uint32_t ms, void (*callback)(HANDLE, void *), void *arg, uint8_t flags);
extern HANDLE NutTimerStartTicks(uint32_t ticks, void (*callback) (HANDLE, void *), void *arg, uint8_t flags);
extern void NutTimerStop(HANDLE handle);

/*
 * Clock frequencies.
 */
#if NUT_HWCLK_MAX
/* More than 1 clock available: Implement function. */
extern uint32_t NutClockGet(int idx);
#else
/* Only 1 clock available: Map to CPU clock. */
#define NutClockGet(i)          NutGetCpuClock()
#endif

extern int NutClockSet(int idx, uint32_t freq);

/* On some platforms the clock query functions may be defined by
** a preprocessor macro to avoid function call overhead. */
#if !defined(NutGetCpuClock)
extern uint32_t NutGetCpuClock(void);
#endif

#endif
