#ifndef _SYS_EVENT_H_
#define _SYS_EVENT_H_

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

#include <sys/thread.h>

/*!
 * \file sys/event.h
 * \brief Event management definitions.
 */

/*!
 * \addtogroup xgEvent
 */
/*@{*/

/*!
 * \brief Signaled state definition.
 *
 * The root of an event queue is set to this value if an event
 * is posted to an empty queue. As this may happen during
 * interrupts, the root of an event queue must be considered
 * volatile.
 *
 * Timer handles in the THREADINFO structure are set to this value
 * if a timeout occured while waiting for an event.
 */
#define SIGNALED    ((void *)-1)

/*!
 * \brief Infinite waiting time definition.
 *
 * Applications should use this value to disable timeout monitoring
 * while waiting for an event.
 */
#define NUT_WAIT_INFINITE   0

/*!
 * \brief Post an event to a specified queue from interrupt context.
 *
 * Wake up the thread with the highest priority waiting on the
 * specified queue. This function is explicitly provided for IRQ
 * handlers to wakeup waiting user threads.
 *
 * Internally a counter is used to keep track of the posted events.
 * This counter will be examined when the currently running thread is
 * ready to release the CPU.
 *
 * \note When calling this function, interrupt routines will change
 *       the root of an empty event queue to SIGNALED.
 *
 * \param qp Identifies the queue an event is posted to.
 *
 */
#define NutEventPostFromIrq(qp)     \
{                                   \
    if (*qp == 0) {                 \
        *qp = SIGNALED;             \
    }                               \
    else if (*qp != SIGNALED) {     \
        NUTTHREADINFO *tp = (NUTTHREADINFO *)(*qp);    \
        tp->td_qpec++;              \
    }                               \
}

/*@}*/

extern void NutEventTimeout(HANDLE timer, void *arg);

extern int NutEventWait(volatile HANDLE *qhp, uint32_t ms);
extern int NutEventWaitNext(volatile HANDLE *qhp, uint32_t ms);
extern int NutEventPostAsync(volatile HANDLE *qhp);
extern int NutEventPost(volatile HANDLE *qhp);
extern int NutEventBroadcastAsync(volatile HANDLE *qhp);
extern int NutEventBroadcast(volatile HANDLE *qhp);

#endif
