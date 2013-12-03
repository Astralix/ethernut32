/*
 * Copyright (C) 2000-2004 by ETH Zurich
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
 * THIS SOFTWARE IS PROVIDED BY ETH ZURICH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ETH ZURICH
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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

/* mutex.c - a nut/os implementation of recursive mutex functions
 *
 * 2004.05.06 Matthias Ringwald <matthias.ringwald@inf.ethz.ch>
 *
 */

#ifndef _SYS_MUTEX_H
#define _SYS_MUTEX_H

#include <compiler.h>
#include <sys/types.h>
#include <sys/thread.h>

/*!
 * \addtogroup xgMutex
 */
/*@{*/

/*!
 * \brief Recursive mutex type.
 */
typedef struct _MUTEX MUTEX;

/*!
 * \struct _MUTEX mutex.h sys/mutex.h
 * \brief Recursive mutex.
 *
 */
struct _MUTEX {
    HANDLE qhp;             /*!< \brief Queue to wait, if mutex is not free. */
    NUTTHREADINFO *thread;  /*!< \brief Thread who owns the mutex */
    uint16_t count;          /*!< \brief Lock counter. */
};

extern void NutMutexInit(MUTEX * mutex);
extern void NutMutexLock(MUTEX * mutex);
extern int NutMutexUnlock(MUTEX * mutex);
extern int NutMutexTrylock(MUTEX * mutex);
extern int NutMutexDestroy(MUTEX * mutex);

/*@}*/

#endif /* #ifndef _SYS_MUTEX_H */

