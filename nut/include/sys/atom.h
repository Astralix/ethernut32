#ifndef _SYS_ATOM_H_
#define _SYS_ATOM_H_

/*
 * Copyright (C) 2001-2005 by egnite Software GmbH. All rights reserved.
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

#include <cfg/os.h>
#include <sys/types.h>
#include <stdint.h>

#ifdef NUTTRACER_CRITICAL
#include <sys/tracer.h>
#endif

/*
 * The following macros must at least be defined for each supported platform.
 *
 * NutEnterCritical()
 * Starts a critical section.
 * The global interrupt flag must be disabled to protect the folowing code from
 * interruption. The state of the global interrupt flag must be saved, to
 * later restore.
 *
 * NutExitCritical()
 * Finalize a critical section. Must be used only at the end of a critical section
 * to restore the global interrupt flag to the state saved by NutEnterCritical()
 *
 * NutJumpOutCritical()
 * Macro for early leaving the critical section. Must be used in the middle on a
 * critical section if you want to terminate the critical section.
 *
 */

#if defined(__arm__) && !defined(__CORTEX__)
#include <arch/arm/atom.h>
#elif defined(__arm__) && defined(__CORTEX__)
#include <arch/cm3/atom.h>
#elif defined(__AVR32__)
#include <arch/avr32/atom.h>
#endif

#endif
