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
 * -
 * Portions Copyright (c) 1983, 1993 by
 *  The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _SYS_TYPES_H_

#ifndef _SYS_VIRTUAL_TYPES_H_
#define _SYS_VIRTUAL_TYPES_H_

#ifndef _SYS_TYPES_H
#define _SYS_TYPES_H_
#endif

/*!
 * \file sys/types.h
 * \brief Nut/OS type declarations.
 */

#include <compiler.h>

// Definition of size_t
#ifdef __cplusplus
#include <cstddef>
#else
#include <stddef.h>
#endif

/*!
 * \weakgroup xgNutOS
 */
/*@{*/

#ifndef NUTMEM_ALIGNMENT
#if defined(__CORTEX__)
#define NUTMEM_ALIGNMENT        4
#elif defined(__arm__)
#define NUTMEM_ALIGNMENT        4
#elif defined(__AVR32__)
#define NUTMEM_ALIGNMENT        4
#else
#define NUTMEM_ALIGNMENT        sizeof(int)
#endif
#endif

/*!
 * \brief Return the next lower aligned value.
 */
#define NUTMEM_BOTTOM_ALIGN(s)  ((s) & ~(NUTMEM_ALIGNMENT - 1))

/*!
 * \brief Return the next higher aligned value.
 */
#define NUTMEM_TOP_ALIGN(s)     NUTMEM_BOTTOM_ALIGN((s + (NUTMEM_ALIGNMENT - 1)))


#ifndef __NUT_EMULATION__

/*! \brief Unsigned 8-bit value.
 *
 * \todo We should switch to stdint.h as suggested by Dirk Kaufmann. See
 *       feature request #1282721.
 */
    typedef unsigned char u_char;

/*! \brief Unsigned 16-bit value. */
    typedef unsigned short u_short;

/*! \brief Unsigned int value. */
/* Warning: size is highly architecture/compiler dependant! */
    typedef unsigned int u_int;

/*! \brief Unsigned 32-bit value */
    typedef unsigned long u_long;

/*! \brief Unsigned 64-bit value */
    typedef unsigned long long u_longlong;

#endif                          /* unix emulation */

/*! \brief Void pointer */
typedef void *HANDLE;

/*!
 * \brief Unsigned register type.
 *
 * The size of this type is equal to the size of a register,
 * the hardware datapath or whatever might fit to give optimum
 * performance for values from 0 to 255.
 *
 * Typically 8 bit CPUs will use unsigned characters, 16 bit
 * CPUs will use unsigned shorts etc.
 */
#if defined(__arm__)
    typedef unsigned short ureg_t;
#elif defined(__CORTEX__)
    typedef unsigned short ureg_t;
#elif defined(__AVR32__)
    typedef unsigned long ureg_t;
#else
    typedef unsigned short ureg_t;
#endif

/*!
 * \brief Signed register type.
 *
 * Similar to ureg_t, but for signed values from -128 to +127.
 */
#if defined(__arm__)
    typedef unsigned short reg_t;
#elif defined(__CORTEX__)
    typedef unsigned short reg_t;
#elif defined(__AVR32__)
    typedef unsigned short reg_t;
#else
    typedef unsigned short reg_t;
#endif

/*!
 * \brief Unsigned pointer value type.
 *
 * The size of this type is at least the size of a memory pointer.
 * For CPUs with 16 address bits this will be an unsigned short.
 */
#if defined(__GNUC__)
/*
 * For remaining MCUs GCC is assumed where __PTRDIFF_TYPE__ macro is defined
 */
    typedef unsigned __PTRDIFF_TYPE__ uptr_t;
#else
    typedef unsigned int uptr_t;
#endif

#define __byte_swap2(val)           \
    ((((val) & 0xff) << 8) |        \
     (((val) & 0xff00) >> 8))

#define __byte_swap4(val)           \
    ((((val) & 0xff) << 24) |       \
     (((val) & 0xff00) << 8) |      \
     (((val) & 0xff0000) >> 8) |    \
     (((val) & 0xff000000) >> 24))

    /*!
 * \brief Convert short value from host to network byte order.
 */
#ifndef __BIG_ENDIAN__
#define htons(x) __byte_swap2(x)
#else
#define htons(x) (x)
#endif

/*!
 * \brief Convert long value from host to network byte order.
 */
#ifndef __BIG_ENDIAN__
#define htonl(x) __byte_swap4(x)
#else
#define htonl(x) (x)
#endif

/*!
 * \brief Convert short value from network to host byte order.
 */
#ifndef __BIG_ENDIAN__
#define ntohs(x) __byte_swap2(x)
#else
#define ntohs(x) (x)
#endif

/*!
 * \brief Convert long value from network to host byte order.
 */
#ifndef __BIG_ENDIAN__
#define ntohl(x) __byte_swap4(x)
#else
#define ntohl(x) (x)
#endif

/*@}*/

#endif                          /* #ifndef _SYS_VIRTUAL_TYPES_H_ */

#endif                          /* #ifndef _SYS_TYPES_H_ */
