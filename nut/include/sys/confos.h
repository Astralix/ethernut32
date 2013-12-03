#ifndef _SYS_CONFOS_H_
#define _SYS_CONFOS_H_

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
 */

/*!
 * \file sys/confos.h
 * \brief Header file for global system configuration.
 *
 * \verbatim
 *
 * $Log$
 * Revision 1.9  2009/01/16 17:03:02  haraldkipp
 * Configurable host name length. The *nix conditional is
 * no longer required as this will be handled in the nvmem
 * routines. NutLoadConfig will now set the virgin host
 * name, if no valid configuration is available. Cookie
 * and virgin host name are configurable too, but disabled
 * in the Configurator until we fixed the string value
 * problem. You may use UserConf.mk instead.
 *
 * \endverbatim
 */

#include <sys/types.h>
#include <cfg/eeprom.h>

#include <stdint.h>

/*!
 * \addtogroup xgConfOs
 */
/*@{*/

/*!
 * \brief Non-volatile memory location.
 *
 * Offset into non-volatile memory, where Nut/OS stores the system
 * configuration. The default may be overridden by the Configurator.
 */
#ifndef CONFOS_EE_OFFSET
#define CONFOS_EE_OFFSET    0
#endif

#ifndef CONFOS_EE_MAGIC
#define CONFOS_EE_MAGIC     "OS"
#endif

/*!
 * \brief Maximum number of characters allowed for hostname.
 *
 * Intentionally not MAXHOSTNAMELEN to avoid conflicts with existing
 * C libraries.
 */
#ifndef MAX_HOSTNAME_LEN
#define MAX_HOSTNAME_LEN    15
#endif

/*!
 * \brief Default hostname.
 *
 * Used on virgin systems without any valid configuration.
 */
#ifndef CONFOS_VIRGIN_HOSTNAME
#define CONFOS_VIRGIN_HOSTNAME  "nut32"
#endif

/*!
 * \brief Operating system configuration type.
 */
typedef struct _CONFOS CONFOS;

/*!
 * \struct _CONFOS confos.h sys/confos.h
 * \brief Operating system configuration structure.
 *
 * Applications may directly access the global variable \ref confos to
 * read or modify the current configuration.
 */
struct NUT_PACKED_TYPE _CONFOS {
    /*! \brief Size of this structure.
     *
     * Used by Nut/Net to verify, that the structure contents is valid
     * after reading it from non-volatile memory.
     */
    uint8_t size;

    /*! \brief Magic cookie.
     *
     * Contains CONFOS_EE_MAGIC.
     */
    uint8_t magic[sizeof(CONFOS_EE_MAGIC) - 1];

    /*! \brief Host name of the system.
     */
    char hostname[MAX_HOSTNAME_LEN + 1];
};

extern CONFOS confos;

extern int NutLoadConfig(void);
extern int NutSaveConfig(void);

/*@}*/

#endif
