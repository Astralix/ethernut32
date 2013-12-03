#ifndef _SYS_CONFNET_H_
#define _SYS_CONFNET_H_

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

#include <sys/types.h>
#include <cfg/eeprom.h>

#include <stdint.h>

/*!
 * \addtogroup xgConfNet
 */
/*@{*/

/*!
 * \brief Non-volatile memory location.
 *
 * Offset into non-volatile memory, where Nut/Net stores the network
 * configuration. The default may be overridden by the Configurator.
 */
#ifndef CONFNET_EE_OFFSET
#define CONFNET_EE_OFFSET   64
#endif

#ifndef CONFNET_MAX_IF
#define CONFNET_MAX_IF      1
#endif

/*!
 * \brief Network configuration type.
 */
typedef struct _CONFNET CONFNET;

/*!
 * \struct _CONFNET confnet.h sys/confnet.h
 * \brief Network configuration structure.
 *
 * Applications may directly access the global variable \ref confnet to
 * read or modify the current network configuration.
 */
struct NUT_PACKED_TYPE _CONFNET {
    /*! \brief Size of this structure.
     *
     * Used by Nut/Net to verify, that the structure contents is valid
     * after reading it from non-volatile memory.
     */
    uint8_t cd_size;

    /*! \brief Magic cookie.
     *
     * Contains the device name of the network interface.
     */
    char cd_name[9];

    /*! \brief Ethernet MAC address.
     *
     * Unique Ethernet address of the network interface.
     */
    uint8_t cdn_mac[6];

    /*! \brief Last used IP address.
     *
     * Each time Nut/Net receives an IP address during boot, it
     * will store the address in here.
     *
     * If no fixed IP address has been configured (cdn_cip_addr
     * contains 0.0.0.0) and if no DHCP server is available, then
     * Nut/Net will use this one, if it is not 0.0.0.0.
     */
    uint32_t cdn_ip_addr;

    /*! \brief IP netmask.
     *
     * The netmask is used to determine which machines are
     * available in the local network.
     */
    uint32_t cdn_ip_mask;

    /*! \brief Default route.
     *
     * Nut/Net will redirect IP packets to this node, if the
     * target IP is not located in the local network.
     */
    uint32_t cdn_gateway;

    /*! \brief Configured IP address.
     *
     * If this address is set to 0.0.0.0, Nut/Net will try
     * to obtain one from the DHCP server.
     */
    uint32_t cdn_cip_addr;
};

extern CONFNET confnet;

/*@}*/

#endif
