#ifndef _SYS_SOCK_VAR_H_
#define _SYS_SOCK_VAR_H_

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

#include <compiler.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/device.h>
#include <dev/netbuf.h>


/*!
 * \file sys/sock_var.h
 * \brief UDP and TCP socket interface definitions.
 */

/*********************************************************************\
 * UDP
\*********************************************************************/

/*!
 * \addtogroup xgUdpSocket
 */
/*@{*/

/*!
 * \brief UDP socket type.
 */
typedef struct udp_socket UDPSOCKET;

/*!
 * \brief UDP socket information structure.
 *
 * Applications should not rely on the content of this structure.
 * It may change without notice.
 */
struct udp_socket {
    UDPSOCKET *so_next;      /*!< \brief Link to next tcp socket structure. */
    uint16_t so_local_port;  /*!< \brief Local port number in net byte order. */
    NETBUF  *so_rx_nb;       /*!< \brief Received, but not read by application. */
    HANDLE  so_rx_rdy;       /*!< \brief Receiver event queue. */
    int     so_rx_cnt;       /*!< \brief Number of data bytes in the receive buffer. */
    int     so_rx_bsz;       /*!< \brief Receive buffer size. */

    uint16_t so_last_error;  /*!< \brief Last error of socket */
    uint32_t so_remote_addr; /*!< \brief Remote IP address in net byte order. Important just in case of an error */
    uint16_t so_remote_port; /*!< \brief Remote port number in net byte order. Important just in case of an error */
};

/*@}*/

extern int NutUdpInput(NUTDEVICE * dev, NETBUF *nb);
extern int NutUdpOutput(UDPSOCKET *sock, uint32_t dest, uint16_t port, NETBUF *nb);


/*********************************************************************\
 * TCP
\*********************************************************************/

/*!
 * \addtogroup xgTcpSocket
 */
/*@{*/

/*!
 * \brief TCP socket type.
 */
typedef struct tcp_socket TCPSOCKET;

/*!
 * \brief TCP socket information structure.
 *
 * Applications should not rely on the content of this structure.
 * It may change without notice.
 */
struct tcp_socket {
    TCPSOCKET *so_next;     /*!< \brief Link to next tcp socket structure. */
    void *so_device;        /*!< \brief Always zero. */
    uint8_t so_devtype;     /*!< \brief Device type, always IFTYP_TCPSOCK. */
    int (*so_devread) (TCPSOCKET *, void *, int); /*!< \brief Read from device. */
    int (*so_devwrite) (TCPSOCKET *, const void *, int); /*!< \brief Write to device. */
    int (*so_devioctl) (TCPSOCKET *, int, void *); /*!< \brief Driver control function. */

    uint16_t so_devocnt;     /*!< \brief Number of data bytes in output buffer. */
    uint8_t *so_devobuf;     /*!< \brief Pointer to output buffer. */
    uint16_t so_devobsz;     /*!< \brief Output buffer size. */

    volatile uint8_t  so_state;       /*!< \brief Connection state, see tcp_fsm.h */
    uint32_t  so_local_addr;  /*!< \brief Local IP address in net byte order. */
    uint16_t so_local_port;  /*!< \brief Local port number in net byte order. */
    uint32_t  so_remote_addr; /*!< \brief Remote IP address in net byte order. */
    uint16_t so_remote_port; /*!< \brief Remote port number in net byte order. */

    uint8_t  so_tx_flags;    /*!< \brief Flags used during transmissions - see below */
    uint32_t  so_tx_isn;      /*!< \brief Initial sequence number. */
    uint32_t  so_tx_una;      /*!< \brief Unacknowledged sequence number. */
    uint32_t  so_tx_nxt;      /*!< \brief Next sequence number to send. */
    uint32_t  so_tx_wl1;      /*!< \brief Sequence number of last window update. */
    uint32_t  so_tx_wl2;      /*!< \brief Acknowledged sequence of last window update. */
    uint16_t so_tx_win;      /*!< \brief Peer's receive window. */
    uint8_t  so_tx_dup;      /*!< \brief Duplicate ACK counter. */
    NETBUF  *so_tx_nbq;     /*!< \brief Network buffers waiting to be acknowledged. */
    HANDLE  so_tx_tq;       /*!< \brief Threads waiting for transmit buffer space. */

    uint32_t  so_rx_isn;      /*!< \brief Initial sequence number of remote. */
    uint32_t  so_rx_nxt;      /*!< \brief Next sequence number to receive. */
    uint16_t so_rx_win;      /*!< \brief Local receive window. */
    int     so_rx_cnt;      /*!< \brief Number of data bytes in the receive buffer. */
    int     so_rx_bsz;      /*!< \brief Receive buffer size. */
    int     so_rd_cnt;      /*!< \brief Number of bytes read from buffer top. */
    int_fast8_t so_rx_apc;  /*!< \brief Number of packets received in advance. */
    NETBUF  *so_rx_buf;     /*!< \brief Data waiting to be read by application. */
    HANDLE  so_rx_tq;       /*!< \brief Threads waiting for received data. */
    NETBUF  *so_rx_nbq;     /*!< \brief Network buffers received in advance. */

    uint16_t so_mss;         /*!< \brief MSS, limited by remote option or MTU. */

    uint32_t  so_rtt_seq;     /*!< \brief Sequence number for RTT calculation. */
    uint16_t so_rtto;        /*!< \brief Current retransmission timeout. */
    uint16_t so_retransmits; /*!< \brief Number of retransmits. */
    uint16_t so_time_wait;   /*!< \brief Time wait counter. */
    uint16_t so_retran_time; /*!< \brief Retransmit time counter. */
    uint16_t so_last_error;  /*!< \brief Last error code. */
    HANDLE  so_pc_tq;       /*!< \brief Listening thread. */
    HANDLE  so_ac_tq;       /*!< \brief Connecting thread. */

    uint32_t  so_read_to;     /*!< \brief Read timeout. */
    uint32_t  so_write_to;    /*!< \brief Write timeout. */
    uint32_t  so_oos_drop;    /*!< \brief Out of sequence dropped. */
};

/*
 * TCP send flags.
 */
#define SO_FIN      0x01    /*!< \brief Socket transmit flag. Send FIN after all data has been transmitted. */
#define SO_SYN      0x02    /*!< \brief Socket transmit flag. Send SYN first. */
#define SO_FORCE    0x08    /*!< \brief Socket transmit flag. Force sending ACK. */
#define SO_ACK      0x10    /*!< \brief Socket transmit flag. Send ACK. */

/*@}*/

#include <netinet/tcp_fsm.h>

extern int NutTcpOutput(TCPSOCKET *sock, const uint8_t *data, uint16_t size);
extern int NutTcpReject(NETBUF *nb);

#endif
