/*
 * Copyright (C) 2001-2004 by egnite Software GmbH
 * Copyright (c) 1989 by Carnegie Mellon University
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

/*!
 * \file net/lcpin.c
 * \brief PPP LCP input functions.
 *
 * \verbatim
 * $Id: lcpin.c 4600 2012-09-13 09:35:55Z haraldkipp $
 * \endverbatim
 */

#include <net/if_var.h>
#include <dev/ppp.h>
#include <sys/types.h>
#include <netinet/if_ppp.h>
#include <netinet/ppp_fsm.h>
#include <netinet/in.h>
#include <string.h>
#include <dev/usart.h>
#include <io.h>

/*!
 * \addtogroup xgLCP
 *
 */
/*@{*/

extern uint32_t new_magic;

/*
 * Received Configure-Request.
 */
static INLINE void LcpRxConfReq(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;
    int rc = XCP_CONFACK;
    XCPOPT *xcpo;
    uint16_t xcpl;
    XCPOPT *xcpr;
    uint16_t xcps;
    uint16_t len = 0;
    uint16_t sval;
    uint_fast8_t i;

    switch (dcb->dcb_lcp_state) {
    case PPPS_CLOSED:
        /*
         * Go away, we're closed.
         */
        NutNetBufFree(nb);
        NutLcpOutput(dev, XCP_TERMACK, id, 0);
        return;

    case PPPS_CLOSING:
    case PPPS_STOPPING:
        /*
         * Silently ignore configuration requests while going down.
         */
        NutNetBufFree(nb);
        return;

    case PPPS_OPENED:
        /*
         * Go down and restart negotiation.
         */
        IpcpLowerDown(dev);
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        break;

    case PPPS_STOPPED:
        /*
         * Negotiation started by our peer.
         */
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;
    }

    /*
     * Check if there is anything to reject.
     */
    xcpo = nb->nb_ap.vp;
    xcpl = nb->nb_ap.sz;
    xcpr = nb->nb_ap.vp;
    xcps = 0;
    while (xcpl >= 2) {
        len = xcpo->xcpo_len;
        if (len > xcpl)
            len = xcpl;
        else {
            switch (xcpo->xcpo_type) {
            case LCP_MRU:
                if (xcpo->xcpo_len == 4)
                    len = 0;
                break;
            case LCP_MAGICNUMBER:
            case LCP_ASYNCMAP:
                if (xcpo->xcpo_len == 6)
                    len = 0;
                break;
            case LCP_AUTHTYPE:
                if (xcpo->xcpo_len >= 4)
                    len = 0;
                break;
            case LCP_PCOMPRESSION:
                break;
            case LCP_ACCOMPRESSION:
                break;
            }
        }

        if (len) {
            if (xcpr != xcpo) {
                xcpr->xcpo_type = xcpo->xcpo_type;
                xcpr->xcpo_len = len;
                for (i = 0; i < len - 2; i++)
                    /* bug fix by Michel Hendriks. Thanks! */
                    xcpr->xcpo_.uc[i] = xcpo->xcpo_.uc[i];
            }
            xcpr = (XCPOPT *) ((char *) xcpr + len);
            xcps += len;
        }
        xcpl -= xcpo->xcpo_len;
        xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
    }

    if (xcps) {
        nb->nb_ap.sz = xcps;
        rc = XCP_CONFREJ;
    }

    /*
     * Check if there is anything to negotiate.
     */
    else {
        xcpo = nb->nb_ap.vp;
        xcpl = nb->nb_ap.sz;
        xcpr = nb->nb_ap.vp;
        xcps = 0;
        len = 0;
        while (xcpl >= 2) {
            switch (xcpo->xcpo_type) {
            case LCP_MRU:
                if ((sval = htons(xcpo->xcpo_.us)) < MIN_LCPMRU) {
                    len = 4;
                    xcpr->xcpo_.us = ntohs(MIN_LCPMRU);
                } else
                    dcb->dcb_rem_mru = sval;
                break;
            case LCP_ASYNCMAP:
                dcb->dcb_accm = ntohl(xcpo->xcpo_.ul);
                break;
            case LCP_AUTHTYPE:
                if (htons(xcpo->xcpo_.us) != PPP_PAP) {
                    len = 4;
                    xcpr->xcpo_.us = htons(PPP_PAP);
                }
                break;
            case LCP_MAGICNUMBER:
                if (xcpo->xcpo_.ul == dcb->dcb_loc_magic || xcpo->xcpo_.ul == dcb->dcb_neg_magic) {
                    dcb->dcb_rem_magic = new_magic;
                    len = 6;
                    xcpr->xcpo_.ul = dcb->dcb_rem_magic;
                }
                break;
            case LCP_PCOMPRESSION:
                dcb->dcb_compr |= PPP_PFC;
                break;
            case LCP_ACCOMPRESSION:
                dcb->dcb_compr |= PPP_ACFC;
                break;
            }

            if (len) {
                if (xcpr != xcpo) {
                    xcpr->xcpo_type = xcpo->xcpo_type;
                    xcpr->xcpo_len = len;
                }
                xcpr = (XCPOPT *) ((char *) xcpr + len);
                xcps += len;
                len = 0;
            }
            xcpl -= xcpo->xcpo_len;
            xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
        }
        if (xcps) {
            nb->nb_ap.sz = xcps;
            rc = XCP_CONFNAK;
        }
    }

    NutLcpOutput(dev, rc, id, nb);

    if (rc == XCP_CONFACK) {
        if (dcb->dcb_lcp_state == PPPS_ACKRCVD) {
            dcb->dcb_lcp_state = PPPS_OPENED;
            _ioctl(dcb->dcb_fd, HDLC_SETTXACCM, &(dcb->dcb_accm) );
            if (dcb->dcb_auth == PPP_PAP)
                PapTxAuthReq(dev, ++dcb->dcb_reqid);
            else
                IpcpLowerUp(dev);
        } else
            dcb->dcb_lcp_state = PPPS_ACKSENT;
        dcb->dcb_lcp_naks = 0;
    } else if (dcb->dcb_lcp_state != PPPS_ACKRCVD)
        dcb->dcb_lcp_state = PPPS_REQSENT;
}

/*
 * Configure-Ack received.
 * Never called in INITIAL or STARTING phase.
 */
static INLINE void LcpRxConfAck(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;
    XCPOPT *xcpo;
    uint16_t xcpl;

    /*
     * Check if this is a valid ack.
     */
    if (id == dcb->dcb_reqid && dcb->dcb_acked == 0) {
        dcb->dcb_acked = 1;
        xcpo = nb->nb_ap.vp;
        xcpl = nb->nb_ap.sz;
        while (xcpl >= 2) {
            switch (xcpo->xcpo_type) {
            case LCP_MRU:
                if (htons(xcpo->xcpo_.us) != 1500)
                    dcb->dcb_acked = 0;
                break;
            case LCP_ASYNCMAP:
                //if(ntohl(xcpo->xcpo_.ul) != )
                //    dcb->dcb_acked = 0;
                break;
            case LCP_AUTHTYPE:
                if (htons(xcpo->xcpo_.us) != dcb->dcb_auth)
                    dcb->dcb_acked = 0;
                break;
            case LCP_MAGICNUMBER:
                if (xcpo->xcpo_.ul == dcb->dcb_neg_magic) {
                    dcb->dcb_loc_magic = dcb->dcb_neg_magic;
                } else {
                    dcb->dcb_acked = 0;
                }
                break;
            case LCP_PCOMPRESSION:
                dcb->dcb_acked = 0;
                break;
            case LCP_ACCOMPRESSION:
                dcb->dcb_acked = 0;
                break;
            }
            xcpl -= xcpo->xcpo_len;
            xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
        }
    }

    /*
     * We don't need the NETBUF any more.
     */
    NutNetBufFree(nb);

    /*
     * Ignore invalid acks.
     */
    if (dcb->dcb_acked == 0)
        return;

    switch (dcb->dcb_lcp_state) {
    case PPPS_CLOSED:
    case PPPS_STOPPED:
        /*
         * Go away, we're closed.
         */
        NutLcpOutput(dev, XCP_TERMACK, id, 0);
        break;

    case PPPS_REQSENT:
        dcb->dcb_lcp_state = PPPS_ACKRCVD;
        dcb->dcb_retries = 0;
        break;

    case PPPS_ACKRCVD:
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;

    case PPPS_ACKSENT:
        /*
         * ACK sent and ACK received.
         */
        dcb->dcb_lcp_state = PPPS_OPENED;
        _ioctl(dcb->dcb_fd, HDLC_SETTXACCM, &(dcb->dcb_accm) );

        if (dcb->dcb_auth == PPP_PAP)
            PapTxAuthReq(dev, ++dcb->dcb_reqid);
        else
            IpcpLowerUp(dev);
        break;

    case PPPS_OPENED:
        /*
         * Go down and restart negotiation.
         */
        IpcpLowerDown(dev);
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;
    }
}

/*
 * Configure-Nak or Configure-Reject received.
 */
static INLINE void LcpRxConfNakRej(NUTDEVICE * dev, uint8_t id, NETBUF * nb, uint8_t rejected)
{
    PPPDCB *dcb = dev->dev_dcb;

    XCPOPT *xcpo;
    uint16_t xcpl;

    /*
     * Ignore, if we are not expecting this id.
     */
    if (id != dcb->dcb_reqid || dcb->dcb_acked) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * TODO: Process acked options.
     */
    dcb->dcb_acked = 1;

    xcpo = nb->nb_ap.vp;
    xcpl = nb->nb_ap.sz;
    while (xcpl >= 2) {
        xcpl -= xcpo->xcpo_len;
        xcpo = (XCPOPT *) ((char *) xcpo + xcpo->xcpo_len);
    }

    NutNetBufFree(nb);

    switch (dcb->dcb_lcp_state) {
    case PPPS_CLOSED:
    case PPPS_STOPPED:
        /*
         * Go away, we're closed.
         */
        NutLcpOutput(dev, XCP_TERMACK, id, 0);
        break;

    case PPPS_REQSENT:
    case PPPS_ACKSENT:
        /* They didn't agree to what we wanted - try another request */
        LcpTxConfReq(dev, ++dcb->dcb_reqid, rejected);
        break;

    case PPPS_ACKRCVD:
        /* Got a Nak/reject when we had already had an Ack?? oh well... */
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;

    case PPPS_OPENED:
        /*
         * Go down and restart negotiation.
         */
        IpcpLowerDown(dev);
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;
    }
}

/*
 * Terminate-Request received.
 */
static INLINE void LcpRxTermReq(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    NutNetBufFree(nb);

    switch (dcb->dcb_lcp_state) {
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
        dcb->dcb_lcp_state = PPPS_REQSENT;      /* Start over but keep trying */
        break;

    case PPPS_OPENED:
        IpcpLowerDown(dev);
        dcb->dcb_lcp_state = PPPS_STOPPING;
        break;
    }
    NutLcpOutput(dev, XCP_TERMACK, id, 0);
}

/*
 * Terminate-Ack received.
 */
static INLINE void LcpRxTermAck(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    NutNetBufFree(nb);

    switch (dcb->dcb_lcp_state) {
    case PPPS_CLOSING:
        dcb->dcb_lcp_state = PPPS_CLOSED;
        break;
    case PPPS_STOPPING:
        dcb->dcb_lcp_state = PPPS_STOPPED;
        break;

    case PPPS_ACKRCVD:
        dcb->dcb_lcp_state = PPPS_REQSENT;
        break;

    case PPPS_OPENED:
        IpcpLowerDown(dev);
        LcpTxConfReq(dev, ++dcb->dcb_reqid, 0);
        break;
    }
}

/*
 * Peer doesn't speak this protocol.
 *
 * Treat this as a catastrophic error (RXJ-).
 */
void LcpRxProtRej(NUTDEVICE * dev)
{
    PPPDCB *dcb = dev->dev_dcb;

    switch (dcb->dcb_lcp_state) {
    case PPPS_CLOSING:
    case PPPS_CLOSED:
        dcb->dcb_lcp_state = PPPS_CLOSED;
        break;

    case PPPS_STOPPING:
    case PPPS_REQSENT:
    case PPPS_ACKRCVD:
    case PPPS_ACKSENT:
    case PPPS_STOPPED:
        dcb->dcb_lcp_state = PPPS_STOPPED;
        break;

    case PPPS_OPENED:
        IpcpLowerDown(dev);
        NutIpcpOutput(dev, XCP_TERMREQ, dcb->dcb_reqid, 0);
        dcb->dcb_lcp_state = PPPS_STOPPING;
        break;
    }
}

/*
 * Received a Code-Reject.
 */
static INLINE void LcpRxCodeRej(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    NutNetBufFree(nb);

    if (dcb->dcb_lcp_state == PPPS_ACKRCVD)
        dcb->dcb_lcp_state = PPPS_REQSENT;
}

/*
 * Received an Echo-Request.
 */
static INLINE void LcpRxEchoReq(NUTDEVICE * dev, uint8_t id, NETBUF * nb)
{
    PPPDCB *dcb = dev->dev_dcb;

    if (dcb->dcb_lcp_state != PPPS_OPENED) {
        NutNetBufFree(nb);
    } else {
        /* Use local magic number. */
        memcpy(nb->nb_ap.vp, &dcb->dcb_loc_magic, sizeof(uint32_t));
        NutLcpOutput(dev, LCP_ERP, id, nb);
    }
}



/*!
 * \brief Handle incoming LCP packets.
 *
 * Packets not destined to us or packets with unsupported
 * address type or item length are silently discarded.
 *
 * \note This routine is called by the Ethernet layer on
 *       incoming ARP packets. Applications typically do
 *       not call this function.
 *
 * \param dev Identifies the device that received the packet.
 * \param nb  Pointer to a network buffer structure containing
 *            the ARP packet.
 */
void NutLcpInput(NUTDEVICE * dev, NETBUF * nb)
{
    XCPHDR *lcp;
    PPPDCB *dcb = dev->dev_dcb;
    uint16_t len;

    /*
     * Discard packets with illegal lengths.
     */
    if (nb->nb_nw.sz < sizeof(XCPHDR)) {
        NutNetBufFree(nb);
        return;
    }
    lcp = (XCPHDR *) nb->nb_nw.vp;
    if ((len = htons(lcp->xch_len)) < sizeof(XCPHDR) || len > nb->nb_nw.sz) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * Discard all packets while we are in initial or starting state.
     */
    if (dcb->dcb_lcp_state == PPPS_INITIAL || dcb->dcb_lcp_state == PPPS_STARTING) {
        NutNetBufFree(nb);
        return;
    }

    /*
     * Split the LCP packet.
     */
    nb->nb_ap.vp = lcp + 1;
    nb->nb_ap.sz = htons(lcp->xch_len) - sizeof(XCPHDR);

    /*
     * Action depends on code.
     */
    switch (lcp->xch_code) {
    case XCP_CONFREQ:
        LcpRxConfReq(dev, lcp->xch_id, nb);
        break;

    case XCP_CONFACK:
        LcpRxConfAck(dev, lcp->xch_id, nb);
        break;

    case XCP_CONFNAK:
        LcpRxConfNakRej(dev, lcp->xch_id, nb, 0);
        break;

    case XCP_CONFREJ:
        LcpRxConfNakRej(dev, lcp->xch_id, nb, 1);
        break;

    case XCP_TERMREQ:
        LcpRxTermReq(dev, lcp->xch_id, nb);
        break;

    case XCP_TERMACK:
        LcpRxTermAck(dev, lcp->xch_id, nb);
        break;

    case XCP_CODEREJ:
        LcpRxCodeRej(dev, lcp->xch_id, nb);
        break;

    case LCP_ERQ:
        LcpRxEchoReq(dev, lcp->xch_id, nb);
        break;

    case LCP_ERP:
    case LCP_DRQ:
        /* Silently ignore echo responses and discard requests. */
        NutNetBufFree(nb);
        break;

    default:
        /*
         * TODO: Send code reject.
         */
        NutNetBufFree(nb);
        break;
    }
}

/*@}*/
