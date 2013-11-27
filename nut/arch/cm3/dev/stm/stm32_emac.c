/*
 * Copyright (C) 2012 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
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

#include <cfg/arch.h>
#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/phycfg.h>
#include <dev/board.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/heap.h>
#include <sys/thread.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <sys/confnet.h>

#include <netinet/if_ether.h>
#include <net/ether.h>
#include <net/if_var.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/phy.h>
#include <dev/stm32_emac.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_gpio.h>

/* WARNING: Variadic macros are C99 and may fail with C89 compilers. */
#ifdef NUTDEBUG
#include <stdio.h>
#define EMPRINTF(args,...) printf(args,##__VA_ARGS__)
#else
#define EMPRINTF(args,...)
#endif

#if SYSCLK_FREQ < 35000001L
#define ETH_MACMIIAR_CR_Div ETH_MACMIIAR_CR_Div16
#elif SYSCLK_FREQ < 60000001L
#define ETH_MACMIIAR_CR_Div ETH_MACMIIAR_CR_Div26
#elif SYSCLK_FREQ < 100000001L
#define ETH_MACMIIAR_CR_Div ETH_MACMIIAR_CR_Div42
#elif SYSCLK_FREQ < 150000001L
#define ETH_MACMIIAR_CR_Div ETH_MACMIIAR_CR_Div62
#elif SYSCLK_FREQ < 168000001L
#define ETH_MACMIIAR_CR_Div ETH_MACMIIAR_CR_Div102
#endif

/*
 * For the benefit of EMC the GPIO is run at the lowest speed
 * required to operate. For RMII this is 50 MHz for MII 25 MHz.
 */
#ifdef PHY_MODE_RMII
#define EMAC_GPIO_SPEED         GPIO_CFG_SPEED_FAST
#else
#define EMAC_GPIO_SPEED         GPIO_CFG_SPEED_MED
#endif

#ifndef NUT_THREAD_NICRXSTACK
#define NUT_THREAD_NICRXSTACK   1024
#endif

#ifndef EMAC_RX_BUFFERS
#define EMAC_RX_BUFFERS         32
#endif
#define EMAC_RX_BUFSIZ          128

#define EMAC_TX_BUFFERS         2
#ifndef EMAC_TX_BUFSIZ
#define EMAC_TX_BUFSIZ          1536
#endif

#ifndef NIC_PHY_ADDR
#define NIC_PHY_ADDR            1
#endif

/*
 * EMAC default GPIO definitions for alternate
 * functions which can be assigned to different ports on
 * STM32F2/F4 parts.
 */
#ifndef EMAC_CRS_PORT
#define EMAC_CRS_PORT        NUTGPIO_PORTA
#define EMAC_CRS_PIN         0
#endif

#ifndef EMAC_COL_PORT
#define EMAC_COL_PORT        NUTGPIO_PORTA
#endif

#ifndef EMAC_RXD2_PORT
#define EMAC_RXD2_PORT       NUTGPIO_PORTB
#define EMAC_RXD2_PIN        0
#endif

#ifndef EMAC_RXD3_PORT
#define EMAC_RXD3_PORT       NUTGPIO_PORTB
#define EMAC_RXD3_PIN        1
#endif

#ifndef EMAC_RX_ER_PORT
#define EMAC_RX_ER_PORT      NUTGPIO_PORTB
#endif

#ifndef EMAC_TXEN_PORT
#define EMAC_TXEN_PORT       NUTGPIO_PORTB
#endif

#ifndef EMAC_TXD0_PORT
#define EMAC_TXD0_PORT       NUTGPIO_PORTB
#define EMAC_TXD0_PIN        12
#endif

#ifndef EMAC_TXD1_PORT
#define EMAC_TXD1_PORT       NUTGPIO_PORTB
#define EMAC_TXD1_PIN        13
#endif

#ifndef EMAC_PPS_OUT_PORT
#undef EMAC_PPS_OUT_PORT /* Default is not to use a PPS signal */
#undef EMAC_PPS_OUT_PIN
#endif


 /*!
 * \brief Network interface controller information structure.
 */
struct _EMACINFO {
#ifdef NUT_PERFMON
    uint32_t ni_rx_packets;       /*!< Number of packets received. */
    uint32_t ni_tx_packets;       /*!< Number of packets sent. */
    uint32_t ni_overruns;         /*!< Number of packet overruns. */
    uint32_t ni_rx_frame_errors;  /*!< Number of frame errors. */
    uint32_t ni_rx_crc_errors;    /*!< Number of CRC errors. */
    uint32_t ni_rx_missed_errors; /*!< Number of missed packets. */
#endif
    HANDLE volatile ni_rx_rdy;  /*!< Receiver event queue. */
    HANDLE volatile ni_tx_rdy;  /*!< Transmitter event queue. */
    HANDLE ni_mutex;            /*!< Access mutex semaphore. */
    volatile int ni_tx_queued;  /*!< Number of packets in transmission queue. */
    volatile int ni_tx_quelen;  /*!< Number of bytes in transmission queue not sent. */
    volatile int ni_insane;     /*!< Set by error detection. */
    int ni_iomode;              /*!< 8 or 16 bit access. 32 bit is not supported. */
};

/*!
 * \brief Network interface controller information type.
 */
typedef struct _EMACINFO EMACINFO;

/*
 * TODO: Buffers and their descriptors should be part of the EMACINFO
 * structure. Actually there will be no dual Ethernet chip (sure?),
 * but just to keep the code clean.
 */
typedef struct _TBufDescriptor {
    uint32_t TDES0;
    uint32_t TDES1;
    uint32_t TDES2;
    uint32_t TDES3;
} TBufDescriptor_t;

typedef struct _RBufDescriptor {
    uint32_t RDES0;
    uint32_t RDES1;
    uint32_t RDES2;
    uint32_t RDES3;
} RBufDescriptor_t;

static volatile TBufDescriptor_t txBufTab[EMAC_TX_BUFFERS];
static volatile uint8_t txBuf[EMAC_TX_BUFFERS * EMAC_TX_BUFSIZ] __attribute__ ((aligned(8)));
static unsigned int txBufIdx = 0;

static volatile RBufDescriptor_t rxBufTab[EMAC_RX_BUFFERS];
static volatile uint8_t rxBuf[EMAC_RX_BUFFERS * EMAC_RX_BUFSIZ] __attribute__ ((aligned(8)));
static unsigned int rxBufIdx = 0;

#define TDES0_OWN           0x80000000  /* Bit set: descriptor is owned by DMA */
#define TDES0_IC            0x40000000  /* Set transmit interrupt after transmission */
#define TDES0_LS            0x20000000  /* Indicates last segment of the frame */
#define TDES0_FS            0x10000000  /* Indicates first segment of the frame */
#define TDES0_DC            0x08000000  /* Disable CRC on transmit frame */
#define TDES0_DP            0x04000000  /* Disable padding on short frames */
#define TDES0_TTSE          0x02000000  /* Transmit time stamp enable */
#define TDES0_CIC_DIS       0x00000000  /* Checksum insertion disabled */
#define TDES0_CIC_IP        0x00400000  /* Checksum only for ip header */
#define TDES0_CIC_FULL      0x00800000  /* Checksum for header and payload */
#define TDES0_CIC_ALL       0x00C00000  /* Checksum for header and payload + pseudo header */
#define TDES0_TER           0x00200000  /* Indicates final descriptor */
#define TDES0_TCH           0x00100000  /* Second address points to next descriptor */
#define TDES0_TTSS          0x00020000  /* Indicates time stamp captured for transmit frame */
#define TDES0_IHE           0x00010000  /* MAC indicates ip header error */
#define TDES0_ES            0x00008000  /* Error summary */
#define TDES0_JT            0x00004000  /* MAC indicates jabber timeout */
#define TDES0_FF            0x00002000  /* Indicates that DMA flushed the frame */
#define TDES0_IPE           0x00001000  /* MAC indicates ip payload error */
#define TDES0_LCA           0x00000800  /* Indicates loss of carrier */
#define TDES0_NC            0x00000400  /* Carrier sense signal not asserted */
#define TDES0_LCO           0x00000200  /* Late collision occured */
#define TDES0_EC            0x00000100  /* Indicates 16 successive collisions */
#define TDES0_VF            0x00000080  /* Indicates VLAN-type frame on transmission */
#define TDES0_CC_MASK       0x00000078  /* Mask for collision counter */
#define TDES0_CC_SHIFT      3           /* Number of right shifts to get collision number */
#define TDES0_ED            0x00000004  /* Indicates excessive deferral */
#define TDES0_UF            0x00000002  /* MAC indicates underflow error on DMA access */
#define TDES0_DB            0x00000001  /* MAC defers because of the presence of the carrier */

#define TDES1_TBS2_MASK     0x1FFF0000  /* Mask for Transmit buffer 2 size */
#define TDES1_TBS2_SHIFT    16          /* Number of right shifts to get transmit buffer 2 size */
#define TDES1_TBS1_MASK     0x00001FFF  /* Mask for Transmit buffer 1 size */
#define TDES1_TBS1_SHIFT    0           /* Number of right shifts to get transmit buffer 1 size */


#define RDES0_OWN           0x80000000  /* Bit set: Descriptor is owned by DMA */
#define RDES0_AFM           0x40000000  /* Indicates fail of destination address filter in MAC */
#define RDES0_FL_MASK       0x3FFF0000  /* Mask for frame length */
#define RDES0_FL_SHIFT      16          /* Number of right shifts to get frame length */
#define RDES0_ES            0x00008000  /* Error summary */
#define RDES0_DE            0x00004000  /* Descriptor error */
#define RDES0_SAF           0x00002000  /* Indicates fail of source address filter in MAC */
#define RDES0_LE            0x00001000  /* Indicates error in length of received frame */
#define RDES0_OE            0x00000800  /* Indicates buffer overflow */
#define RDES0_VLAN          0x00000400  /* Indicates a VLAN frame */
#define RDES0_FS            0x00000200  /* Indicates descriptor for first buffer of the frame */
#define RDES0_LS            0x00000100  /* Indicates descriptor for last buffer of the frame */
#define RDES0_IPHCE         0x00000080  /* Indicates IP header checksum error */
#define RDES0_LCO           0x00000040  /* Indicates a late collision */
#define RDES0_FT            0x00000020  /* Indicates that the received frame is of ethernet-type */
#define RDES0_RWT           0x00000010  /* Indicates receive watchdog timeout */
#define RDES0_RE            0x00000008  /* Indicates receive error during reception */
#define RDES0_DBE           0x00000004  /* Indicates non-integer multiple of bytes in frame */
#define RDES0_CE            0x00000002  /* Indicates a CRC error on received frame */
#define RDES0_PCE           0x00000001  /* Indicates payload checksum error */

#define RDES1_DIC           0x80000000  /* Disable interrupt on completed reception of frame */
#define RDES1_RBS2_MASK     0x1FFF0000  /* Receive buffer 2 size */
#define RDES1_RBS2_SHIFT    16          /* Number of right shifts to get received buffer 2 size */
#define RDES1_RER           0x00008000  /* End of ring: final descriptor of the list */
#define RDES1_RCH           0x00004000  /* Second address points to next descriptor */
#define RDES1_RBS1_MASK     0x00001FFF  /* Receiver buffer 1 size */
#define RDES1_RBS1_SHIFT    0           /* Number of right shifts to get received buffer 1 size */

/*!
 * \brief Read contents of PHY register.
 *
 * \param reg PHY register number.
 *
 * \return Contents of the specified register.
 */
static uint16_t phy_inw(uint8_t reg)
{
    uint32_t tempReg;
    uint16_t val = 0;

    /* PHY read command. */
    tempReg = inr(&(ETH->MACMIIAR));
    tempReg &= 0xffff0020;
    tempReg |= ((NIC_PHY_ADDR) << 11) | ((reg & 0x1F) << 6) | ETH_MACMIIAR_MB| ETH_MACMIIAR_CR_Div;
    outr(&(ETH->MACMIIAR), tempReg);

    /* Wait until PHY logic completed. */
    while (((inr(&(ETH->MACMIIAR)) & ETH_MACMIIAR_MB) == ETH_MACMIIAR_MB) && val<10)
    {
        NutSleep(1);
        val++;
    }

    /* Timeout */
    if(val>=10)
    {
        val = 0xFFFF;
        EMPRINTF("phy_inw: Could not read phy register.\n");
    }

    /* Get data from PHY maintenance register. */
    val = (uint16_t) (inr(&(ETH->MACMIIDR)) & ETH_MACMIIDR_MD);

    return val;
}

/*!
 * \brief Write value to PHY register.
 *
 * \param reg PHY register number.
 * \param val Value to write.
 */
static void phy_outw(uint8_t reg, uint16_t val)
{
    uint32_t tempReg;
    uint16_t wait = 0;

    /* PHY write command. */
    outr(&(ETH->MACMIIDR), val);
    tempReg = inr(&(ETH->MACMIIAR));
    tempReg &= 0xffff0020;
    tempReg |= ((NIC_PHY_ADDR) << 11) | ((reg & 0x1F) << 6) | ETH_MACMIIAR_MW | ETH_MACMIIAR_MB| ETH_MACMIIAR_CR_Div;
    outr(&(ETH->MACMIIAR), tempReg);

    /* Wait until PHY logic completed. */
    while (((inr(&(ETH->MACMIIAR)) & ETH_MACMIIAR_MB) == ETH_MACMIIAR_MB) && wait<10)
    {
        NutSleep(1);
        wait++;
    }

    if(wait>=10)
    {
        EMPRINTF("phy_outw: Could not set phy register.\n");
    }
}

/*!
 * \brief Reset the Ethernet controller.
 *
 * \return 0 on success, -1 otherwise.
 */
static int EmacReset(void)
{
    int rc = 0;
    uint32_t phy = 0;
    int link_wait;

#ifdef STM32F10X_CL
    /* force reset mac */
    RCC->AHBRSTR |= RCC_AHBRSTR_ETHMACRST;

    /* enable clocks for mac */
    RCC->AHBENR |= RCC_AHBENR_ETHMACEN | RCC_AHBENR_ETHMACTXEN |
        RCC_AHBENR_ETHMACRXEN;

    /* release reset mac */
    RCC->AHBRSTR &= ~RCC_AHBRSTR_ETHMACRST;
#else
    RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;

    /* enable clocks for mac */
    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACTXEN |
        RCC_AHB1ENR_ETHMACRXEN;

    /* release reset mac */
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST;
#endif
    /* Register PHY to be able to reset it */
    rc = NutRegisterPhy( NIC_PHY_ADDR, phy_outw, phy_inw);

    /* Reset PHY
       (Note that this does not set all PHY registers to reset values!) */
    phy = 1;
    NutPhyCtl(PHY_CTL_RESET, &phy);
    NutSleep(250);

    /* Register PHY again to do all the initianlization */
    rc = NutRegisterPhy( NIC_PHY_ADDR, phy_outw, phy_inw);

#if 0
    /* Clear MII isolate. */
    phy = 0;
    NutPhyCtl(PHY_CTL_ISOLATE, &phy);
#endif

    /* Restart autonegotiation */
    phy = 1;
    NutPhyCtl(PHY_CTL_AUTONEG_RE, &phy);

    /* Wait for auto negotiation completed and link established. */
    for (link_wait = 25;; link_wait--) {
        NutPhyCtl(PHY_GET_STATUS, &phy);
        if((phy & PHY_STATUS_HAS_LINK) && (phy & PHY_STATUS_AUTONEG_OK)) {
            break;
        }
        if (link_wait == 0) {
            EMPRINTF("NO LINK!\n");
            return -1;
        }
        NutSleep(200);
    }

    return rc;
}

/*
 * NIC interrupt entry.
 */
static void EmacInterrupt(void *arg)
{
    uint32_t isr;
    EMACINFO *ni = (EMACINFO *) ((NUTDEVICE *) arg)->dev_dcb;

    /* Read interrupt status and reset interrupt flags. */
    isr = inr(&(ETH->DMASR));
    outr(&(ETH->DMASR), 0x0001E7FF);

    /* Receiver interrupt. */
    if ((isr & ETH_DMASR_RS) != 0 || (isr & ETH_DMASR_ROS) != 0) {
        NutEventPostFromIrq(&(ni->ni_rx_rdy));
    }

    /* Transmitter interrupt. */
    if ((isr & ETH_DMASR_TS) != 0) {
        NutEventPostFromIrq(&(ni->ni_tx_rdy));
    }

    /* Handling of corrupted rx buffers is done in reception
     * routines without any additional activities here */

    /* Also clear interrupt status bits in MAC interrupt status register
     * although we do not use them now, but who knows. */
    isr = inr(&(ETH->MACSR));
}

/*!
 * \brief Fetch the next packet out of the receive buffers.
 *
 * \return 0 on success, -1 otherwise.
 */
static int EmacGetPacket(EMACINFO * ni, NETBUF ** nbp)
{
    int rc = -1;
    uint16_t fbc = 0;
    unsigned int i;

    *nbp = NULL;

    /*
     * Search the next frame start. Release any fragment.
     */
    while ((rxBufTab[rxBufIdx].RDES0 & RDES0_OWN) == 0 && (rxBufTab[rxBufIdx].RDES0 & RDES0_FS) == 0) {
        rxBufTab[rxBufIdx].RDES0 = RDES0_OWN;
        rxBufIdx++;
        if (rxBufIdx >= EMAC_RX_BUFFERS) {
            rxBufIdx = 0;
        }
    }

    /*
     * Determine the size of the next frame.
     */
    i = rxBufIdx;
    while ((rxBufTab[i].RDES0 & RDES0_OWN) == 0) {
        if ((rxBufTab[i].RDES0 & RDES0_LS) != 0) {
            if((rxBufTab[i].RDES0 & RDES0_DE) == 0) {
                fbc = (rxBufTab[i].RDES0 & RDES0_FL_MASK) >> RDES0_FL_SHIFT;
            }
            else {
                /* Release buffers with corrupted frame data */
                while((rxBufTab[rxBufIdx].RDES0 & RDES0_OWN) == 0) {
                    if((rxBufTab[rxBufIdx].RDES0 & RDES0_LS) != 0)
                    {
                        rxBufTab[rxBufIdx].RDES0 = RDES0_OWN;
                        rxBufIdx++;
                        if (rxBufIdx >= EMAC_RX_BUFFERS) {
                            rxBufIdx = 0;
                        }
                        break;
                    }
                    rxBufTab[rxBufIdx].RDES0 = RDES0_OWN;
                    rxBufIdx++;
                    if (rxBufIdx >= EMAC_RX_BUFFERS) {
                        rxBufIdx = 0;
                    }
                }
            }
            break;
        }
        i++;
        if (i >= EMAC_RX_BUFFERS) {
            i = 0;
        }
    }

    if (fbc) {
        /*
         * Receiving long packets is unexpected. Let's declare the
         * chip insane. Short packets will be handled by the caller.
         */
        if (fbc > EMAC_TX_BUFSIZ) {
            ni->ni_insane = 1;
        } else {
            *nbp = NutNetBufAlloc(0, NBAF_DATALINK, fbc);
            if (*nbp != NULL) {
                uint8_t *bp = (uint8_t *) (* nbp)->nb_dl.vp;
                unsigned int len;

                while (fbc) {
                    if (fbc > EMAC_RX_BUFSIZ) {
                        len = EMAC_RX_BUFSIZ;
                    } else {
                        len = fbc;
                    }
                    memcpy(bp, (void *) rxBufTab[rxBufIdx].RDES2, len);
                    rxBufTab[rxBufIdx].RDES0 = RDES0_OWN;
                    rxBufIdx++;
                    if (rxBufIdx >= EMAC_RX_BUFFERS) {
                        rxBufIdx = 0;
                    }
                    fbc -= len;
                    bp += len;
                }
                rc = 0;
            }
            else {
                EMPRINTF("NutNetBufAlloc Failed!\n");
            }
        }
    }

    return rc;
}

/*!
 * \brief Load a packet into the nic's transmit ring buffer.
 *
 * \param nb Network buffer structure containing the packet to be sent.
 *           The structure must have been allocated by a previous
 *           call NutNetBufAlloc(). This routine will automatically
 *           release the buffer in case of an error.
 *
 * \return 0 on success, -1 in case of any errors. Errors
 *         will automatically release the network buffer
 *         structure.
 */
static int EmacPutPacket(int bufnum, EMACINFO * ni, NETBUF * nb)
{
    int rc = -1;
    unsigned int sz;
    uint8_t *buf;

    /*
     * Calculate the number of bytes to be send. Do not send packets
     * larger than the Ethernet maximum transfer unit. The MTU
     * consist of 1500 data bytes plus the 14 byte Ethernet header
     * plus 4 bytes CRC. We check the data bytes only.
     */
    if ((sz = nb->nb_nw.sz + nb->nb_tp.sz + nb->nb_ap.sz) > ETHERMTU) {
        return -1;
    }
    sz += nb->nb_dl.sz;
    /*if (sz & 1) {
        sz++;
    }*/

    /* Disable EMAC interrupts. */
    NutIrqDisable(&sig_EMAC);

    /* TODO: Check for link. */
    if (ni->ni_insane == 0) {
        buf = (uint8_t *) (txBuf + (bufnum * EMAC_TX_BUFSIZ));
        memcpy(buf, nb->nb_dl.vp, nb->nb_dl.sz);
        buf += nb->nb_dl.sz;
        memcpy(buf, nb->nb_nw.vp, nb->nb_nw.sz);
        buf += nb->nb_nw.sz;
        memcpy(buf, nb->nb_tp.vp, nb->nb_tp.sz);
        buf += nb->nb_tp.sz;
        memcpy(buf, nb->nb_ap.vp, nb->nb_ap.sz);

        if(bufnum < (EMAC_TX_BUFFERS - 1)) {
            txBufTab[bufnum].TDES0 = TDES0_LS | TDES0_FS | TDES0_IC | TDES0_TCH;
            txBufTab[bufnum].TDES3 = (uint32_t) &(txBufTab[bufnum + 1]);
        }
        else {
            txBufTab[bufnum].TDES0 = TDES0_LS | TDES0_FS | TDES0_IC | TDES0_TER;
        }
        txBufTab[bufnum].TDES1 = ((sz & TDES1_TBS1_MASK) << TDES1_TBS1_SHIFT);
        txBufTab[bufnum].TDES2 = (uint32_t) (txBuf + (bufnum * EMAC_TX_BUFSIZ));

        /* make descriptor useable for dma controller */
        txBufTab[bufnum].TDES0 |= TDES0_OWN;

        /* trigger dma polling */
        outr(&(ETH->DMATPDR), 0);

        rc = 0;
#ifdef NUT_PERFMON
        ni->ni_tx_packets++;
#endif
    }

    /* Enable EMAC interrupts. */
    NutIrqEnable(&sig_EMAC);

    return rc;
}

/*!
 * \brief Fire up the network interface.
 *
 * NIC interrupts must be disabled when calling this function.
 *
 * \param mac Six byte unique MAC address.
 */
static int EmacStart(const uint8_t * mac)
{
    int i;
    uint32_t regvalue;

    /* stop transmission and reception */
    outr(&(ETH->DMAOMR), (inr(&(ETH->DMAOMR)) & ~(ETH_DMAOMR_ST | ETH_DMAOMR_SR)) | ETH_DMAOMR_FTF);

    /* waiting for flush to finish */
    i=0;
    while(((inr(&(ETH->DMAOMR)) & ETH_DMAOMR_FTF) == ETH_DMAOMR_FTF) && i <100)
    {
        NutSleep(1);
        i++;
    }
    if (i >= 100)
    {
        EMPRINTF("EmacStart: Could not flush\n");
        return -1;
    }
    if (EmacReset()) {
        return -1;
    }

    /* Initialize transmit buffer descriptors using chained list */
    for (i = 0; i < EMAC_TX_BUFFERS - 1; i++) {
        txBufTab[i].TDES0 = TDES0_TCH;
        txBufTab[i].TDES1 = 0;
        txBufTab[i].TDES3 = (uint32_t) &(txBufTab[i+1]);
    }
    txBufTab[EMAC_TX_BUFFERS - 1].TDES0 = /*TDES0_TCH |*/ TDES0_TER;
    txBufTab[EMAC_TX_BUFFERS - 1].TDES1 = 0;

    /* Initialize receive buffer descriptors using chained list */
    for (i = 0; i < EMAC_RX_BUFFERS; i++) {
        rxBufTab[i].RDES0 = RDES0_OWN;
        rxBufTab[i].RDES1 = RDES1_RCH | (EMAC_RX_BUFSIZ << RDES1_RBS1_SHIFT);
        rxBufTab[i].RDES2 = (uint32_t) (rxBuf + (i * EMAC_RX_BUFSIZ));
        if (i == EMAC_RX_BUFFERS - 1) {
            rxBufTab[i].RDES1 = RDES1_RER | (EMAC_RX_BUFSIZ << RDES1_RBS1_SHIFT);
            rxBufTab[i].RDES3 = 0;
        }
        else {
            rxBufTab[i].RDES1 = RDES1_RCH | (EMAC_RX_BUFSIZ << RDES1_RBS1_SHIFT);
            rxBufTab[i].RDES3 = (uint32_t) &(rxBufTab[i+1]);
        }
    }

    /* reset dma and wait until reset is finished */
    outr(&(ETH->DMABMR), ETH_DMABMR_SR);
    i=0;
    while(((inr(&(ETH->DMABMR)) & ETH_DMABMR_SR) == ETH_DMABMR_SR)&& i <100)
    {
        NutSleep(1);
        i++;
    }
    if (i >= 100)
    {
        EMPRINTF("EmacStart: Could not reset dma\n");
        return -1;
    }

    /* Set local MAC address (used e.g. for filtering). */
    outr(&(ETH->MACA0HR), (mac[5] << 8) | mac[4]);
    outr(&(ETH->MACA0LR), (mac[3] << 24) | (mac[2] << 16) | (mac[1] << 8) | mac[0]);

    /* Set bus mode in register DMABMR */
    /* leave at reset state */

    /* Enable dma interrupts */
    outr(&(ETH->DMAIER), ETH_DMAIER_NISE | ETH_DMAIER_AISE |
        /*ETH_DMAIER_ERIE |*/ ETH_DMAIER_FBEIE | /*ETH_DMAIER_ETIE |
        ETH_DMAIER_RWTIE | ETH_DMAIER_RPSIE |*/ ETH_DMAIER_RBUIE |
        ETH_DMAIER_RIE | /*ETH_DMAIER_TUIE | ETH_DMAIER_ROIE |
        ETH_DMAIER_TJTIE | ETH_DMAIER_TBUIE | ETH_DMAIER_TPSIE |*/
        ETH_DMAIER_TIE);

    /* Set start address of descriptor lists */
    outr(&(ETH->DMATDLAR), (uint32_t) txBufTab);
    outr(&(ETH->DMARDLAR), (uint32_t) rxBufTab);

    /* clear management counters */
    outr(&(ETH->MMCCR), ETH_MMCCR_CR);

    /* setting speed and duplex to values from autonegotiation process */
    NutPhyCtl(PHY_GET_STATUS, &regvalue);

    if(regvalue & PHY_STATUS_100M)
    {
        /* set emac to 100Mbit/s */
        outr(&(ETH->MACCR), inr(&(ETH->MACCR)) | ETH_MACCR_FES);
    }
    else
    {
        /* set emac to 10Mbit/s */
        outr(&(ETH->MACCR), inr(&(ETH->MACCR)) & ~ETH_MACCR_FES);
    }
    if(regvalue & PHY_STATUS_FULLDUPLEX)
    {
        /* set emac to full duplex */
        outr(&(ETH->MACCR), inr(&(ETH->MACCR)) | ETH_MACCR_DM);
    }
    else
    {
        /* set emac to half duplex */
        outr(&(ETH->MACCR), inr(&(ETH->MACCR)) & ~ETH_MACCR_DM);
    }

    /* ToDo: Set up MAC frame filters. ATM we receive all frames */
    outr(&(ETH->MACFFR), inr(&(ETH->MACFFR)) /*| ETH_MACFFR_RA | ETH_MACFFR_PM | ETH_MACFFR_PAM*/);

    /* enable transmitter and receiver state machine */
    outr(&(ETH->MACCR), inr(&(ETH->MACCR)) | ETH_MACCR_TE | ETH_MACCR_RE);

    /* start transmission and reception */
    outr(&(ETH->DMAOMR), inr(&(ETH->DMAOMR)) | ETH_DMAOMR_ST | ETH_DMAOMR_SR
        /*testing:*/ | ETH_DMAOMR_FEF | ETH_DMAOMR_FUGF);
    /*outr(&(ETH->DMARPDR), 0);*/

    return 0;
}

/*! \fn EmacRxThread(void *arg)
 * \brief NIC receiver thread.
 *
 */
THREAD(EmacRxThread, arg)
{
    NUTDEVICE *dev;
    IFNET *ifn;
    EMACINFO *ni;
    NETBUF *nb;

    dev = arg;
    ifn = (IFNET *) dev->dev_icb;
    ni = (EMACINFO *) dev->dev_dcb;
    /*
     * This is a temporary hack. Due to a change in initialization,
     * we may not have got a MAC address yet. Wait until a valid one
     * has been set.
     */
    while (!ETHER_IS_UNICAST(ifn->if_mac)) {
        NutSleep(10);
    }

    /*
     * Do not continue unless we managed to start the NIC. We are
     * trapped here if the Ethernet link cannot be established.
     * This happens, for example, if no Ethernet cable is plugged
     * in.
     */
    while (EmacStart(ifn->if_mac)) {
        NutSleep(1000);
    }

    /* Run at high priority. */
    NutThreadSetPriority(9);

    /* Initialize the access mutex. */
    NutEventPost(&ni->ni_mutex);

    /* Clear pending interrupt flags and enable receive interrupts. */
    outr(&(ETH->DMASR), ((unsigned int)inr(&(ETH->DMASR))) | 0x0001E7FF);
    NutIrqEnable(&sig_EMAC);

    for (;;) {
        /*
         * Wait for the arrival of new packets or poll the receiver
         * every two seconds.
         */
        NutEventWait(&ni->ni_rx_rdy, 2000);

        /*
         * Fetch all packets from the NIC's internal buffer and pass
         * them to the registered handler.
         */
        while (EmacGetPacket(ni, &nb) == 0) {
            /* Discard short packets. */
            if (nb->nb_dl.sz < 60) {
                NutNetBufFree(nb);
            } else {
                (*ifn->if_recv) (dev, nb);
            }
        }

        /* We got a weird chip, try to restart it. */
        while (ni->ni_insane) {
            if (EmacStart(ifn->if_mac) == 0) {
                ni->ni_insane = 0;
                ni->ni_tx_queued = 0;
                ni->ni_tx_quelen = 0;
                NutIrqEnable(&sig_EMAC);
            } else {
                NutSleep(1000);
            }
        }
    }
}

/*!
 * \brief Send Ethernet packet.
 *
 * \todo This routine does not work.
 *
 * \param dev Identifies the device to use.
 * \param nb  Network buffer structure containing the packet to be sent.
 *            The structure must have been allocated by a previous
 *            call NutNetBufAlloc().
 *
 * \return 0 on success, -1 in case of any errors.
 */
int EmacOutput(NUTDEVICE * dev, NETBUF * nb)
{
    static uint32_t mx_wait = 5000;
    int rc = -1;
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;

    /*
     * After initialization we are waiting for a long time to give
     * the PHY a chance to establish an Ethernet link.
     */
    while (rc) {
        if (ni->ni_insane) {
            break;
        }
        if (NutEventWait(&ni->ni_mutex, mx_wait)) {
            break;
        }

        /* Check for packet queue space. */
        if ((txBufTab[txBufIdx].TDES0 & TDES0_OWN) == TDES0_OWN) {
            if (NutEventWait(&ni->ni_tx_rdy, 500) && (txBufTab[txBufIdx].TDES0 & TDES0_OWN) == TDES0_OWN) {
                /* No queue space. Release the lock and give up. */
                NutEventPost(&ni->ni_mutex);
                break;
            }
        } else {
            if ((rc = EmacPutPacket(txBufIdx, ni, nb)) == 0) {
                txBufIdx++;
                if(txBufIdx >= EMAC_TX_BUFFERS)
                {
                    txBufIdx = 0;
                }
            }
        }
        NutEventPost(&ni->ni_mutex);
    }

    /*
     * Probably no Ethernet link. Significantly reduce the waiting
     * time, so following transmission will soon return an error.
     */
    if (rc) {
        mx_wait = 500;
    } else {
        /* Ethernet works. Set a long waiting time in case we
           temporarly lose the link next time. */
        mx_wait = 5000;
    }

    return rc;
}

/*!
 * \brief Initialize Ethernet hardware.
 *
 * Applications should do not directly call this function. It is
 * automatically executed during during device registration by
 * NutRegisterDevice().
 *
 * \param dev Identifies the device to initialize.
 */
int EmacInit(NUTDEVICE * dev)
{
    EMACINFO *ni = (EMACINFO *) dev->dev_dcb;
    EMPRINTF("Using Mode %s, Remap %s, NIC_PHY_ADDR %d, \n",
#ifdef PHY_MODE_RMII
             "RMII",
#else
             "MII",
#endif
#ifdef EMAC_REMAP_ENABLE
             "enabled",
#else
             "disabled",
#endif
             NIC_PHY_ADDR
        );

    /* Clear EMACINFO structure. */
    memset(ni, 0, sizeof(EMACINFO));

    /* Register interrupt handler. */
    if (NutRegisterIrqHandler(&sig_EMAC, EmacInterrupt, dev)) {
        return -1;
    }

#ifdef STM32F10X_CL
    /* disable clocks for mac */
    RCC->AHBENR &= ~(RCC_AHBENR_ETHMACEN | RCC_AHBENR_ETHMACTXEN |
                     RCC_AHBENR_ETHMACRXEN);

#else
    /* disable clocks for mac */
    RCC->AHB1ENR &= ~(RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACTXEN |
                     RCC_AHB1ENR_ETHMACRXEN);

    /* Switch on SYSCFG Clock*/
    CM3BBREG(RCC_BASE, RCC_TypeDef, APB2ENR, _BI32(RCC_APB2ENR_SYSCFGEN)) = 1;
#endif

    /*
     * Alternate function remapping and GPIO set-up
     */
#ifdef STM32F10X_CL /* STM32F1 */
 #ifdef EMAC_REMAP_ENABLE
    CM3BBREG(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_ETH_REMAP)) = 1;
 #else
    CM3BBREG(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_ETH_REMAP)) = 0;
 #endif
    /* Configure (R)MII lines as alternate function */
    GpioPinConfigSet(NUTGPIO_PORTC,  1, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_FAST); // ETH_MDC (50MHz)
    GpioPinConfigSet(NUTGPIO_PORTA,  2, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_FAST); // ETH_MDIO (50MHz)
    GpioPinConfigSet(NUTGPIO_PORTB, 11, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_FAST); // ETH_(R)MII_TX_EN (50MHz)
    GpioPinConfigSet(NUTGPIO_PORTB, 12, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_FAST); // ETH_(R)MII_TXD0 (50MHz)
    GpioPinConfigSet(NUTGPIO_PORTB, 13, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_FAST); // ETH_(R)MII_TXD1 (50MHz)
    #if !defined(PHY_MODE_RMII)
        /* Configure additional MII lines as alternate function */
        GpioPinConfigSet(NUTGPIO_PORTC,  2, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_FAST); // ETH_MII_TXD2 (50MHz)
        GpioPinConfigSet(NUTGPIO_PORTB,  8, GPIO_CFG_OUTPUT | GPIO_CFG_PERIPHAL | GPIO_CFG_SPEED_FAST); // ETH_MII_TXD3 (50MHz)
    #endif
#else /* STM32F2/F4 parts */
 #ifdef EMAC_REMAP_ENABLE
  #error General remapping not available for this part. Use alternate function remapping instead!
 #endif
    GpioPinConfigSet(NUTGPIO_PORTA, 1, GPIO_CFG_PERIPHAL); // ETH_RMII_REF_CLK / ETH_MII_RX_CLK
    GpioPinConfigSet(NUTGPIO_PORTA, 2, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // ETH_MDIO
    GpioPinConfigSet(NUTGPIO_PORTA, 7, GPIO_CFG_PERIPHAL); // CRS_DV / ETH_MII_RX_DV
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTA, 1, GPIO_AF_ETH); // REF_CLK / ETH_MII_RX_CLK
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTA, 2, GPIO_AF_ETH); // ETH_MDIO
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTA, 7, GPIO_AF_ETH); // CRS_DV / ETH_MII_RX_DV

    GpioPinConfigSet(NUTGPIO_PORTC, 1, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // ETH_MDC
    GpioPinConfigSet(NUTGPIO_PORTC, 4, GPIO_CFG_PERIPHAL); // ETH_RX_D0
    GpioPinConfigSet(NUTGPIO_PORTC, 5, GPIO_CFG_PERIPHAL); // ETH_RX_D1
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTC, 1, GPIO_AF_ETH); // ETH_MDC
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTC, 4, GPIO_AF_ETH); // ETH_RX_D0
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTC, 5, GPIO_AF_ETH); // ETH_RX_D1

    GpioPinConfigSet(EMAC_TXEN_PORT, 11, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // TXEN
    GpioPinConfigSet(EMAC_TXD0_PORT, EMAC_TXD0_PIN, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // TXD0
    GpioPinConfigSet(EMAC_TXD1_PORT, EMAC_TXD1_PIN, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // TXD1
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_TXEN_PORT, 11, GPIO_AF_ETH); // TXEN
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_TXD0_PORT, EMAC_TXD0_PIN, GPIO_AF_ETH); // TXD0
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_TXD1_PORT, EMAC_TXD1_PIN, GPIO_AF_ETH); // TXD1

   #ifdef EMAC_PPS_OUT_PORT
    GpioPinConfigSet(EMAC_PPS_OUT_PORT, EMAC_PPS_OUT_PIN, GPIO_CFG_PERIPHAL); // ETH_PPS_OUT
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_PPS_OUT_PORT, EMAC_PPS_OUT_PIN, GPIO_AF_ETH); // ETH_PPS_OUT
   #endif /* EMAC_PPS_OUT_PORT */

   #ifdef EMAC_PHY_CLOCK_MCO
    GpioPinConfigSet(NUTGPIO_PORTA, 8, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // MCO1
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTA, 8, GPIO_AF_MCO); // MCO1
    /* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE)) | RCC_CFGR_MCO1_1;
   #endif /* PHY_CLOCK_MCO */

  #if !defined(PHY_MODE_RMII)
    GpioPinConfigSet(NUTGPIO_PORTB, 8, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // ETH_MII_TXD3
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTB,  8,  GPIO_AF_ETH); // ETH_MII_TXD3

    GpioPinConfigSet(NUTGPIO_PORTC, 2, GPIO_CFG_PERIPHAL|GPIO_CFG_OUTPUT|EMAC_GPIO_SPEED); // ETH_MII_TXD2
    GpioPinConfigSet(NUTGPIO_PORTC, 3, GPIO_CFG_PERIPHAL); // ETH_MII_TX_CLK
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTC, 2, GPIO_AF_ETH); // ETH_MII_TXD2
    GPIO_PinAFConfig((GPIO_TypeDef*) NUTGPIO_PORTC, 3, GPIO_AF_ETH); // ETH_MII_TX_CLK

    GpioPinConfigSet(EMAC_CRS_PORT, EMAC_CRS_PIN, GPIO_CFG_PERIPHAL|EMAC_GPIO_SPEED); // ETH_MII_CRS
    GpioPinConfigSet(EMAC_COL_PORT, 3, GPIO_CFG_PERIPHAL|EMAC_GPIO_SPEED); // ETH_MII_COL
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_CRS_PORT, EMAC_CRS_PIN,  GPIO_AF_ETH); // ETH_MII_CRS
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_COL_PORT, 3, GPIO_AF_ETH); // ETH_MII_COL

    GpioPinConfigSet(EMAC_RXD2_PORT, EMAC_RXD2_PIN, GPIO_CFG_PERIPHAL); // ETH_MII_RXD2
    GpioPinConfigSet(EMAC_RXD3_PORT, EMAC_RXD3_PIN, GPIO_CFG_PERIPHAL); // ETH_MII_RXD3
    GpioPinConfigSet(EMAC_RX_ER_PORT, 10, GPIO_CFG_PERIPHAL); // ETH_MII_RX_ER
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_RXD2_PORT, EMAC_RXD2_PIN, GPIO_AF_ETH); // ETH_MII_RXD2
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_RXD3_PORT, EMAC_RXD3_PIN, GPIO_AF_ETH); // ETH_MII_RXD3
    GPIO_PinAFConfig((GPIO_TypeDef*) EMAC_RX_ER_PORT, 10, GPIO_AF_ETH); // ETH_MII_RX_ER
  #endif /* !PHY_MODE_RMII */
#endif

    /*
     * MII or RMII mode selection
     */
#ifdef PHY_MODE_RMII
    /* switch to RMII mode */
 #ifdef STM32F10X_CL
    CM3BBREG(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_MII_RMII_SEL)) = 1;
 #else
    CM3BBREG(SYSCFG_BASE, SYSCFG_TypeDef, PMC, _BI32(SYSCFG_PMC_MII_RMII)) = 1;
 #endif
#else
    /* switch to MII mode */
 #ifdef STM32F10X_CL
    CM3BBREG(AFIO_BASE, AFIO_TypeDef, MAPR, _BI32(AFIO_MAPR_MII_RMII_SEL)) = 0;
 #else
    CM3BBREG(SYSCFG_BASE, SYSCFG_TypeDef, PMC, _BI32(SYSCFG_PMC_MII_RMII)) = 0;
 #endif
#endif

    /* Start the receiver thread. */
    if (NutThreadCreate("emacrx", EmacRxThread, dev,
        (NUT_THREAD_NICRXSTACK * NUT_THREAD_STACK_MULT) + NUT_THREAD_STACK_ADD) == NULL) {
        return -1;
    }
    return 0;
}

static EMACINFO dcb_eth0;

/*!
 * \brief Network interface information structure.
 *
 * Used to call.
 */
static IFNET ifn_eth0 = {
    IFT_ETHER,                  /*!< \brief Interface type, if_type. */
    0,                          /*!< \brief Interface flags, if_flags. */
    {0, 0, 0, 0, 0, 0},         /*!< \brief Hardware net address, if_mac. */
    0,                          /*!< \brief IP address, if_local_ip. */
    0,                          /*!< \brief Remote IP address for point to point, if_remote_ip. */
    0,                          /*!< \brief IP network mask, if_mask. */
    ETHERMTU,                   /*!< \brief Maximum size of a transmission unit, if_mtu. */
    0,                          /*!< \brief Packet identifier, if_pkt_id. */
    0,                          /*!< \brief Linked list of arp entries, arpTable. */
    0,                          /*!< \brief Linked list of multicast address entries, if_mcast. */
    NutEtherInput,              /*!< \brief Routine to pass received data to, if_recv(). */
    EmacOutput,                 /*!< \brief Driver output routine, if_send(). */
    NutEtherOutput,             /*!< \brief Media output routine, if_output(). */
    NULL                        /*!< \brief Interface specific control function, if_ioctl(). */
#ifdef NUT_PERFMON
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#endif
};

/*!
 * \brief Device information structure.
 *
 * A pointer to this structure must be passed to NutRegisterDevice()
 * to bind this Ethernet device driver to the Nut/OS kernel.
 * An application may then call NutNetIfConfig() with the name \em eth0
 * of this driver to initialize the network interface.
 *
 */
NUTDEVICE devStm32Emac = {
    0,                          /*!< \brief Pointer to next device. */
    {'e', 't', 'h', '0', 0, 0, 0, 0, 0},        /*!< \brief Unique device name. */
    IFTYP_NET,                  /*!< \brief Type of device. */
    0,                          /*!< \brief Base address. */
    0,                          /*!< \brief First interrupt number. */
    &ifn_eth0,                  /*!< \brief Interface control block. */
    &dcb_eth0,                  /*!< \brief Driver control block. */
    EmacInit,                   /*!< \brief Driver initialization routine. */
    0,                          /*!< \brief Driver specific control function. */
    0,                          /*!< \brief Read from device. */
    0,                          /*!< \brief Write to device. */
#ifdef __HARVARD_ARCH__
    0,                          /*!< \brief Write from program space data to device. */
#endif
    0,                          /*!< \brief Open a device or file. */
    0,                          /*!< \brief Close a device or file. */
    0                           /*!< \brief Request file size. */
};

/*@}*/

