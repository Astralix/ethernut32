#ifndef _SYS_DEVICE_H_
#define _SYS_DEVICE_H_

/*
 * Copyright (C) 2001-2003 by egnite Software GmbH. All rights reserved.
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

#include <sys/file.h>
#include <stdint.h>

/*!
 * \file sys/device.h
 * \brief Nut/OS device definitions.
 */

// wait times for emulation and reality
// has to be overworked

#define WAIT5       5
#define WAIT50      50
#define WAIT100     100
#define WAIT250     250
#define WAIT500     500

/*!
 * \addtogroup xgDevice
 */
/*@{*/

#define IFTYP_RAM       0   /*!< \brief RAM device */
#define IFTYP_ROM       1   /*!< \brief ROM device */
#define IFTYP_STREAM    2   /*!< \brief Stream device */
#define IFTYP_NET       3   /*!< \brief Net device */
#define IFTYP_TCPSOCK   4   /*!< \brief TCP socket */
#define IFTYP_CHAR      5   /*!< \brief Character stream device */
#define IFTYP_CAN       6       /*!< \brief CAN device */
#define IFTYP_BLKIO     7   /*!< \brief Block I/O device */
#define IFTYP_FS       16   /*!< \brief file system device */

/*!
 * \brief Device structure type.
 */
typedef struct _NUTDEVICE NUTDEVICE;

/*!
 * \brief Device structure.
 *
 * Each device driver provides a global variable of this type.
 * Applications use NutRegisterDevice() to bind the device
 * driver to the application code. Except this call, applications
 * refer to device drivers by the name of the device when using
 * standard C functions like _open() or fopen().
 *
 * More than one device driver may be available for the same
 * hardware device. Typically these drivers provide the same
 * name for the device and applications must not refer to
 * more than one device driver with the same name.
 */
struct _NUTDEVICE {

    /*!
     * \brief Link to the next device structure.
     */
    NUTDEVICE *dev_next;

    /*!
     * \brief Unique device name.
     */
    char dev_name[9];

    /*!
     * \brief Type of interface.
     *
     * May be any of the following:
     * - IFTYP_RAM
     * - IFTYP_ROM
     * - IFTYP_STREAM
     * - IFTYP_NET
     * - IFTYP_TCPSOCK
     * - IFTYP_CHAR
     */
    uint8_t dev_type;

    /*!
     * \brief Hardware base address.
     *
     * Will be set by calling NutRegisterDevice(). On some device
     * drivers this address may be fixed.
     */
    uintptr_t dev_base;

    /*! \brief Interrupt registration number.
     *
     * Will be set by calling NutRegisterDevice(). On some device
     * drivers the interrupt may be fixed.
     */
    uint8_t dev_irq;

    /*! \brief Interface control block.
     *
     * With stream devices, this points to the IFSTREAM structure and
     * with network devices this is a pointer to the IFNET structure.
     */
    void *dev_icb;

    /*!
     * \brief Driver control block.
     *
     * Points to a device specific information block.
     */
    void *dev_dcb;

    /*!
     * \brief Driver initialization routine.
     *
     * This routine is called during device registration.
     */
    int (*dev_init) (NUTDEVICE *);

    /*!
     * \brief Driver control function.
     *
     * Used to modify or query device specific settings.
     */
    int (*dev_ioctl) (NUTDEVICE *, int, void *);

    /*!
     * \brief Read from device.
     */
    int (*dev_read) (NUTFILE *, void *, int);

    /*!
     * \brief Write to device.
     */
    int (*dev_write) (NUTFILE *, const void *, int);

    /*!
     * \brief Open a device or file.
     */
    NUTFILE * (*dev_open) (NUTDEVICE *, const char *, int, int);

    /*!
     * \brief Close a device or file.
     */
    int (*dev_close) (NUTFILE *);

    /*!
     * \brief Request file size.
     */
    long (*dev_size) (NUTFILE *);

};

/*!
 * \brief Device structure type.
 */
typedef struct _NUTVIRTUALDEVICE NUTVIRTUALDEVICE;

/*!
 * \brief Virtual device structure.
 */
struct _NUTVIRTUALDEVICE {
    NUTVIRTUALDEVICE *vdv_next;
    NUTVIRTUALDEVICE *vdv_zero;
    uint8_t vdv_type;
    int (*vdv_read) (void *, void *, int);
    int (*vdv_write) (void *, const void *, int);
    int (*vdv_ioctl) (void *, int, void *);
};

/*!
 * \brief Stream interface type.
 */
typedef struct _IFSTREAM IFSTREAM;

/*!
 * \brief Stream interface information structure.
 *
 * Deprecated structure. Device drivers should use
 * the device control block.
 */
struct _IFSTREAM {
    int  (*if_input)(NUTDEVICE *);  /*!< \brief Wait for input. */
    int  (*if_output)(NUTDEVICE *); /*!< \brief Initiate output. */
    int  (*if_flush)(NUTDEVICE *);  /*!< \brief Wait until output buffer empty. */
    volatile uint8_t if_rx_idx;      /*!< \brief Next input index. */
    uint8_t if_rd_idx;               /*!< \brief Next read index. */
    volatile uint8_t if_tx_idx;      /*!< \brief Next output index. */
    uint8_t if_wr_idx;               /*!< \brief Next write index. */
    volatile uint8_t if_tx_act;      /*!< \brief Set if transmitter running. */
    uint8_t if_last_eol;             /*!< \brief Last end of line character read. */
    uint8_t if_rx_buf[256];          /*!< \brief Input buffer. */
    uint8_t if_tx_buf[256];          /*!< \brief Output buffer. */
};

/*@}*/


extern NUTDEVICE *nutDeviceList;

extern int NutRegisterDevice(NUTDEVICE * dev, uintptr_t base, uint8_t irq);
extern NUTDEVICE *NutDeviceLookup(const char *name);
extern NUTDEVICE *NutDeviceLookupType(NUTDEVICE *dev, uint_fast8_t type);

#endif
