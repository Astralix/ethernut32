/*
 * Copyright (C) 2002 by egnite Software GmbH. All rights reserved.
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

/*!
 * \file arch/avr/dev/uart1.c
 * \brief AVR fast UART1 driver.
 *
 * \verbatim
 * $Id: uart1.c 4706 2012-10-06 17:42:01Z haraldkipp $
 * \endverbatim
 */

#include <dev/uartavr.h>

extern int UartAvrInit(NUTDEVICE * dev);
extern int UartAvrIOCtl(NUTDEVICE * dev, int req, void *conf);
extern int UartAvrInput(NUTDEVICE * dev);
extern int UartAvrOutput(NUTDEVICE * dev);
extern int UartAvrFlush(NUTDEVICE * dev);

extern int UartAvrGetRaw(uint8_t * cp);
extern int UartAvrPutRaw(uint8_t ch);

extern int UartAvrRead(NUTFILE * fp, void *buffer, int size);
extern int UartAvrWrite(NUTFILE * fp, const void *buffer, int len);
#ifdef __HARVARD_ARCH__
extern int UartAvrWrite_P(NUTFILE * fp, PGM_P buffer, int len);
#endif
extern NUTFILE *UartAvrOpen(NUTDEVICE * dev, const char *name, int mode, int acc);
extern long UartAvrSize(NUTFILE * fp);
extern int UartAvrClose(NUTFILE * fp);

/*!
 * \addtogroup xgUartAvr
 */
/*@{*/


static UARTDCB dcb_uart1;
static IFSTREAM ifs_uart1;

/*!
 * \brief UART 1 Device information structure.
 */
NUTDEVICE devUart1 = {
    0,                          /*!< Pointer to next device. */
    {'u', 'a', 'r', 't', '1', 0, 0, 0, 0},      /*!< Unique device name. */
    IFTYP_STREAM,               /*!< Type of device. */
    1,                          /*!< Base address. */
    0,                          /*!< First interrupt number. */
    &ifs_uart1,                 /*!< Interface control block. */
    &dcb_uart1,                 /*!< Driver control block. */
    UartAvrInit,                /*!< Driver initialization routine. */
    UartAvrIOCtl,               /*!< Driver specific control function. */
    UartAvrRead,                /*!< Read from device. */
    UartAvrWrite,               /*!< Write to device. */
    UartAvrWrite_P,             /*!< Write to device from program space. */
    UartAvrOpen,                /*!< Open a device or file. */
    UartAvrClose,               /*!< Close a device or file. */
    UartAvrSize                 /*!< Request file size. */
};

/*@}*/
