#ifndef _OWI_UART_H_
#define _OWI_UART_H_
/*
 * Copyright (C) 2012 by Uwe Bonnes(bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * \file dev/owibus_uartif.c
 * \brief Header for the One-Wire API over UART Implementation.
 *
 * \verbatim
 * $Id: owibus_uartif.h 4424 2012-08-17 12:12:06Z haraldkipp $
 * \endverbatim
 */

#include <sys/device.h>

/*!
 * \addtogroup xgOwibusUart
 */
/*@{*/

/*!
 * \brief Data to send on the UART for the OWI primitives.
 */
#define OWI_UART_WRITE_RST  0xf0    /*!< \brief UART data for presence impulse. */
#define OWI_UART_WRITE_ONE  0xff    /*!< \brief UART data for write '1' and read. */
#define OWI_UART_WRITE_ZERO 0x00    /*!< \brief UART data for write '0'. */

/*!
 * \brief Data to expect from the UART for the OWI primitives.
 */
#define OWI_UART_READ_ONE   0x01    /*!< \brief UART data received for read '1'. */

/*!
 * \brief Baud rates to use for OWI primitives.
 */
#define OWI_UART_BAUD_RESET 9600    /*!< \brief UART baudrate for presence impulse. */
#define OWI_UART_BAUD_RWBIT 115200  /*!< \brief UART baudrate for RW bit. */

/*!
 * \brief OWI runtime control block container.
 *
 * This is installed in heap at initialization.
 */
struct _NUTOWIINFO_UART {
    int uart_fd;
    int pp_port;
    int pp_pin;
};

typedef struct _NUTOWIINFO_UART NUTOWIINFO_UART;

int NutRegisterOwiBus_Uart(NUTOWIBUS *bus, NUTDEVICE *uart, int PARASITIC_PWR_PORT, uint_fast8_t PARASITIC_PWR_PIN);

/*@}*/

#endif
