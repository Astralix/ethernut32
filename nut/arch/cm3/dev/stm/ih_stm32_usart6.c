/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
 * Copyright (C) 2012 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 * THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
 * SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
 * \verbatim
 * $Id: ih_stm32_usart6.c 4575 2012-09-10 00:11:06Z olereinhardt $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>
#include <dev/irqreg.h>
#include <sys/device.h>

#ifndef NUT_IRQPRI_UART6
#define NUT_IRQPRI_UART6  4
#endif

extern NUTDEVICE devUsartStm32_6;

static int Uart6IrqCtl(int cmd, void *param);

/*!
 * \brief IRQ Handler for USART 1.
 */
IRQ_HANDLER sig_USART6 = {
#ifdef NUT_PERFMON
    0,                  /* Interrupt counter, ir_count. */
#endif
    &devUsartStm32_6,   /* Passed argument, ir_arg. */
    NULL,               /* Handler subroutine, ir_handler. */
    Uart6IrqCtl         /* Interrupt control, ir_ctl. */
};

/*!
 * \brief UART 1 interrupt entry.
 */
void Uart6IrqEntry(void *arg)
{
#ifdef NUT_PERFMON
    sig_USART6.ir_count++;
#endif
    if (sig_USART6.ir_handler) {
        (sig_USART6.ir_handler) (sig_USART6.ir_arg);
    }
}

/*!
 * \brief UART 1 interrupt control.
 *
 * \param cmd   Control command.
 *              - NUT_IRQCTL_INIT Initialize and disable interrupt.
 *              - NUT_IRQCTL_STATUS Query interrupt status.
 *              - NUT_IRQCTL_ENABLE Enable interrupt.
 *              - NUT_IRQCTL_DISABLE Disable interrupt.
 *              - NUT_IRQCTL_GETMODE Query interrupt mode.
 *              - NUT_IRQCTL_SETMODE Set interrupt mode (NUT_IRQMODE_LEVEL or NUT_IRQMODE_EDGE).
 *              - NUT_IRQCTL_GETPRIO Query interrupt priority.
 *              - NUT_IRQCTL_SETPRIO Set interrupt priority.
 *              - NUT_IRQCTL_GETCOUNT Query and clear interrupt counter.
 * \param param Pointer to optional parameter.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Uart6IrqCtl(int cmd, void *param)
{
    int rc = 0;
    uint32_t *ival = (uint32_t *)param;
    int enabled = NVIC_GetEnableIRQ(USART6_IRQn);

    /* Disable interrupt. */
    if (enabled) {
        NVIC_DisableIRQ(USART6_IRQn);
    }

    switch(cmd) {
    case NUT_IRQCTL_INIT:
        /* Set the vector. */
        Cortex_RegisterInt(USART6_IRQn, Uart6IrqEntry);
        /* Initialize with defined priority. */
        NVIC_SetPriority(USART6_IRQn, NUT_IRQPRI_UART6);
        /* Clear interrupt */
        NVIC_ClearPendingIRQ(USART6_IRQn);
        break;
    case NUT_IRQCTL_STATUS:
        if (enabled) {
            *ival |= 1;
        }
        else {
            *ival &= ~1;
        }
        break;
    case NUT_IRQCTL_ENABLE:
        enabled = 1;
        break;
    case NUT_IRQCTL_DISABLE:
        enabled = 0;
        break;
    case NUT_IRQCTL_GETMODE:
            *ival = NUT_IRQMODE_EDGE;
        break;
    case NUT_IRQCTL_SETMODE:
            rc = -1;
        break;
    case NUT_IRQCTL_GETPRIO:
        *ival = NVIC_GetPriority(USART6_IRQn);
        break;
    case NUT_IRQCTL_SETPRIO:
        NVIC_SetPriority(USART6_IRQn,*ival);
        break;
#ifdef NUT_PERFMON
    case NUT_IRQCTL_GETCOUNT:
        *ival = (uint32_t)sig_USART6.ir_count;
        sig_USART6.ir_count = 0;
        break;
#endif
    default:
        rc = -1;
        break;
    }

    /* Enable interrupt. */
    if (enabled) {
        NVIC_EnableIRQ(USART6_IRQn);
    }
    return rc;
}
