/*!
 * Copyright (C) 2013 Uwe Bonnes, bon@elektron.ikp.physik.tu-darmstadt.de
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
 * $Id: cantest.c 5125 2013-05-09 16:14:18Z u_bonnes $
 */

/*!
 * \example cantest/cantest.c
 *
 * This sample demonstrates usage of the can_dev API
 * At present only available on STM32
 *
 */

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <dev/canbus.h>
#include <dev/uart.h>
#include <sys/timer.h>
#include <sys/thread.h>

static char *banner = "\nNut/OS CANTEST Sample " __DATE__ " " __TIME__"\n";

#define MASTER 0
#define SLAVE  1

#if defined(DEF_CANBUS)
THREAD(service_thread, arg)
{
    FILE *uart = arg;
    char inbuf[1];
    int res, pending;
    CANFRAME data;
    uint32_t millis;

    for (;;)
    {
        res = _read(_fileno(uart), inbuf, sizeof(inbuf));
        if (res == 0)
            continue;
        switch (inbuf[0])
        {
        case 'C':
            printf("MASTER CAN_RX_FRAMES %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_RX_FRAMES));
            printf("MASTER CAN_TX_FRAMES %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_TX_FRAMES));
            printf("MASTER CAN_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_INTERRUPTS));
            printf("MASTER CAN_RX_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_RX_INTERRUPTS));
            printf("MASTER CAN_TX_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_TX_INTERRUPTS));
            printf("MASTER CAN_SCE_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_SCE_INTERRUPTS));
            printf("MASTER CAN_OVERRUNS %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_OVERRUNS));
            printf("MASTER CAN_ERRORS %d\n",
                   CanGetCounter(&DEF_CANBUS, CAN_ERROR));

#if defined(DEF_CANBUS_SLAVE)
            printf(" SLAVE CAN_RX_FRAMES %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_RX_FRAMES));
            printf(" SLAVE CAN_TX_FRAMES %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_TX_FRAMES));
            printf(" SLAVE CAN_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_INTERRUPTS));
            printf(" SLAVE CAN_RX_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_RX_INTERRUPTS));
            printf(" SLAVE CAN_TX_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_TX_INTERRUPTS));
            printf(" SLAVE CAN_SCE_INTERRUPTS %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_SCE_INTERRUPTS));
            printf(" SLAVE CAN_OVERRUNS %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_OVERRUNS));
            printf(" SLAVE CAN_ERRORS %d\n",
                   CanGetCounter(&DEF_CANBUS_SLAVE, CAN_ERROR));
#endif
            break;
        case 's':
            memset(&data, 0, sizeof(data));
            data.id = 0x5a5;
            pending = 1;
            break;
        case 'e':
            memset(&data, 0, sizeof(data));
            data.id  = 0x01234567;
            data.ext = 1;
            data.len = 4;
            millis = NutGetMillis();
            data.byte[0] = (millis>>24) & 0xff;
            data.byte[1] = (millis>>16) & 0xff;
            data.byte[2] = (millis>> 8) & 0xff;
            data.byte[3] = (millis>> 0) & 0xff;
            pending = 1;
            break;
        }
        if (pending && CanTxFree(&DEF_CANBUS))
        {
            CanOutput(&DEF_CANBUS, &data);
            pending = 0;
        }
    }
}

void print_frame(int type)
{
    int res, i;
    CANFRAME data;

    res = CanInput((type)?&DEF_CANBUS_SLAVE:&DEF_CANBUS, &data);
    if (res)
    {
        printf("CanInput %s failed %d\n", (type)?" SLAVE":"MASTER", res);
        while(1) NutSleep(100);
    }
    if (data.ext)
        printf("ID %08lx", data.id);
    else
        printf("ID      %03lx", data.id);
    printf("%s len %2d%4s%4s",
           (type)?"  SLAVE ":" MASTER ",
           data.len,
           (data.ext)?" ext":"    ",
           (data.rtr)?" rtr":"    ");
    if(data.len)
    {
        printf(" data:");
        for(i=0; i<data.len; i++)
            printf(" %02x",data.byte[i]);
    }
    printf("\n");
}
#else
THREAD(service_thread, arg)
{
    while(1)
        NutSleep(1);
}
void print_frame(int type)
{
}
#endif

/*
 * CAN_DEV
 *
 */
int main(void)
{
    int res;
    uint32_t baud = 115200, read_timeout = 10;
    FILE *uart;
    CANFILTER  filter;

    NutRegisterDevice(&DEV_CONSOLE, 0, 0);

    uart = fopen(DEV_CONSOLE.dev_name, "r+");
    _ioctl(_fileno(uart), UART_SETSPEED, &baud);
    _ioctl(_fileno(uart), UART_SETREADTIMEOUT, &read_timeout);
    freopen(DEV_CONSOLE.dev_name, "r", stdin);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);

    printf(banner);
    NutThreadCreate("input", service_thread, uart, 1024);

#if !defined(DEF_CANBUS)
    (void) res;
    (void) filter;
    printf("No suitable CAN driver found for that device");
    for (;;) {
        NutSleep(100);
    }
#else
    res = NutRegisterCanBus( &DEF_CANBUS, -1);
    if (res) {
        printf("NutRegisterCanBus failed %d\n", res);
        while(1) NutSleep(100);
    }

    res = CanSetBaud(&DEF_CANBUS, CAN_SPEED_1M, 0);
    if (res) {
        printf("CanSetBaud failed %d\n", res);
        while(1) NutSleep(100);
    }

    /* Receive extended frame with any address on Master*/
    memset(&filter, 0, sizeof(filter));
    filter.mask_ext = 1;
    filter.id_ext = 1;
    res = CanAddFilter(&DEF_CANBUS, &filter);
    if (res) {
        printf("CanAddFilter Slave failed %d\n", res);
        while(1) NutSleep(100);
    }

#if defined(DEF_CANBUS_SLAVE)
    res = NutRegisterCanBus( &DEF_CANBUS_SLAVE, -1);
    if (res) {
        printf("NutRegisterCanBus compaignon failed %d\n", res);
        while(1) NutSleep(100);
    }

    /* Receive standard frame with any address on Slave*/
    filter.id_ext = 0;
    res = CanAddFilter(&DEF_CANBUS_SLAVE, &filter);
    if (res) {
        printf("CanAddFilter Master failed %d\n", res);
        while(1) NutSleep(100);
    }
#endif

    CanEnableRx(&DEF_CANBUS);
#if defined(DEF_CANBUS_SLAVE)
    CanEnableRx(&DEF_CANBUS_SLAVE);
#endif
    for (;;) {
        if(CanRxAvail(&DEF_CANBUS))
           print_frame(MASTER);
#if defined(DEF_CANBUS_SLAVE)
        if(CanRxAvail(&DEF_CANBUS_SLAVE))
           print_frame(SLAVE);
#endif
        NutSleep(100);
    }
#endif
    return 0;
}
