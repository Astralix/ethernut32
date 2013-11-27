/*!
 * Copyright (C) 2001-2003 by egnite Software GmbH
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
 * $Id: uart.c 4942 2013-01-22 18:56:24Z haraldkipp $
 */

/*!
 * \example uart/uart.c
 *
 * This sample demonstrates the usage of the ATmega on-chip UART.
 * Note, that we don't do any error checking, because without this
 * UART we can't tell the user our problem.
 *
 * We use floating points. Make sure to link with nutlibcrtf.
 */

#include <cfg/crt.h>    /* Floating point configuration. */

#include <string.h>
#include <stdio.h>
#include <io.h>

#include <dev/board.h>
#include <dev/gpio.h>
#include <dev/led.h>
#include <sys/timer.h>
#include <dev/twif.h>

#include <arch/cm3/stm/stm32_mco.h>
//#include <arch/cm3/stm/stm32_clk.h>
#include "dsc11_ifc.h"

HANDLE *hLed_G;
#define LED_G_BANK  NUTGPIO_PORTB
#define LED_G_PIN   7

HANDLE *hLed_B;
#define LED_B_BANK  NUTGPIO_PORTB
#define LED_B_PIN   6

#if 0
#define I2CDS		&Stm32TwiBus_1		/* I2C1_SCL:PB8  / I2C1_SDA:PB9 */
#define I2CDS_BAUD	400
#define I2CDS_BANK	NUTGPIO_PORTB
#define I2CDS_SDA	9
#define I2CDS_SCL	8
#endif

#define I2CFM		&Stm32TwiBus_2		/* I2C2_SCL:PB10 / I2C2_SDA:PB11 */
#define I2CFM_BAUD	400
#define I2CFM_BANK	NUTGPIO_PORTB
#define I2CFM_SDA	11
#define I2CFM_SCL	10

static const char *banner = "\nNut/OS FM RDS Radio " __DATE__ " " __TIME__"\n";
static const char *sOk = "OK\n";
static const char *sFail = "FAIL\n";

void dbg_start( const char *str, int ret)
{
	if (ret)
		printf( "%s... %s %d", str, sFail, ret);
	else
		printf( "%s... %s", str, sOk);
	fflush(stdout);
}

int init_gpios( void)
{
    GpioPortConfigSet( LED_G_BANK, _BV(LED_G_PIN), GPIO_CFG_OUTPUT);
    GpioPortConfigSet( LED_B_BANK, _BV(LED_B_PIN), GPIO_CFG_OUTPUT);
    return 0;
}

int init_i2cfm( uint32_t baud)
{
	int ret = 0;
    uint8_t rda_regs[10] = { 0x00, 0x02 };
    uint8_t i;

    ret = NutRegisterTwiBus( I2CFM, 0x00);
    if (ret) {
        printf("I2C Reg Error %d", ret);
        return ret;
    }

    ret = NutTwiMasterTranceive( I2CFM, 0x20, &rda_regs[0], 2, &rda_regs[0], 10, 200);
    if (ret < 0) {
        printf("I2C Error %d", ret);
        return ret;
    }
    for (i=0; i< 10; i++)
    	printf("0x%02x ", rda_regs[i] );

    return ret;
}

void clock_info( void)
{
	printf("Clock Info:\n");
	printf("SYS:  %lu\n", NutClockGet(NUT_HWCLK_CPU));
	printf("AHB:  %lu\n", NutClockGet(NUT_HWCLK_AHB));
	printf("APB1: %lu\n", NutClockGet(NUT_HWCLK_APB1));
	printf("APB2: %lu\n", NutClockGet(NUT_HWCLK_APB2));
	printf("ADC:  %lu\n", NutClockGet(NUT_HWCLK_ADC));
}

int main(void)
{
    uint32_t baud = 115200;
    int ret = 0;
    FILE *uart;
    

    /* Register USART0 for debugging.
     */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    uart = fopen(DEV_CONSOLE.dev_name, "r+");
    _ioctl(_fileno(uart), UART_SETSPEED, &baud);
    stdin = stdout = stderr = uart;

    /* Greetings
     */
    printf("%s", banner); 
    fflush(stdout);

    clock_info();

    /* Do some fancy blinking stuff */
    dbg_start("Setup GPIOs", init_gpios());
    dbg_start("Reg LED_G", NutRegisterLed(hLed_G, LED_G_BANK, LED_G_PIN));
    dbg_start("Reg LED_B", NutRegisterLed(hLed_B, LED_B_BANK, LED_B_PIN));
    NutSetLed(hLed_G, LED_BLINK, 200, 300);
    NutSetLed(hLed_B, LED_BLINK, 400, 500);
    /* Register I2C bus for fm communication */
    dbg_start("Register I2C-FM... ", init_i2cfm(I2CFM_BAUD));

    /*
     * Nut/OS never expects a thread to return. So we enter an
     * endless loop here.
     */
    for (;;) {
        NutSleep(1000);
        printf(".");
        
    }
    return 0;
}
