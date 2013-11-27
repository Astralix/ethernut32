/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Rittal GmbH & Co. KG. All rights reserved.
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

/*
 * \verbatim
 * $Id: stm32_uart4.c 5409 2013-10-17 11:59:53Z u_bonnes $
 * \endverbatim
 */


#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <cfg/uart.h>
#include <cfg/arch/gpio.h>

#include <string.h>

#include <sys/atom.h>
#include <sys/event.h>
#include <sys/timer.h>

#include <dev/irqreg.h>
#include <dev/gpio.h>
#include <dev/usart.h>

#include <arch/cm3/stm/stm32xxxx.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <arch/cm3/stm/stm32_usart.h>

/*!
 * \addtogroup xgNutArchCm3Stm32
 */
/*@{*/

/*!
 * \brief UART4 device control block structure.
 */
static USARTDCB dcb_uart4 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    { 0,0,0,0,0,0,0},           /* dcb_tx_rbf */
    { 0,0,0,0,0,0,0},           /* dcb_rx_rbf */
    0,                          /* dcb_last_eol */
    Stm32UsartInit,             /* dcb_init */
    Stm32UsartDeinit,           /* dcb_deinit */
    Stm32UsartTxStart,          /* dcb_tx_start */
    Stm32UsartRxStart,          /* dcb_rx_start */
    Stm32UsartSetFlowControl,   /* dcb_set_flow_control */
    Stm32UsartGetFlowControl,   /* dcb_get_flow_control */
    Stm32UsartSetSpeed,         /* dcb_set_speed */
    Stm32UsartGetSpeed,         /* dcb_get_speed */
    Stm32UsartSetDataBits,      /* dcb_set_data_bits */
    Stm32UsartGetDataBits,      /* dcb_get_data_bits */
    Stm32UsartSetParity,        /* dcb_set_parity */
    Stm32UsartGetParity,        /* dcb_get_parity */
    Stm32UsartSetStopBits,      /* dcb_set_stop_bits */
    Stm32UsartGetStopBits,      /* dcb_get_stop_bits */
    Stm32UsartSetStatus,        /* dcb_set_status */
    Stm32UsartGetStatus,        /* dcb_get_status */
    Stm32UsartSetClockMode,     /* dcb_set_clock_mode */
    Stm32UsartGetClockMode,     /* dcb_get_clock_mode */
};

/*!
 * \name STM32 UART4 Device
 */
/*@{*/
/*!
 * \brief UART4 device information structure.
 *
 * An application must pass a pointer to this structure to
 * NutRegisterDevice() before using the serial communication
 * driver of the STM32s on-chip UART4.
 *
 * The device is named \b uart4.
 *
 * \showinitializer
 */
NUTDEVICE devUartStm32_4 = {
    0,                          /* Pointer to next device, dev_next. */
    {'u', 'a', 'r', 't', '4', 0, 0, 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,                 /* Type of device, dev_type. */
    UART4_BASE,                 /* Base address, dev_base. */
    UART4_IRQn,                 /* First interrupt number, dev_irq. */
    NULL,                       /* Interface control block, dev_icb. */
    &dcb_uart4,                 /* Driver control block, dev_dcb. */
    UsartInit,                  /* Driver initialization routine, dev_init. */
    UsartIOCtl,                 /* Driver specific control function, dev_ioctl. */
    UsartRead,                  /* Read from device, dev_read. */
    UsartWrite,                 /* Write to device, dev_write. */
    UsartOpen,                  /* Open a device or file, dev_open. */
    UsartClose,                 /* Close a device or file, dev_close. */
    UsartSize                   /* Request file size, dev_size. */
};

/*@}*/

/*!
 * \brief UART4 GPIO configuartion and assignment.
 */
/*
 * F1/F3  no alternate pins
 * L1/F2/F4
 * TX  PA0     PC10
 * RX  PA1     PC11
 */
#if defined(MCU_STM32F1)
 #undef STM_USART_REMAP_MASK
 #define TX_GPIO_PORT    NUTGPIO_PORTA
 #define TX_GPIO_PIN      0
 #define RX_GPIO_PORT    NUTGPIO_PORTA
 #define RX_GPIO_PIN      1
#elif defined(MCU_STM32F3)
 #define STM_USART_REMAP  GPIO_AF_5
 #define TX_GPIO_PORT    NUTGPIO_PORTC
 #define TX_GPIO_PIN     10
 #define RX_GPIO_PORT    NUTGPIO_PORTC
 #define RX_GPIO_PIN      11
#else /* L1/F2/F4*/
 #define STM_USART_REMAP  GPIO_AF_UART4
 #if !defined(UART4_TX_PIN) || UART4_TX_PIN == 0
  #define TX_GPIO_PORT    NUTGPIO_PORTA
  #define TX_GPIO_PIN      0
 #elif UART4_TX_PIN == 10
  #define TX_GPIO_PORT    NUTGPIO_PORTC
  #define TX_GPIO_PIN     10
 #else
  #warning "Illegal USART TX pin assignement"
 #endif
 #if !defined(UART4_RX_PIN) || UART4_RX_PIN == 1
  #define RX_GPIO_PORT    NUTGPIO_PORTA
  #define RX_GPIO_PIN      1
 #elif UART4_RX_PIN == 11
  #define RX_GPIO_PORT    NUTGPIO_PORTC
  #define RX_GPIO_PIN      11
 #else
  #warning "Illegal UART4 RX pin assignement"
 #endif
#endif


#ifdef UART4_RS485_CTRL
#define USART_485_CTRL
#ifdef UART4_485RE_INV
#define USART_4485RE_INV
#endif
#ifdef UART4_485DE_INV
#define USART_4485DE_INV
#endif
#if defined(UART4_485DE_PORT) && defined(UART4_485DE_PIN)
#define DE_GPIO_BASE  GPIO_ID2GPIO(UART4_485DE_PORT)
#define DE_GPIO_PORT  UART4_485DE_PORT
#define DE_GPIO_PIN   UART4_485DE_PIN
#endif
#if defined(UART4_485RE_PORT) && defined(UART4_485RE_PIN)
#define NRE_GPIO_BASE  GPIO_ID2GPIO(UART4_485RE_PORT)
#define NRE_GPIO_PORT  UART4_485RE_PORT
#define NRE_GPIO_PIN   UART4_485RE_PIN
#endif
#endif /* UART4_RS485_CTRL */

/*!
 * \brief UART4 base configuration.
 */
#define STM_USART_CLK     RCC_APB1ENR_UART4EN

#ifdef UART4_INIT_BAUDRATE
#define USART_INIT_BAUTRATE UART4_INIT_BAUDRATE
#endif

#undef USART_HWFLOWCTRL
#undef  US_MODE_HWHANDSHAKE

#ifdef UART4_XONXOFF_CONTROL
#define USART_XONXOFF_CONTROL
#else
#undef  USART_XONXOFF_CONTROL
#endif

#ifdef UART4_MODE_IRDA
#define USART_MODE_IRDA
#else
#undef  USART_MODE_IRDA
#endif

#ifdef UART4_HARDWARE_HDX
#define USART_HARDWARE_HDX
#else
#undef USART_HARDWARE_HDX
#endif

#ifdef UART4_SUPPORT_DMA
 #if defined(MCU_STM32F1)||defined(MCU_STM32L1)
  #define UART_DMA_TXCHANNEL  DMA2_C5
  #define UART_DMA_RXCHANNEL  DMA2_C3
 #elif  defined(MCU_STM32F2)||defined(MCU_STM32F4)
  #define UART_DMA_TXCHANNEL  DMA_CONTROL0 | DMA_STREAM4 | DMA_CHANNEL4
  #define UART_DMA_RXCHANNEL  DMA_CONTROL0 | DMA_STREAM2 | DMA_CHANNEL4
 #else
  #warning "STM32 family has no implemented DMA"
 #endif
#else
#undef UART_DMA_TXCHANNEL
#undef UART_DMA_RXCHANNEL
#endif

#define USARTn      UART4
#define USARTnBase  UART4_BASE
#define USARTirqn   UART4_IRQn
#define USARTclk    NUT_HWCLK_PCLK1
#define UART_DR_PTR (uint32_t*)(USARTnBase+4)

#define SigUSART sig_UART4
#define DcbUSART dcb_uart4

/*@}*/
#include "stm32_usart.c"
