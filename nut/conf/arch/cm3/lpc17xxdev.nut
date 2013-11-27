nutarch_cm3_lpc17xx_devices =
{
    -- ***********************************
    --
    -- LPC17xx Device Drivers
    --
    -- ***********************************

    --
    -- LPC176x DEBUG Macro (Low-Level Debug UART definition)
    --
    {
        name = "nutarch_cm3_lpc17xx_debugmacro",
        brief = "LPC17xx Low-Level Debug UART macros for use in exception handlers",
        description = "Polling UART function (macro) to use in exception hanlders\n",
        requires = { "HW_UART0_LPC17xx", "HW_UART1_LPC17xx", "HW_UART2_LPC17xx", "HW_UART3_LPC17xx" },
        provides = { "DEBUG_MACRO"},
		sources = { "cm3/cmsis/cortex_debug.c" },
        options =
        {
            {
                macro = "DEBUG_MACRO",
                brief = "Enabled",
                description = "Check to enable debug output of exception handlers",
                flavor = "boolean",
                file = "include/cfg/cortex_debug.h"
            },
            {
                macro = "DEBUG_UART_NR",
                brief = "Debug UART",
                description = "Select the UART to use for low level debugging",
                type = "enumerated",
                choices = { "LPC_UART0", "LPC_UART1", "LPC_UART2", "LPC_UART3" },
				default = "LPC_UART0",
                file = "include/cfg/cortex_debug.h"
            }
        }
    },

    --
    -- LPC17xx RTC
    --
    {
        name = "nutarch_cm3_lpc17xx_rtc",
        brief = "LPC17xx RTC Driver",
        description = "LPC17xx RTC driver.",
        requires = { "HW_RTC_LPC17xx" },
        provides = { "DEV_RTC" },
        sources = { "cm3/dev/nxp/lpc17xx_rtc.c", "cm3/dev/nxp/ih_lpc17xx_rtc.c" },
    },
    --
    -- LPC17xx Watchdog Timer
    --
    {
        name = "nutarch_cm3_lpc17xx_wdt",
        brief = "LPC17xx Watchdog Timer",
        requires = { "HW_WDT_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_wdt.c", "cm3/dev/nxp/ih_lpc17xx_wdt.c"}
    },
    --
    -- LPC17xx Flash Memory Controller
    --
    {
        name = "nutarch_cm3_lpc17xx_iap",
        brief = "LPC17xx In Application Flash Programming API (IAP)",
        description = "Routines for setup and programming LPC17x series internal FLASH.\n",
        requires = { "HW_FLASH_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_iap.c" }
    },
    --
    -- LPC17xx General purpose DMA Controller
    --
    {
        name = "nutarch_cm3_lpc17xx_gpdma",
        brief = "LPC17xx General purpose DMA Controller API",
        description = "Routines for setup and programming LPC17x series GPDMA controller.\n",
        requires = { "HW_GPDMA_LPC17xx" },
        sources = { "cm3/dev/nxp/lpc17xx_gpdma.c", "cm3/dev/nxp/ih_lpc17xx_dma.c" }
    },
    --
    -- LPC17xx EMAC
    --
    {
        name = "nutarch_cm3_lpc17xx_emac",
        brief = "LPC17xx EMAC Driver",
        description = "LAN driver for LPC176x, LPC177x_8x etc.",
        requires = { "HW_EMAC_LPC17xx", "NUT_EVENT", "NUT_TIMER" },
        provides = { "NET_MAC" },
        sources = { "cm3/dev/nxp/lpc17xx_emac.c", "cm3/dev/nxp/ih_lpc17xx_emac.c" },
        options =
        {
            {
                macro = "NUT_THREAD_NICRXSTACK",
                brief = "Receiver Thread Stack",
                description = "Number of bytes to be allocated for the stack of the NIC receive thread.",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_RX_BUFFERS",
                brief = "Receive Buffers",
                description = "Number of 128 byte receive buffers.\n"..
                              "Increase to handle high traffic situations.\n"..
                              "Decrease to handle low memory situations.\n"..
                              "Default is 32.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_TX_BUFSIZ",
                brief = "Transmit Buffer Size",
                description = "The driver will allocate two transmit buffers.\n"..
                              "Can be decreased in low memory situations. Be aware, "..
                              "that this may break your network application. Do not "..
                              "change this without exactly knowing the consequences.\n"..
                              "Default is 1536.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
            {
                macro = "EMAC_LINK_LOOPS",
                brief = "Link Polling Loops",
                description = "This simple implementation runs a dumb polling loop "..
                              "while waiting for the Ethernet link status.\n"..
                              "If you experience link problems, increasing this value "..
                              "may help.\n"..
                              "Default is 10000.\n",
                flavor = "booldata",
                type = "integer",
                file = "include/cfg/dev.h"
            },
        }
    }
}

