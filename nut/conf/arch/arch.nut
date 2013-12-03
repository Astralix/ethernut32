--
-- Copyright (C) 2004-2007 by egnite Software GmbH. All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
--
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
-- 3. Neither the name of the copyright holders nor the names of
--    contributors may be used to endorse or promote products derived
--    from this software without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-- FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
-- COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
-- OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
-- AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
-- OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
-- THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--
-- For additional information see http://www.ethernut.de/
--

-- Operating system functions
--
--

nutarch =
{
    {
        name = "nutarch_mcu",
        brief = "Target CPU",
        description = "Select one only.",
        options =
        {
            {
                macro = "MCU_AT91SAM9260",
                brief = "Atmel AT91SAM9260",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9260",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_MCI_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_EXT_CALYPSO"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM9G45",
                brief = "Atmel AT91SAM9G45",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9G45",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_PIT_AT91"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM9XE512",
                brief = "Atmel AT91SAM9XE512",
                description = "ARM926EJ-S RISC microcontroller with Ethernet MAC, "..
                              "one USB Device Port, and a USB Host controller. "..
                              "It also integrates several standard peripherals, "..
                              "such as the USART, SPI, TWI, Timer Counters, Synchronous "..
                              "Serial Controller, ADC and MultiMedia Card Interface.\n\n"..
                              "Experimental port.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM9XE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART0_MODEM",
                    "HW_UART1_RTSCTS",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_MCI_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_PIT_AT91"
                },
                makedefs = { "MCU=arm9" }
            },
            {
                macro = "MCU_AT91SAM7X128",
                brief = "Atmel AT91SAM7X128",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 128K bytes flash, "..
                              "32K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7X256",
                brief = "Atmel AT91SAM7X256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                              "64K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_EXT_CALYPSO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7X512",
                brief = "Atmel AT91SAM7X512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                              "128K bytes RAM, Ethernet MAC, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7X",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_EMAC_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_PDC_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S16",
                brief = "Atmel AT91SAM7S16",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 16K bytes flash, "..
                    "4K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S32",
                brief = "Atmel AT91SAM7S32",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 32K bytes flash, "..
                    "8K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S64",
                brief = "Atmel AT91SAM7S64",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 64K bytes flash, "..
                    "16K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S128",
                brief = "Atmel AT91SAM7S128",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 128K bytes flash, "..
                    "32K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S256",
                brief = "Atmel AT91SAM7S256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                    "64K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7S512",
                brief = "Atmel AT91SAM7S512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                    "64K bytes RAM, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7S",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE32",
                brief = "Atmel AT91SAM7SE32",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 32K bytes flash, "..
                    "8K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE256",
                brief = "Atmel AT91SAM7SE256",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes flash, "..
                    "32K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91SAM7SE512",
                brief = "Atmel AT91SAM7SE512",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 512K bytes flash, "..
                    "32K bytes RAM, external bus interface, USB, 2 USARTs and more. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91SAM7SE",
                    "HW_TIMER_AT91",
                    "HW_PLL_AT91",
                    "HW_SDRAMC",
                    "HW_PIT_AT91",
                    "HW_UART_AT91",
                    "HW_DBGU_AT91",
                    "HW_UART0_RTSCTS",
                    "HW_UART1_RTSCTS",
                    "HW_UART1_MODEM",
                    "HW_SPI_AT91",
                    "HW_TWI_AT91",
                    "HW_SSC_AT91",
                    "HW_RSTC_AT91",
                    "HW_PDC_AT91",
                    "HW_EFC_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91",
                    "HW_WDOG_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_AT91R40008",
                brief = "Atmel AT91R40008",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with 256K bytes RAM, "..
                              "64M bytes address space, 2 USARTs and 3 timers. ",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_AT91",
                    "HW_MCU_AT91R40008",
                    "HW_TIMER_AT91",
                    "HW_UART_AT91",
                    "HW_MCU_SWTWI",
                    "HW_WDOG_AT91",
                    "HW_GPIO",
                    "HW_EBI_AT91"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            {
                macro = "MCU_GBA",
                brief = "Nintendo GBA",
                description = "ARM7TDMI 16/32-bit RISC microcontroller",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_GBA",
                    "HW_TIMER_GBA",
                    "HW_LCD_GBA",
                    "DEV_NVMEM"
                },
                makedefs = { "MCU=arm7tdmi" }
            },
            --
            -- STM STM32F10X SERIES CONTROLLER
            --
            {
                macro = "MCU_STM32F100",
                brief = "STM STM32F100",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F100",
                    "MCU_STM32F100"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F101",
                brief = "STM STM32F101",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F101",
                    "MCU_STM32F101",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F102",
                brief = "STM STM32F102",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F102",
                    "MCU_STM32F102",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F103",
                brief = "STM STM32F103",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F103",
                    "MCU_STM32F103",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_STM32F105",
                brief = "STM STM32F105",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F105",
                    "MCU_STM32F105",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32F107",
                brief = "STM STM32F107",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F10X",
                    "HW_MCU_STM32F107",
                    "MCU_STM32F107",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32L151",
                brief = "STM STM32L151",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1XX",
                    "HW_MCU_STM32L15X",
                    "HW_MCU_STM32L151"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32L152",
                brief = "STM STM32L152",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1XX",
                    "HW_MCU_STM32L15X",
                    "HW_MCU_STM32L152"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32VL100",
                brief = "STM STM32L100",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32L1XX",
                    "HW_MCU_STM32VL10X",
                    "HW_MCU_STM32VL100"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" }
            },
            {
                macro = "MCU_STM32F401",
                brief = "STM STM32F401",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F401",
                    "MCU_STM32F401",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F405",
                brief = "STM STM32F405",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F40X",
                    "HW_MCU_STM32F405",
                    "MCU_STM32F405",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F407",
                brief = "STM STM32F407",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F40X",
                    "HW_MCU_STM32F407",
                    "MCU_STM32F407",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F415",
                brief = "STM STM32F415",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F40X",
                    "HW_MCU_STM32F415",
                    "MCU_STM32F415",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F417",
                brief = "STM STM32F417",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F40X",
                    "HW_MCU_STM32F417",
                    "MCU_STM32F417",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F427",
                brief = "STM STM32F427",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F42X",
                    "HW_MCU_STM32F427",
                    "MCU_STM32F427",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F429",
                brief = "STM STM32F429",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F42X",
                    "HW_MCU_STM32F429",
                    "MCU_STM32F429",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F437",
                brief = "STM STM32F437",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F42X",
                    "HW_MCU_STM32F437",
                    "MCU_STM32F437",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F439",
                brief = "STM STM32F439",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F4XX",
                    "HW_MCU_STM32F42X",
                    "HW_MCU_STM32F439",
                    "MCU_STM32F439",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F205",
                brief = "STM STM32F205",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F2XX",
                    "HW_MCU_STM32F205",
                    "MCU_STM32F205",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3" }
            },
            {
                macro = "MCU_STM32F207",
                brief = "STM STM32F207",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F2XX",
                    "HW_MCU_STM32F207",
                    "MCU_STM32F207",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3" }
            },
            {
                macro = "MCU_STM32F302",
                brief = "STM STM32F302",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3XX",
                    "HW_MCU_STM32F30X",
                    "MCU_STM32F302",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F303",
                brief = "STM STM32F303",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3XX",
                    "HW_MCU_STM32F30X",
                    "MCU_STM32F303",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F313",
                brief = "STM STM32F313",
                description = "CortexM3 32-bit RISC microcontroller\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3XX",
                    "HW_MCU_STM32F30X",
                    "MCU_STM32F313",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F373",
                brief = "STM STM32F373",
                description = "CortexM3 32-bit RISC microcontroller with 16-bit SDADC\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3XX",
                    "HW_MCU_STM32F37X",
                    "MCU_STM32F373",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            {
                macro = "MCU_STM32F383",
                brief = "STM STM32F383",
                description = "CortexM3 32-bit RISC microcontroller with 16-bit SDADC\n\n"..
                              "Select the correct sub-type in Architecture->CM3->STM32 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_STM32",
                    "HW_MCU_STM32F3XX",
                    "HW_MCU_STM32F37X",
                    "MCU_STM32F383",
                    "HW_GPIO",
                    "HW_MCU_FPU"
                },
                makedefs = { "MCU=cortex-m4" }
            },
            --
            -- TI LM3S SERIES CONTROLLER
            --
            {
                macro = "MCU_LM3S9B96",
                brief = "Luminary LM3S9B96",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LM3 Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LM3",
                    "HW_MCU_LM3S9B96",
                    "MCU_LM3S9B96",
                    "HW_GPIO"
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            --
            -- NXP LPC17xx CONTROLLER
            --
            {
                macro = "MCU_LPC175x",
                brief = "NXP LCP175x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC175x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC175x",
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_LPC176x",
                brief = "NXP LCP176x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC176x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC176x",
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            {
                macro = "MCU_LPC177x_8x",
                brief = "NXP LCP177x_8x series",
                description = "CortexM3 32-bit RISC microcontroller.\n\n"..
                              "Select the correct sub-type in Architecture->CM3->LPC177x_8x Family.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_CM3" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_CM3",
                    "HW_MCU_LPC17xx",
                    "HW_MCU_LPC177x_8x",
                },
                makedefs = { "MCU=cortex-m3", "MFIX=-mfix-cortex-m3-ldrd" },
            },
            --
            -- UNIX EMULATION FOR NUT/OS
            --
            {
                macro = "MCU_LINUX_EMU",
                brief = "Linux Emulator",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_LINUX" },
                provides = { "HW_TARGET", "DEV_NVMEM", "HW_EMU_LINUX" }
            },
            {
                macro = "MCU_H8_3068",
                brief = "Renesas H8/3068",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_H8300" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_H8300",
                    "H83068_TIMER",
                    "H83068_UART"
                }
            },
            {
                macro = "MCU_S3C4510B",
                brief = "Samsung S3C4510B",
                description = "ARM7TDMI 16/32-bit RISC microcontroller with Ethernet MAC,"..
                              "HDLC protocol, 64M bytes address space, I2C, 2 UARTs and "..
                              "2 timers.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_MCU_S3C45",
                    "HW_TIMER_S3C45",
                    "HW_UART_S3C45"
                }
            },

            --
            -- AVR32
            --
            {
                macro = "MCU_AVR32UC3A0512",
                brief = "Atmel AT32UC3A0512",
                description = "32-bit AVR UC3 RISC microcontroller with 512K flash, 64K SRAM,\n"..
                              "10/100 Ethernet MAC, Full-Speed + OTG USB,"..
                              "I2C, 4 UARTs and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_MACB_AVR32",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a0512" }
            },
            {
                macro = "MCU_AVR32UC3A0512ES",
                brief = "Atmel AT32UC3A0512-ES",
                description = "*** Engineering Sample, do not use this CPU for new designs ***\n\n"..
                              "32-bit AVR UC3 RISC microcontroller with 512K flash, 64K SRAM,\n"..
                              "10/100 Ethernet MAC, Full-Speed + OTG USB,"..
                              "I2C, 4 UARTs and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_MACB_AVR32",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a0512es" }
            },
            {
                macro = "MCU_AVR32UC3A3256",
                brief = "Atmel AT32UC3A3256",
                description = "32-bit AVR UC3 RISC microcontroller with 256k Flash, 64k SRAM,\n"..
                              "High-Speed + OTG USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_SDRAMC",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_SPI_AVR32_1",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO",
                    "HW_EBI_AVR32"
                },
                makedefs = { "MCU=uc3a3256" }
            },
            {
                macro = "MCU_AVR32UC3B0256",
                brief = "Atmel AT32UC3B0256",
                description = "32-bit AVR UC3 RISC microcontroller with 256k Flash, 32k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3b0256" }
            },
            {
                macro = "MCU_AVR32UC3B164",
                brief = "Atmel AT32UC3B164",
                description = "32-bit AVR UC3 RISC microcontroller with 64k Flash, 16k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                    "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                    "HW_PLL_AVR32",
                    "HW_RTC_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3b164" }
            },
            {
                macro = "MCU_AVR32UC3L064",
                brief = "Atmel AT32UC3L064",
                description = "32-bit AVR UC3 RISC microcontroller with 64k Flash, 16k SRAM,\n"..
                              "Full-Speed + Mini-Host USB device and many other peripherals.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_AVR32" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_AVR32",
                    "HW_MCU_AVR32UC3",
                    "HW_TIMER_AVR32",
                    "HW_UART_AVR32",
                    "HW_UART2_AVR32",
                    "HW_SPI_AVR32_0",
                   -- "HW_EFC_AVR32",
                    "HW_WDOG_AVR32",
                   -- "HW_PLL_AVR32",
                    "HW_DFLL_AVR32",
                    "HW_GPIO"
                },
                makedefs = { "MCU=uc3l064" }
            },
            {
                macro = "MCU_MCF51CN128",
                brief = "Freescale MCF51CN128",
                description = "32-bit RISC microcontroller, V1 Coldfire Core, 128K flash, 24K SRAM"..
                              "10/100 Ethernet MAC, MII, 3xSCI(UART), 2xIIC, 2xSPI, ADC, RTC, 2xModulo Timer, 2xTimer/PWM."..
                              "Mini FlexBUS, Keyboard Interrupts, 1xExternal Interrupt, Rapid GPIO",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_M68K" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_M68K",
                    "HW_MCU_COLDFIRE",
                    "HW_MCU_MCF51CN",
                    "HW_MCU_MCF51CN128",
                },
                makedefs = { "MCU=51cn" }
            },
            {
                macro = "MCU_MCF52259",
                brief = "Freescale MCF52259",
                description = "32-bit RISC microcontroller, V2 Coldfire Core, 10/100 Ethernet MAC"..
                              "512K flash, 64K SRAM, 3xUART, 2xI2C, ADC, QSPI, DMA, FlexCAN,"..
                              "Mini FlexBUS, USB, RTC, Random Number Generator, Cryptographic Accelerator.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_M68K" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_M68K",
                    "HW_MCU_COLDFIRE",
                    "HW_MCU_MCF5225X",
                    "HW_MCU_MCF52259",
                },
                makedefs = { "MCU=52259" }
            },

            --
            -- Imaginary Zero CPU
            --
            {
                macro = "MCU_ZERO",
                brief = "Zero Dummy CPU",
                description = "Imaginary ARM9 CPU, useful as a porting template.",
                flavor = "boolean",
                exclusivity = mcu_names,
                file = "include/cfg/arch.h",
                requires = { "TOOL_CC_ARM" },
                provides = {
                    "HW_TARGET",
                    "HW_MCU_ARM",
                    "HW_TIMER_ZERO",
                    "HW_UART_ZERO"
                },
                makedefs = { "MCU=arm9" }
            }
        }
    },

    --
    -- Architecture Dependent Implementations
    --
    {
        name = "nutarch_arm",
        brief = "ARM",
        requires = { "HW_MCU_ARM" },
        provides = { "ARM_SEMIHOSTING" };
        script = "arch/arm.nut"
    },
    {
        name = "nutarch_cm3",
        brief = "CM3",
        requires = { "HW_MCU_CM3" },
        provides = { "ARM_SEMIHOSTING" };
        script = "arch/cm3.nut"
    },
    {
        name = "nutarch_avr32",
        brief = "AVR32",
        requires = { "HW_MCU_AVR32" },
        script = "arch/avr32.nut"
    },

}
