--
--  * Copyright (C) 2013 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

-- ARM CortexM3 Architecture
-- STMicroelectronics STM32L Family Devices
--
--
stm32_memory_f30 = { "128", "256" }
stm32_memory_f37 = { "64", "128", "256" }
stm32f3_devices = { "MCU_STM32F302" , "MCU_STM32F303", "MCU_STM32F313", "MCU_STM32F373", "MCU_STM32F383" }

-- *****************************************************************************
-- STM32F30 Family
-- *****************************************************************************
--

nutarch_cm3_stm32f3 =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32f3_family",
        brief = "MCU F3 Family",
        requires = { "HW_MCU_STM32F3XX" },
        options =
        {
            {
                macro = "MCU_STM32F3",
                brief = "STM32F3 family",
                type = "integer",
                default = 1,
                provides =
                {
                    "HW_PLL_STM32F3",
                    "HW_FLASH_STM32F1_3",
                    "HW_GPIO_STM32V2",
                    "HW_CRC32_STM32",
                    "DEV_IRQ_STM32",
                    "HW_DMA1_STM32F1",
                    "HW_DMA2_STM32F1",
                    "HW_EXTI04_STM32F3",
                    "HW_EXTI95_STM32",
                    "HW_EXTI1510_STM32",
                    "HW_I2C1_STM32V2",
                    "HW_I2C2_STM32V2",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_SPI3_STM32",
                      "HW_UART1_STM32",
                      "HW_UART2_STM32",
                      "HW_UART3_STM32",
                      "HW_CAN1_STM32",
                      "HW_RTC_STM32_V2",
                    "HW_STM32_TIM2_32BIT",
                    "HW_STM32_TIM3",
                    "HW_STM32_TIM4",
                    "HW_STM32_TIM6",
                    "HW_STM32_TIM15",
                    "HW_STM32_TIM16",
                    "HW_STM32_TIM17",
                    "HW_STM32_ADC1",
                    "HW_STM32_DAC1",
                    "HW_STM32_COMP1_2",
                    "HW_STM32_OP1_2",
                    "HW_STM32_DAC1",
                },
                file = "include/cfg/arch.h"
            }
        }
    },
    --
    -- STM32F30 MCU Classes
    --
    {
        name = "nutarch_cm3_stm32F3_devices",
        brief = "STM32F30 Device Classes",
        requires = { "HW_MCU_STM32F3XX" },
        options =
        {
            {
                macro = "MCU_STM32F302",
                brief = "STM32F30 Series",
                description = "STM32F30 devices.",
                flavor = "booldata",
                exclusivity = stm32f3_devices,
                provides =
                {
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_STM32_TIM1",
                    "HW_STM32_USB",
                    "HW_STM32_CAPSENSE",
                    "HW_STM32_COMP3_4",
                    "HW_STM32_ADC2",
                },
            },
            {
                macro = "MCU_STM32F303",
                brief = "STM32F303 ",
                description = "STM32F30 devices.",
                flavor = "booldata",
                exclusivity = stm32f3_devices,
                provides =
                {
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_STM32_TIM1",
                    "HW_STM32_TIM7",
                    "HW_STM32_TIM8",
                    "HW_STM32_USB",
                    "HW_STM32_CAPSENSE",
                    "HW_STM32_ADC2",
                    "HW_STM32_ADC3",
                    "HW_STM32_ADC4",
                    "HW_STM32_DAC2",
                    "HW_STM32_COMP3_4",
                    "HW_STM32_COMP5_7",
                    "HW_STM32_OP3_4",
                },
                file = "include/cfg/arch.h"
            },
            {
                macro = "MCU_STM32F313",
                brief = "STM32F313 ",
                description = "STM32F30 devices.",
                flavor = "booldata",
                exclusivity = stm32f3_devices,
                provides =
                {
                    "HW_UART4_STM32",
                    "HW_UART5_STM32",
                    "HW_STM32_TIM1",
                    "HW_STM32_TIM7",
                    "HW_STM32_TIM8",
                    "HW_STM32_ADC2",
                    "HW_STM32_ADC3",
                    "HW_STM32_ADC4",
                    "HW_STM32_DAC2",
                    "HW_STM32_COMP3_4",
                    "HW_STM32_COMP5_7",
                    "HW_STM32_OP3_4",
                },
            },
            {
                macro = "MCU_STM32F373",
                brief = "STM32F373 ",
                description = "STM32F373 devices.",
                flavor = "booldata",
                exclusivity = stm32f3_devices,
                provides =
                {
                    "HW_STM32_TIM5_32BIT",
                    "HW_STM32_TIM7",
                    "HW_STM32_TIM12",
                    "HW_STM32_TIM13",
                    "HW_STM32_TIM14",
                    "HW_STM32_TIM18",
                    "HW_STM32_TIM19",
                    "HW_STM32_USB",
                    "HW_STM32_CAPSENSE",
                    "HW_STM32_SDADC1_3",
                    "HW_STM32_DAC2",
                    "HW_STM32_DAC3",
                },
            },
            {
                macro = "MCU_STM32F383",
                brief = "STM32F383 ",
                description = "STM32F383 devices.",
                flavor = "booldata",
                exclusivity = stm32f3_devices,
                provides =
                {
                    "HW_STM32_TIM5_32BIT",
                    "HW_STM32_TIM7",
                    "HW_STM32_TIM12",
                    "HW_STM32_TIM13",
                    "HW_STM32_TIM14",
                    "HW_STM32_TIM18",
                    "HW_STM32_TIM19",
                    "HW_STM32_CAPSENSE",
                    "HW_STM32_SDADC1_3",
                    "HW_STM32_DAC2",
                    "HW_STM32_DAC3",
                },
            },
        }
    },
    {
        name = "nutarch_cm3_stm32F3_memory",
        brief = "STM32F30x Device Memory",
        requires = { "HW_MCU_STM32F3XX" },
        options =
        {
            {
                macro = "MCU_STM32F30X",
                brief = "STM32F30x memory",
                description = "Select your devices memory by the marked alphanumeric code on the chip:\n"..
                              "STM32F30x>Y<zz where Y is one of the list below.\n\n"..
                              "B =   128 kbytes Flash\n"..
                              "C =   256 kbytes Flash\n",
                requires = { "HW_MCU_STM32F30X" },
                type = "enumerated",
                choices = stm32_memory_f30,
                file = "include/cfg/arch.h",
                makedefs = { "HWDEF+=-DSTM32F30X" },
            },
            {
                macro = "MCU_STM32F37X",
                brief = "STM32F37x memory",
                description = "Select your devices memory by the marked alphanumeric code on the chip:\n"..
                              "STM32F37x>Y<zz where Y is one of the list below.\n\n"..
                              "8 =    64 kbytes Flash\n"..
                              "B =   128 kbytes Flash\n"..
                              "C =   256 kbytes Flash\n",
                requires = { "HW_MCU_STM32F37X" },
                type = "enumerated",
                choices = stm32_memory_f37,
                file = "include/cfg/arch.h",
                makedefs = { "HWDEF+=-DSTM32F37X" },
            },
        }
    },
    --
    -- STM32F30 PLL Configuration
    --
    {
        name = "nutarch_cm3_stm32f30_pll",
        brief = "STM32F30 PLL Setup",
        sources =
        {
            "cm3/dev/stm/system_stm32.c",
            "cm3/dev/stm/stm32f30_clk.c"
        },
        requires = { "LICENSE_ST_GUIDANCE_ONLY", "HW_MCU_STM32", "TOOL_CC_CM3", "TOOL_GCC", "HW_PLL_STM32F3" },
        options =
        {
            {
                macro = "SYSCLK_SOURCE",
                brief = "SYSCLK Source",
                description = "Select where SYSCLK should get its clock from.\n\n"..
                              "SYSCLK_HSI is the internal 8MHz clock.\n"..
                              "SYSCLK_PLL is the internal PLL output. Select the source for the PLL in the next option.\n"..
                              "SYSCLK_HSE is the external oscillator or crystal input.\n",
                type = "enumerated",
                choices = { "SYSCLK_HSI", "SYSCLK_PLL", "SYSCLK_HSE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "HSE_BYPASS",
                brief = "HSE from external source",
                description = "Use the clock signal applied to OSC_IN.",
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLLCLK_SOURCE",
                brief = "PLL Clock Source",
                description = "Select where the PLL should get its clock from.\n\n"..
                              "SYSCLK_HSI is the internal 8MHz clock. PLL is fed with SYSCLK_HSI/2.\n"..
                              "SYSCLK_HSE is the external oscillator or crystal input.\n",
                type = "enumerated",
                choices = { "PLLCLK_HSI", "PLLCLK_HSE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_CLOCK_DIV",
                brief = "PLL Clock Prescaler",
                description = "Select this to force the HSE clock beeing divided by 2.",
                requires = { "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_CLOCK_MUL",
                brief = "PLL Multiplier",
                description = "Set this value to override automatic setup of the PLL.\n"..
                              "Respect that the PLL output has to be at maximum 72MHz.\n"..
                              "If USB is used the ouput must provide a frequency of 72MHz or 48MHz.\n"..
                              "This clock is used for audio I2S interface directly.\n",
                requires = { "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "HSE_VALUE",
                brief = "External Oszillator Frequency",
                description = "Value of the external oscillator in Herz.\n"..
                              "Typical Values is: 8 MHz\n.",
                flavor = "booldata",
                type = "long",
                default = "8000000",
                file = "include/cfg/clock.h"
            },
            {
                macro = "SYSCLK_FREQ",
                brief = "CM3 System Clock",
                description = "System clock (SYSCLK) target frequency after PLL setup.\n\n"..
                              "24MHz USB not supported\n"..
                              "36MHz USB not supported\n"..
                              "48MHz USB supported\n"..
                              "56MHz USB not supported\n"..
                              "72MHz USB supported\n\n",
                type = "enumerated",
                choices = stm32_syclk_frequencies,
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_HCLK",
                brief = "AHB Prescaler",
                description = "This is the divider for the AHB bus. It is supplied by the SYSCLK and must not exceed 72MHz.\n"..
                              "The AHB clocks the core, memory, DMA and all other busses.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "Respect that usage of ethernet or USB requires at least 25MHz on this bus.\n",
                requires = { "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_APB1",
                brief = "APB1 Prescaler",
                description = "This is the divider for the slow peripheral bus (APB1). It is upplied by the AHB clock and "..
                              "it must not exceed 36MHz.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "specific BoardInit() function.\n\n",
                requires = { "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_APB2",
                brief = "APB2 Prescaler",
                description = "This is the divider for the fast peripheral bus (APB2). It is upplied by the AHB clock and "..
                              "it must not exceed 72MHz.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "specific BoardInit() function.\n\n",
                requires = { "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
        }
    },

    -- ***********************************
    --
    -- STM32F30 Device Drivers
    --
    -- ***********************************

}
