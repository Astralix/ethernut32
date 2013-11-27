--
--  * Copyright (C) 2011 by Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

stm32_memory_l1xx = { "32" , "64" , "128" , "256" , "384" }

stm32l15x_device_class = { "STML15X_MD" ,"STML15X_MDP", "STML15X_HD" }

-- *****************************************************************************
-- STM32L1 Family
-- *****************************************************************************
--

nutarch_cm3_stm32l15x =
{

    --
    -- MCU Family
    --
    {
        name = "nutarch_cm3_stm32L15x_family",
        brief = "MCU L15X Family",
        requires = { "HW_MCU_STM32L15X" },
        options =
        {
            {
                macro = "MCU_STM32L15X",
                brief = "STM32L15X",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_CM3" },
                provides =
                {
                	"MCU_STM32L1",
                    "HW_PLL_STM32L1",
                    "HW_RCC_STM32",
                    "HW_FLASH_STM32L1",
                    "HW_EEPROM_STM32L1",
                    "HW_GPIO",
                    "HW_GPIO_STM32V2",
                    "HW_CRC32_STM32",
                    "DEV_IRQ_STM32",
                    "HW_DMA1_STM32F1",
                    "HW_EXTI04_STM32",
                    "HW_EXTI95_STM32",
                    "HW_EXTI1510_STM32",
                    "HW_I2C1_STM32",
                    "HW_I2C2_STM32",
                    "HW_SPI1_STM32",
                    "HW_SPI2_STM32",
                    "HW_UART1_STM32",
                    "HW_UART2_STM32",
                    "HW_UART3_STM32",
                    "HW_STM32_TIM2",
                    "HW_STM32_TIM3",
                    "HW_STM32_TIM4",
                    "HW_STM32_TIM6",
                    "HW_STM32_TIM7",
                    "HW_STM32_TIM9",
                    "HW_STM32_TIM10",
                    "HW_STM32_TIM11",
                    "HW_STM32L1_MCO",
                },
                file = "include/cfg/arch.h"
            }
        }
    },
    --
    -- STM32L15X MCU Classes
    --
    {
        name = "nutarch_cm3_stm32l15x_class",
        brief = "STM32L15x Device Classes",
        requires = { "HW_MCU_STM32" },
        options =
        {
            {
                macro = "STM32L15X_MD",
                brief = "STM32L15X MD Series",
                description = "STM32L15X and STM32L16X Medium Density devices.",
                flavor = "booldata",
                exclusivity = stm32l15x_device_class,
                makedefs = { "HWDEF+=-DSTM32L1XX_MD" },
                provides = { "STM32L1XX_MD", "STM32L15X_MD" },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L15X_MDP",
                brief = "STM32L15X MDP Series",
                description = "STM32L15X and STM32L16X Medium Density Plus devices.",
                flavor = "booldata",
                exclusivity = stm32l15x_device_class,
                makedefs = { "HWDEF+=-DSTM32L1XX_MDP" },
                provides = { "STM32L1XX_MDP", "STM32L15X_MDP" },
                file = "include/cfg/arch.h"
            },
            {
                macro = "STM32L15X_HD",
                brief = "STM32L15X HD Series",
                description = "STM32L15X and STM32L16X High Density devices.",
                flavor = "booldata",
                exclusivity = stm32l15x_device_class,
                makedefs = { "HWDEF+=-DSTM32L1XX_HD" },
                provides = { "STM32L1XX_HD", "STM32L15X_HD" },
                file = "include/cfg/arch.h"
            },
        }
    },
    {
        name = "nutarch_cm3_stm32l15x_memory",
        brief = "STM32L15X Device Memory",
        requires = { "HW_MCU_STM32" },
        options =
        {
            {
                macro = "MCU_STM32L15X",
                brief = "STM32L15X memory",
                description = "Select your devices memory by the marked alphanumeric code on the chip:\n"..
                              "STM32L1xx>Y<zz where Y is one of the list below.\n\n"..
                              "6 =   32 kbytes Flash\n"..
                              "8 =   64 kbytes Flash\n"..
                              "B =  128 kbytes Flash\n",
                              "C =  256 kbytes Flash\n",
                              "D =  384 kbytes Flash\n",
                requires = { "HW_MCU_STM32L15X" },
                type = "enumerated",
                choices = stm32_memory_l1xx,
                file = "include/cfg/arch.h"
            },
        }
    },
    --
    -- STM32L1 Clock Configuration
    --
    {
        name = "nutarch_cm3_stm32L15x_clk",
        brief = "STM32L15X Clocks Setup",
        sources = { "cm3/dev/stm/system_stm32.c",
                    "cm3/dev/stm/stm32l1_clk.c"
                  },
        requires = { "LICENSE_MCD_ST_LIBERTY", "LICENSE_ST_GUIDANCE_ONLY", "HW_MCU_STM32", "TOOL_CC_CM3", "TOOL_GCC" },
        options =
        {
            {
                macro = "SYSCLK_SOURCE",
                brief = "SYSCLK Source",
                description = "Select where SYSCLK should get its clock from.\n\n"..
                              "SYSCLK_MSI is the internal configureable medium speed clock.\n"..
                              "SYSCLK_HSI is the internal 16MHz clock.\n"..
                              "SYSCLK_PLL is the internal PLL output. Select the source for the PLL in the next option.\n"..
                              "SYSCLK_HSE is the external oscillator or crystal input.\n",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
                choices = { "SYSCLK_MSI", "SYSCLK_HSI", "SYSCLK_PLL", "SYSCLK_HSE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "HSI_VALUE",
                brief = "Internal High Speed Oszillator Frequency",
                description = "Value of the external oscillator in Herz.\n"..
                              "Typical Values is 16MHz.\n"..
                              "If 0 is selected, the clock setup system will disable this "..
                              "oscillator.",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
                choices = { "0", "16000000" },
                default = "16000000",
                file = "include/cfg/clock.h"
            },
            {
                macro = "MSI_VALUE",
                brief = "Internal Medium Speed Oszillator Frequency",
                description = "Value of the internal medium speed oscillator in Herz.\n"..
                              "After reset this oscillator is active.\n"..
                              "If 0 is chosen, the clock setup system will disable this "..
                              "oscillator after switching to the selected SYSCLK_SOURCE."..
                              "Frequency Table:\n"..
                              " 0: OFF\n 1: 65.536kHz\n 2: 131.072kHz\n"..
                              " 3: 262.144kHz\n 4: 524.288kHz\n"..
                              " 5: 1048000MHz\n 6: 2.097000MHz(Reset Default)\n"..
                              " 7: 4.194000MHz",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
                choices = { "0", "1", "2", "3", "4", "5", "6", "7" },
                default = "1",
                file = "include/cfg/clock.h"
            },
            {
                macro = "HSE_VALUE",
                brief = "External Oszillator Frequency",
                description = "Value of the external oscillator in Herz.\n"..
                              "Typical Values is 8MHz.",
                requires = { "HW_PLL_STM32L1" },
                type = "long",
                default = "8000000",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLLCLK_SOURCE",
                brief = "PLL Clock Source",
                description = "Select where the PLL should get its clock from.\n\n"..
                              "PLLCLK_NONE switches the PLL off.\n"..
                              "PLLCLK_HSI is the internal 16MHz clock.\n"..
                              "PLLCLK_HSE is the external oscillator or crystal input.\n",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
                choices = { "PLL_NONE", "PLLCLK_HSI", "PLLCLK_HSE" },
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_CLOCK_DIV",
                brief = "PLL Clock Prescaler",
                description = "Select this to force the HSE clock beeing divided by 2.",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_CLOCK_MUL",
                brief = "PLL Multiplier",
                description = "Set this value to override automatic setup of the PLL.\n"..
                              "Respect that the PLL output has to be at maximum 32MHz.\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "SYSCLK_FREQ",
                brief = "CM3 System Clock",
                description = "System clock (SYSCLK) target frequency after PLL setup.",
                requires = { "HW_PLL_STM32L1" },
                type = "enumerated",
--                choices = stm32_syclk_frequencies,
                type = "long",
                default = "16000000",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_HCLK",
                brief = "AHB Prescaler",
                description = "This is the divider for the AHB bus. It is supplied by the SYSCLK and must not exceed 72MHz.\n"..
                              "The AHB clocks the core, memory, DMA and all other busses.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_APB1",
                brief = "APB1 Prescaler",
                description = "This is the divider for the peripheral bus (APB1). It is upplied by the AHB clock.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "specific BoardInit() function.\n\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            },
            {
                macro = "PLL_DIV_APB2",
                brief = "APB2 Prescaler",
                description = "This is the divider for the fast peripheral bus (APB2). It is upplied by the AHB clock.\n"..
                              "To override auto calculation enter a value n here where the division is 2^n\n"..
                              "where a value of 0 disables the prescaler and the auto-calculation.\n"..
                              "specific BoardInit() function.\n\n",
                requires = { "HW_PLL_STM32L1", "DISABLED" },
                flavor = "booldata",
                file = "include/cfg/clock.h"
            }
        }
    },

    -- ***********************************
    --
    -- STM32L Device Drivers
    --
    -- ***********************************

}
