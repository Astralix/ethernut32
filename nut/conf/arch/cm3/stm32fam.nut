--
-- Copyright (C) 2004-2007 by egnite Software GmbH. All rights reserved.
-- Copyright (C) 2011-2013 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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

-- STM Family selection
--
nutarch_cm3_stm32_family =
{
    --
    -- STM32 Based Cpu Directory
    --
    {
        name = "nutarch_stm32_fam",
        brief = "MCU Family",
        options =
        {
            {
                macro = "MCU_STM32",
                brief = "STM32",
                type = "integer",
                default = 1,
                requires = { "HW_MCU_STM32" },
                file = "include/cfg/arch.h"
            }
        }
    },
    {
        name = "nutarch_cm3_stm32vl10x",
        brief = "STM32VL10X",
        requires = { "HW_MCU_STM32", "HW_MCU_STM32VL10X" },
        description = "ST Microelectronics STM32 VL10X Series",
        makedefs = { "HWDEF+=-DSTM32L1 -DMCU_STM32L1" },
        script = "arch/cm3/stm32vl10x.nut"
    },
    {
        name = "nutarch_cm3_stm32l10x",
        brief = "STM32L10X",
        requires = { "HW_MCU_STM32", "HW_MCU_STM32L10X" },
        description = "ST Microelectronics STM32 L10X Series",
        makedefs = { "HWDEF+=-DSTM32L1 -DMCU_STM32L1" },
        script = "arch/cm3/stm32l10x.nut"
    },
    {
        name = "nutarch_cm3_stm32l15x",
        brief = "STM32L15X",
        requires = { "HW_MCU_STM32", "HW_MCU_STM32L15X" },
        description = "ST Microelectronics STM32 L15X and L16X Series",
        makedefs = { "HWDEF+=-DSTM32L1 -DMCU_STM32L1" },
        script = "arch/cm3/stm32l15x.nut"
    },
    {
        name = "nutarch_cm3_stm32f1",
        brief = "STM32F1",
        requires = { "HW_MCU_STM32", "HW_MCU_STM32F10X" },
        description = "ST Microelectronics STM32 F1 Series",
        makedefs = { "HWDEF+=-DSTM32F1 -DMCU_STM32F1" },
        script = "arch/cm3/stm32f1.nut"
    },
    {
        name = "nutarch_cm3_stm32f2",
        brief = "STM32F2",
        requires = { "HW_MCU_STM32", "HW_MCU_STM32F2XX" },
        description = "ST Microelectronics STM32 F2 Series",
        makedefs = { "HWDEF+=-DSTM32F2 -DMCU_STM32F2" },
        script = "arch/cm3/stm32f2.nut"
    },
    {
        name = "nutarch_cm3_stm32f3",
        brief = "STM32F3",
        requires = { "HW_MCU_STM32", "HW_MCU_STM32F3XX" },
        provides = { "STM32F3XX" },
        description = "ST Microelectronics STM32 F3 Series",
        makedefs = { "HWDEF+=-DSTM32F3 -DMCU_STM32F3" },
        script = "arch/cm3/stm32f3.nut"
    },
    {
        name = "nutarch_cm3_stm32f4",
        brief = "STM32F4",
        requires = { "HW_MCU_STM32", "HW_MCU_STM32F4XX" },
        description = "ST Microelectronics STM32 F4 Series",
        makedefs = { "HWDEF+=-DSTM32F4 -DMCU_STM32F4" },
        script = "arch/cm3/stm32f4.nut"
    },
    {
        name = "nutarch_cm3_stm32_devices",
        brief = "Common devices",
        description = "Common devices to the STM32 families",
        requires = { "HW_MCU_STM32" },
        script = "arch/cm3/stm32dev.nut"
    },
}

