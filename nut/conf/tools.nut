--
-- Copyright (C) 2004-2005 by egnite Software GmbH. All rights reserved.
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

-- Tools
--
--
--

toolchain_names = {"ARM_GCC", "ARM_GCC_NOLIBC", "CM3_GCC", "CM3_GCC_NOLIBC" }
gcc_output_format = {"ARMELF", "ARMEABI"}
nuttools =
{
    options =
    {
        {
            brief = "GCC for ARM",
            description = "GNU Compiler Collection for ARM including libc.",
            provides = { "TOOL_CC_ARM", "TOOL_GCC", "TOOL_CXX", "TOOL_ARMLIB" },
            macro = "ARM_GCC",
            flavor = "boolean",
            exclusivity = toolchain_names,
            file = "include/cfg/arch.h"
        },
        {
            brief = "GCC for ARM (no libc)",
            description = "GNU Compiler Collection for ARM excluding libc."..
                          "Nut/OS provides all required C standard functions.",
            provides = { "TOOL_CC_ARM", "TOOL_GCC", "TOOL_CXX", "TOOL_NOLIBC" },
            macro = "ARM_GCC_NOLIBC",
            flavor = "boolean",
            exclusivity = toolchain_names,
            file = "include/cfg/arch.h",
            makedefs = { "ADDLIBS = -lnutc" }
        },
        {
            brief = "GCC for CortexM",
            description = "GNU Compiler Collection for ARM CortexM including libc.",
            provides = { "TOOL_CC_CM3", "TOOL_GCC", "TOOL_CXX", "TOOL_ARMLIB" },
            macro = "CM3_GCC",
            flavor = "boolean",
            exclusivity = toolchain_names,
            file = "include/cfg/arch.h"
        },
        {
            brief = "GCC for CortexM (no libc)",
            description = "GNU Compiler Collection for ARM CortexM excluding libc."..
                          "Nut/OS provides all required C standard functions.",
            provides = { "TOOL_CC_CM3", "TOOL_GCC", "TOOL_CXX", "TOOL_NOLIBC" },
            macro = "CM3_GCC_NOLIBC",
            flavor = "boolean",
            exclusivity = toolchain_names,
            file = "include/cfg/arch.h",
            makedefs = { "ADDLIBS = -lnutc" }
        },
        {
            brief = "GCC for AVR32",
            description = "GNU Compiler Collection for AVR32 including libc.",
            provides = { "TOOL_CC_AVR32", "TOOL_GCC", "TOOL_CXX" },
            macro = "AVR32_GCC",
            flavor = "boolean",
            exclusivity = toolchain_names,
            file = "include/cfg/arch.h",
        },
    },
    {
        name = "nuttools_gccopt",
        brief = "GCC Settings",
        requires = { "TOOL_GCC" },
        options =
        {
            {
                macro = "LDSCRIPT",
                brief = "Predefined Linker Script",
                description = function() return GetLDScriptDescription(); end,
                requires = { "TOOL_GCC" },
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetLDScripts(); end,
                makedefs = function() return { "LDNAME", "LDSCRIPT=$(LDNAME).ld", "LDPATH=" .. GetLDScriptsPath() }; end,
                exclusivity = { "LDSCRIPT", "LLDSCRIPT" },
            },
            {
                macro = "LLDSCRIPT",
                brief = "Local Linker Script",
                description = "Alternatively a local linker script file name can be provided.",
                requires = { "TOOL_GCC" },
                flavor = "booldata",
                type = "bool",
                makedefs = { "LDNAME", "LDSCRIPT=" .. "$(LDNAME)"},
                exclusivity = { "LDSCRIPT", "LLDSCRIPT" },
            },
            {
                brief = "arm-elf",
                description = "Old binary format",
                requires = { "TOOL_CC_ARM" },
                macro = "ARMELF",
                flavor = "boolean",
                exclusivity = gcc_output_format,
                makedefs = { "TRGT = arm-elf-" }
           },
            {
                brief = "arm-none-eabi",
                description = "New binary format",
                requires = { "TOOL_CC_ARM" },
                macro = "ARMEABI",
                flavor = "boolean",
                exclusivity = gcc_output_format,
                makedefs = { "TRGT = arm-none-eabi-" }
            }
        }
    }
    --
    -- Intentionally no programmer or urom creator specified.
    -- This will be part of the application wizard.
    --
}

avr32_ld_description =
{
    uc3a0512_rom        = "AVR32UC3A0512, code running in FLASH",
    uc3a0512_rom_extram = "AVR32UC3A0512, code running in FLASH, data in external SDRAM",
    uc3a3256_rom        = "AT32UC3A3256, code running in FLASH",
    uc3a3256_rom_extram = "AT32UC3A3256, code running in FLASH, data in external SDRAM",
    uc3a0256_rom        = "AT32UC3A0256, code running in FLASH",
    uc3b164_rom         = "AT32UC3B164, code running in FLASH",
    uc3l064_rom         = "AT32UC3B164, code running in FLASH",
}


avr32_ld_choice =
{
    " ",
    "uc3a0512_rom",
    "uc3a0512_rom_extram",
    "uc3a3256_rom",
    "uc3a3256_rom_extram",
    "uc3a0256_rom",
    "uc3b164_rom",
    "uc3l064_rom"
}

arm_ld_description =
{
    at91_boot                = "AT91R40008, code in ROM, copied to and running in RAM",
    at91_bootcrom            = "AT91R40008, code in ROM copied to RAM, but constant data remains in ROM.",
    at91_bootloader_bootcrom = "AT91R40008, code copied to RAM, but consts remain in ROM. Started by bootloader at address 0x10000",
    at91_ram                 = "AT91R40008, code loaded in RAM (deprecated, use at91x40_ram)",
    at91_rom                 = "AT91R40008, code running in FLASH (deprecated, use at91x40_rom)",
    at91_httprom             = "AT91R40008, code running in FLASH. Use this with boothttp.",
    at91sam7x128_rom         = "AT91SAM7X128, code running in FLASH",
    at91sam7x256_rom         = "AT91SAM7X256, code running in FLASH",
    at91sam7x256_bootrom     = "AT91SAM7X256, code running in FLASH, bootloader in FLASH (code entry at offset 0xC000)",
    at91sam7x512_rom         = "AT91SAM7X512, code running in FLASH",
    at91sam7x512_bootrom     = "AT91SAM7X512, code running in FLASH, bootloader in FLASH (code entry at offset 0xC000)",
    at91sam7s16_rom          = "AT91SAM7S16, code running in FLASH",
    at91sam7s32_rom          = "AT91SAM7S32, code running in FLASH",
    at91sam7s64_rom          = "AT91SAM7S64, code running in FLASH",
    at91sam7s64_bootrom      = "AT91SAM7S64, code running in FLASH, bootloader in FLASH",
    at91sam7s128_rom         = "AT91SAM7S128, code running in FLASH",
    at91sam7s256_rom         = "AT91SAM7S256, code running in FLASH",
    at91sam7s256_bootrom     = "AT91SAM7S256, code running in FLASH, bootloader in FLASH",
    at91sam7s512_rom         = "AT91SAM7S512, code running in FLASH",
    at91sam7se32_rom         = "AT91SAM7SE32, code running in FLASH, data in SDRAM",
    at91sam7se32_xram        = "AT91SAM7SE32, code loaded into external RAM",
    at91sam7se256_rom        = "AT91SAM7SE256, code running in FLASH, data in SDRAM",
    at91sam7se256_xram       = "AT91SAM7SE256, code loaded into external RAM",
    at91sam7se512_rom        = "AT91SAM7SE512, code running in FLASH, data in SDRAM",
    at91sam7se512_ram        = "AT91SAM7SE512, code in SDRAM (deprecated, use at91sam7se512_xram)",
    at91sam7se512_xram       = "AT91SAM7SE512, code loaded into external RAM",
    at91sam9260_ram          = "AT91SAM9260, code running in external RAM",
    at91sam9G45_ram          = "AT91SAM9G45, code running in external RAM",
    at91sam9xe512_ram        = "AT91SAM9XE512, code running in external RAM",
    at91x40_ram              = "AT91X40, code loaded in RAM by bootloader/debugger",
    at91x40_rom              = "AT91X40, code running in FLASH",
    ["s3c4510b-ram"]         = "Samsung's S3C4510B, code in RAM (unsupported)",
    eb40a_ram                = "Atmel's AT91EB40A, code in RAM at 0x100",
    gba_xport2               = "Nintendo's Gameboy Advance"
}

arm_ld_choice =
{
    " ",
    "at91_boot",
    "at91_bootcrom",
    "at91_bootloader_bootcrom",
    "at91_ram",
    "at91_rom",
    "at91_httprom",
    "at91sam7s16_rom",
    "at91sam7s32_rom",
    "at91sam7s64_rom",
    "at91sam7s64_bootrom",
    "at91sam7s128_rom",
    "at91sam7s256_rom",
    "at91sam7s256_bootrom",
    "at91sam7s512_rom",
    "at91sam7se32_rom",
    "at91sam7se32_xram",
    "at91sam7se256_rom",
    "at91sam7se256_xram",
    "at91sam7se512_rom",
    "at91sam7se512_ram",
    "at91sam7se512_xram",
    "at91sam7x128_rom",
    "at91sam7x256_rom",
    "at91sam7x256_bootrom",
    "at91sam7x512_rom",
    "at91sam7x512_bootrom",
    "at91sam9260_ram",
    "at91sam9G45_ram",
    "at91sam9xe512_ram",
    "at91x40_ram",
    "at91x40_rom",
    "s3c4510b-ram",
    "eb40a_ram",
    "eb40a_redboot_ram",
    "gbaxport2",
    "zero"
}

stm32f10x_ld_header = { "Select the matching predefined linker script for your chip\n\n" }

stm32f100_ld_description =
{
    stm32f100x8_flash = "STM32F100x8, code running in FLASH, data in SRAM",
    stm32f100xB_flash = "STM32F100xB, code running in FLASH, data in SRAM",
    stm32f100xC_flash = "STM32F100xC, code running in FLASH, data in SRAM",
    stm32f100xD_flash = "STM32F100xD, code running in FLASH, data in SRAM",
    stm32f100xE_flash = "STM32F100xE, code running in FLASH, data in SRAM",
}

stm32f101_ld_description =
{
    stm32f101x8_flash = "STM32F101x8 and STM32F102x8, code running in FLASH, data in SRAM",
    stm32f101xB_flash = "STM32F101xB and STM32F102xB, code running in FLASH, data in SRAM",
    stm32f101xC_flash = "STM32F101xC and STM32F102xC, code running in FLASH, data in SRAM",
    stm32f101xD_flash = "STM32F101xD and STM32F102xD, code running in FLASH, data in SRAM",
    stm32f101xE_flash = "STM32F101xE and STM32F102xE, code running in FLASH, data in SRAM",
}

stm32f102_ld_description =
{
    stm32f102x8_flash = "STM32F102x8 and STM32F102x8, code running in FLASH, data in SRAM",
    stm32f102xB_flash = "STM32F102xB and STM32F102xB, code running in FLASH, data in SRAM",
}

stm32f103_ld_description =
{
    stm32f103x8_flash = "STM32F103x8 and STM32F102x8, code running in FLASH, data in SRAM",
    stm32f103xB_flash = "STM32F103xB and STM32F102xB, code running in FLASH, data in SRAM",
    stm32f103xC_flash = "STM32F103xC and STM32F102xC, code running in FLASH, data in SRAM",
    stm32f103xD_flash = "STM32F103xD and STM32F102xD, code running in FLASH, data in SRAM",
    stm32f103xE_flash = "STM32F103xE and STM32F102xE, code running in FLASH, data in SRAM",
}

stm32f105_ld_description =
{
    stm32f105x8_flash = "STM32F105x8 and STM32F102x8, code running in FLASH, data in SRAM",
    stm32f105xB_flash = "STM32F105xB and STM32F102xB, code running in FLASH, data in SRAM",
    stm32f105xC_flash = "STM32F105xC and STM32F102xC, code running in FLASH, data in SRAM",
}

stm32f107_ld_description =
{
    stm32f107xB_flash = "STM32F107xB and STM32F102xB, code running in FLASH, data in SRAM",
    stm32f107xC_flash = "STM32F107xC and STM32F102xC, code running in FLASH, data in SRAM",
}

stm32f2x5_ld_description =
{
    stm32f205xB_flash = "STM32F205xB and STM32F215xB, code running in FLASH, data in SRAM",
    stm32f205xC_flash = "STM32F205xC and STM32F215xC, code running in FLASH, data in SRAM",
    stm32f205xE_flash = "STM32F205xE and STM32F215xE, code running in FLASH, data in SRAM",
    stm32f205xF_flash = "STM32F205xF and STM32F215xF, code running in FLASH, data in SRAM",
    stm32f205xG_flash = "STM32F205xG and STM32F215xG, code running in FLASH, data in SRAM",
}

stm32f2x7_ld_description =
{
    stm32f207xC_flash = "STM32F207xC and STM32F217xC, code running in FLASH, data in SRAM",
    stm32f207xE_flash = "STM32F207xE and STM32F217xE, code running in FLASH, data in SRAM",
    stm32f207xF_flash = "STM32F207xF and STM32F217xF, code running in FLASH, data in SRAM",
    stm32f207xG_flash = "STM32F207xG and STM32F217xG, code running in FLASH, data in SRAM",
}

stm32l15x_ld_description =
{
    stm32l15xx6_flash = "STM32L151x6 and STM32L152x6, code running in FLASH, data in SRAM",
    stm32l15xx8_flash = "STM32L151x8 and STM32L152x8, code running in FLASH, data in SRAM",
    stm32l15xxB_flash = "STM32L151xB and STM32L152xB, code running in FLASH, data in SRAM",
    stm32l15xxC_flash = "STM32L151xB and STM32L152xC, code running in FLASH, data in SRAM",
}

stm32f100_ld_choice =
{
    "stm32f100x8_flash",
    "stm32f100xB_flash",
    "stm32f100xC_flash",
    "stm32f100xD_flash",
    "stm32f100xE_flash",
}

stm32f101_ld_choice =
{
    "stm32f101x8_flash",
    "stm32f101xB_flash",
    "stm32f101xC_flash",
    "stm32f101xD_flash",
    "stm32f101xE_flash",
}

stm32f102_ld_choice =
{
    "stm32f102x8_flash",
    "stm32f102xB_flash",
}

stm32f103_ld_choice =
{
    "stm32f103x8_flash",
    "stm32f103xB_flash",
    "stm32f103xC_flash",
    "stm32f103xD_flash",
    "stm32f103xE_flash",
}
stm32f105_ld_choice =
{
    "stm32f105x8_flash",
    "stm32f105xB_flash",
    "stm32f105xC_flash",
}
stm32f107_ld_choice =
{
    "stm32f107xB_flash",
    "stm32f107xC_flash",
}

stm32f2x5_ld_choice =
{
    "stm32f2x5xB_flash",
    "stm32f2x5xC_flash",
    "stm32f2x5xE_flash",
    "stm32f2x5xF_flash",
    "stm32f2x5xG_flash",
}

stm32f2x7_ld_choice =
{
    "stm32f2x7xC_flash",
    "stm32f2x7xE_flash",
    "stm32f2x7xF_flash",
    "stm32f2x7xG_flash",
}

stm32l15x_ld_choice =
{
    "stm32l15Xx6_flash",
    "stm32l15Xx8_flash",
    "stm32l15XxB_flash",
    "stm32l15XxC_flash",
}

lm3_ld_description =
{
    lm3s9b96_flash = "LM3S9B96, code running in FLASH, data in SRAM",
}
lm3_ld_choice =
{
    "lm3s9b96_flash",
}

lpc17xx_ld_description =
{
    lpc1768_flash = "LPC1768, code running in FLASH, data in SRAM",
    lpc1778_flash = "LPC1778/LPC1788, code running in FLASH, data in SRAM",
}
lpc17xx_ld_choice =
{
    "lpc1768_flash",
    "lpc1778_flash",
}

--
-- Retrieve platform specific ldscript path.
--
function GetLDScriptsPath()
    local basepath

    basepath = "$(top_srcdir)/arch/"
    if c_is_provided("TOOL_CC_AVR32") then
        return basepath .. "avr32/ldscripts"
    end
    if c_is_provided("TOOL_CC_ARM") then
        return basepath .. "arm/ldscripts"
    end
    if c_is_provided("TOOL_CC_CM3") then
        return basepath .. "cm3/ldscripts"
    end
    return "Unknown Platform - Check GetLDScriptsPath in tools.nut"
end

--
-- Return the list of ldscripts
--
function GetLDScripts()
    if c_is_provided("TOOL_CC_AVR32") then
        return avr32_ld_choice
    end
    if c_is_provided("TOOL_CC_ARM") then
        return arm_ld_choice
    end
    if c_is_provided("TOOL_CC_CM3") then
         if c_is_provided("MCU_STM32F100") then
              return stm32f100_ld_choice
         end
         if c_is_provided("MCU_STM32F101") then
              return stm32f101_ld_choice
         end
        if c_is_provided("MCU_STM32F102") then
            return stm32f102_ld_choice
        end
        if c_is_provided("MCU_STM32F103") then
            return stm32f103_ld_choice
        end
        if c_is_provided("MCU_STM32F105") then
            return stm32f105_ld_choice
        end
        if c_is_provided("MCU_STM32F107") then
            return stm32f107_ld_choice
        end
        if c_is_provided("MCU_STM32F205") then
            return stm32f2x5_ld_choice
        end
        if c_is_provided("MCU_STM32F207") then
            return stm32f2x7_ld_choice
        end
        if c_is_provided("HW_MCU_STM32L15X") then
            return stm32l15x_ld_choice
        end
        if c_is_provided("HW_MCU_STM32L10X") then
            return stm32l15x_ld_choice
        end
        if c_is_provided("HW_MCU_STM32VL10X") then
            return stm32l15x_ld_choice
        end
        if c_is_provided("HW_MCU_LM3") then
            return lm3_ld_choice
        end
        if c_is_provided("HW_MCU_LPC17xx") then
            return lpc17xx_ld_choice
        end
    end
end

--
-- Return the ldscript description
--
function GetLDScriptDescription()
    if c_is_provided("TOOL_CC_AVR32") then
        return FormatLDScriptDescription(avr32_ld_description)
    end
    if c_is_provided("TOOL_CC_ARM") then
        return FormatLDScriptDescription(arm_ld_description)
    end
    if c_is_provided("TOOL_CC_CM3") then
       if c_is_provided("MCU_STM32F100") then
           return FormatLDScriptDescription(stm32f100_ld_description)
       end
       if c_is_provided("MCU_STM32F101") then
           return FormatLDScriptDescription(stm32f101_ld_description)
       end
       if c_is_provided("MCU_STM32F102") then
           return FormatLDScriptDescription(stm32f102_ld_description)
       end
       if c_is_provided("MCU_STM32F103") then
           return FormatLDScriptDescription(stm32f103_ld_description)
       end
       if c_is_provided("MCU_STM32F105") then
           return FormatLDScriptDescription(stm32f105_ld_description)
       end
       if c_is_provided("MCU_STM32F107") then
           return FormatLDScriptDescription(stm32f107_ld_description)
       end
       if c_is_provided("MCU_STM32F205") then
           return FormatLDScriptDescription(stm32f2x5_ld_description)
       end
       if c_is_provided("MCU_STM32F207") then
           return FormatLDScriptDescription(stm32f2x7_ld_description)
       end
       if c_is_provided("MCU_STM32L15X") then
           return FormatLDScriptDescription(stm32l15x_ld_description)
       end
       if c_is_provided("HW_MCU_LM3") then
           return FormatLDScriptDescription(lm3_ld_description)
       end
       if c_is_provided("HW_MCU_LPC17xx") then
           return FormatLDScriptDescription(lpc17xx_ld_description)
       end
    end
	if c_is_provided("TOOL_CC_M68K") then
	   if c_is_provided("MCU_MCF5525X") then
           return FormatLDScriptDescription(mcf5225x_ld_description)
       end
       if c_is_provided("MCU_MCF51CN") then
           return FormatLDScriptDescription(mcf51cn_ld_description)
       end
           return ""
	end
end

--
-- Returns pairs sorted by keys in alphabetic order
--
function pairsByKeys (t, f)
    local a = {}
    -- build temporary table of the keys
    for n in pairs (t) do
        table.insert (a, n)
    end
    table.sort (a, f)  -- sort using supplied function, if any
    local i = 0        -- iterator variable
    return function () -- iterator function
        i = i + 1
        return a[i], t[a[i]]
    end  -- iterator function
end -- pairsByKeys

--
-- Return the formated ldscript description
-- Note: It looks like it's impossible to properly
-- format based on tabs on both nutconf and qnutconf using
-- non-fixed length fonts. When we move definitively to qnutconf
-- this can be made to output html, then we will get it right.
--
function FormatLDScriptDescription( t )
    local result = ""
    local maxKeyLen = 0;
    for k,v in pairs(t) do
        if maxKeyLen < string.len(k) then
            maxKeyLen = string.len(k)
        end
    end

    maxTabs = math.ceil( maxKeyLen / 6 + 1 );
    for k,v in pairsByKeys(t) do
        tabs = maxTabs - math.ceil( string.len(k) / 6 );
        result = result .. k
        for i = 1, tabs do
            result = result .. "\t"
        end
        result = result .. v .. "\n"
    end
    return result
end

