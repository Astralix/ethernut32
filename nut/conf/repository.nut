--
-- Copyright (C) 2005-2007 by egnite Software GmbH. All rights reserved.
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


-- The repository contains an enumerated list
-- of all top-level components.
--
--

mcu_names = {
    "MCU_AT91SAM9260",
    "MCU_AT91SAM9G45",
    "MCU_AT91SAM9XE512",
    "MCU_AT91SAM7X128",
    "MCU_AT91SAM7X256",
    "MCU_AT91SAM7X512",
    "MCU_AT91SAM7S16",
    "MCU_AT91SAM7S32",
    "MCU_AT91SAM7S64",
    "MCU_AT91SAM7S128",
    "MCU_AT91SAM7S256",
    "MCU_AT91SAM7S512",
    "MCU_AT91SAM7SE32",
    "MCU_AT91SAM7SE256",
    "MCU_AT91SAM7SE512",
    "MCU_AT91R40008",
    "MCU_AVR32UC3A0512ES",
    "MCU_AVR32UC3A0512",
    "MCU_AVR32UC3A3256",
    "MCU_AVR32UC3B0256",
    "MCU_AVR32UC3B164",
    "MCU_AVR32UC3L064",
    "MCU_STM32F101",
    "MCU_STM32F103",
    "MCU_STM32F105",
    "MCU_STM32F107",
    "MCU_STM32F205",
    "MCU_STM32F207",
    "MCU_STM32F302",
    "MCU_STM32F303",
    "MCU_STM32F313",
    "MCU_STM32F373",
    "MCU_STM32F383",
    "MCU_STM32F401",
    "MCU_STM32F405",
    "MCU_STM32F407",
    "MCU_STM32F415",
    "MCU_STM32F417",
    "MCU_STM32F427",
    "MCU_STM32F429",
    "MCU_STM32F437",
    "MCU_STM32F439",
    "MCU_LM3S9B96",
    "MCU_LPC1758",
    "MCU_LPC1768",
    "MCU_LPC1778",
    "MCU_MCF52259", 
    "MCU_MCF51CN128",
    "MCU_ZERO"
}

mcu_32bit_choice =
{
    " ",
    "0", "1", "2", "3", "4", "5", "6", "7",
    "8", "9", "10", "11", "12", "13", "14", "15",
    "16", "17", "18", "19", "20", "21", "22", "23",
    "24", "25", "26", "27", "28", "29", "30", "31"
}

stm32_bit_choice =
{
    " ",
    "0", "1", "2", "3", "4", "5", "6", "7",
    "8", "9", "10", "11", "12", "13", "14", "15"
}
mcf5_bit_choice = { " ",
                    "0", "1", "2", "3", "4", "5", "6", "7" }
gpio_port_choice =
{
    " ",
    "NUTGPIO_PORTA",
    "NUTGPIO_PORTB",
    "NUTGPIO_PORTC",
    "NUTGPIO_PORTD",
    "NUTGPIO_PORTE",
    "NUTGPIO_PORTF",
    "NUTGPIO_PORTG",
    "NUTGPIO_PORTH",
    "NUTGPIO_PORTI",
    "NUTGPIO_PORTJ",
    "NUTGPIO_PORTK",
    "NUTGPIO_PORTL"
}

gpio_irq_choice =
{
    " ",
    "NUTGPIO_EXTINT0",
    "NUTGPIO_EXTINT1",
    "NUTGPIO_EXTINT2",
    "NUTGPIO_EXTINT3",
    "NUTGPIO_EXTINT4",
    "NUTGPIO_EXTINT5",
    "NUTGPIO_EXTINT6",
    "NUTGPIO_EXTINT7"
}

at91_pio_id_choice = { " ", "PIO_ID", "PIOA_ID", "PIOB_ID", "PIOC_ID" }

avr32_pio_id_choice = { " ", "PIO_ID", "PIOA_ID", "PIOB_ID", "PIOC_ID" }

pll_clk_choice = { " ", "0", "1", "2", "3", "4" }

hd44780_databits_choice = { " ", "0xFF", "0xF0", "0x0F" }

pca9555_port_choice = { "IOXP_PORT0", "IOXP_PORT1" }
pca9555_pin_choice = { " ", "0", "1", "2", "3", "4", "5", "6", "7" }

--
-- Check for custom configuration.
--
-- Executes custom/custom.nut, if it exists. This allows to extend the
-- configuration without modifying any existing file.
--
-- For example, an application may provide its own file system driver.
-- But mkdir, rmdir and similar function will become available only,
-- if certain requirements are provided. This can be done by creating
-- the file custom.nut with the following contents:
--
-- nutcustom =
-- {
--   {
--     name = "nutcustom_cfs",
--     brief = "CFS",
--     description = "Custom file system driver.",
--     provides = {
--       "NUT_FS",
--       "NUT_FS_READ",
--       "NUT_FS_WRITE",
--       "NUT_FS_DIR"
--     }
--   }
-- }
--
-- Developer's note: The custom directory is reserved for this kind of
-- extensions and must not exist in the official repository. However,
-- it may be included in specialized Nut/OS distributions.
--
function CheckCustomConfig()
    local fp

    fp = io.open(c_repo_path() .. "/custom/custom.nut", "r")
    if fp == nil then
        return {}
    end
    fp:close()
    return
        {
            name = "nutcustom",
            brief = "Custom Configuration",
            description = "Allows to add custom specific configuration items.\n",
            subdir = "custom",
            script = "custom/custom.nut"
        }
end

repository =
{
    {
        name = "nutinfo",
        brief = "Hardware Platform",
        description = "Board specific settings.",
        options =
        {
            {
                macro = "PLATFORM",
                brief = "Platform Macro",
                description = "String constant identifying the target hardware used.\n\n"..
                      "Examples are ETHERNUT1, ETHERNUT2 etc. "..
                      "This string constant is passed as a macro definition to "..
                      "the Makefiles and allows conditional compilation "..
                      "depending on the hardware used.\n\n"..
                      "If required, the lower case variant of this string will be used "..
                      "as a source file name containing board specific functions. Such files "..
                      "are located in directory arch/MCU/board.",
                provides = { "HW_TARGET_BOARD" },
                flavor = "booldata",
                file = "include/cfg/arch.h",
                makedefs = { "PLATFORM", "HWDEF+=-D$(PLATFORM)" }
            },
            {
                macro = "PLATFORM_SUB",
                brief = "User Platform Macro",
                description = "String constant identifying a user specific hardware that is based on the "..
                      "PLATFORM macro above.\n\n"..
                      "This macro can be used if a project is developed for different targets "..
                      "that are based mainly on the same base project and hardware.\n\n"..
                      "An example: You develop different EIR applications, same chip, same board, but "..
                      "one with an OLED, another with an LC-display and a third without display but IR-remote\n\n"..
                      "This string constant is passed as a macro definition to "..
                      "the Makefiles and allows conditional compilation "..
                      "depending on the hardware used.",
                flavor = "booldata",
                file = "include/cfg/arch.h",
                makedefs = { "SUBPLATFORM", "HWDEF+=-D$(SUBPLATFORM)" }
            },
            {
                macro = "NUT_INIT_BOARD",
                brief = "Board Initialization",
                description = "If selected, a board specific initialization function "..
                      "is called before system initialization. At this time the C runtime "..
                      "is basically available and all global and static variables "..
                      "are set. However, kernel services like memory management, "..
                      "timers, threads etc. are not available in this stage.\n\n"..
                      "A related source file must be provided in the "..
                      "arch/MCU/board/ directory, named after the PLATFORM macro.",
                requires = { "HW_TARGET_BOARD" },
                provides = { "HW_BOARD_SUPPORT" },
                flavor = "boolean",
                file = "include/cfg/arch.h",
            },
            {
                macro = "NUT_INIT_IDLE",
                brief = "Idle Initialization",
                description = "If selected, a board specific initialization function "..
                      "is called at the beginning of the idle thread. At this time "..
                      "heap memory management is available and even new threads may be "..
                      "created. However, context switching will not work and timer "..
                      "services (including time outs) are not available.\n\n"..
                      "A related source file must be provided in the "..
                      "arch/MCU/board/ directory, named after the PLATFORM macro.",
                requires = { "HW_TARGET_BOARD" },
                provides = { "HW_BOARD_SUPPORT" },
                flavor = "boolean",
                file = "include/cfg/arch.h",
            },
            {
                macro = "NUT_INIT_MAIN",
                brief = "Main Initialization",
                description = "If selected, a board specific initialization function "..
                      "is called immediately before calling the application's main function. "..
                      "At this time the system is fully functional.\n\n"..
                      "A related source file must be provided in the "..
                      "arch/MCU/board/ directory, named after the PLATFORM macro.",
                requires = { "HW_TARGET_BOARD" },
                provides = { "HW_BOARD_SUPPORT" },
                flavor = "boolean",
                file = "include/cfg/arch.h",
            }
        }
    },
    {
        name = "nuttools",
        brief = "Tools",
        description = "Tool selection.",
        script = "tools.nut"
    },
    {
        name = "nutarch",
        brief = "Architecture",
        description = "Target selection.",
        subdir = "arch",
        script = "arch/arch.nut"
    },
    {
        name = "nutos",
        brief = "RTOS Kernel",
        description = "Operating system core functions",
        requires = { "HW_TARGET" },
        subdir = "os",
        script = "os/os.nut"
    },
    {
        name = "nutdev",
        brief = "Device Drivers",
        description = "This library contains architecture independant...\n"..
                      "... hardware device drivers, typically supporting external chips.\n"..
                      "... device driver frameworks, which provide the hardware independant part of a driver\n"..
                      "... helper routines, which are of general use for device drivers.",
        requires = { "HW_TARGET" },
        subdir = "dev",
        script = "dev/dev.nut"
    },
    {
        name = "nutc",
        brief = "C runtime (tool specific)",
        description = "Hardware independent C runtime",
        requires = { "HW_TARGET" },
        subdir = "c",
        script = "c/c.nut"
    },
    {
        name = "nutcrt",
        brief = "C runtime (target specific)",
        description = "Hardware dependent C runtime",
        requires = { "HW_TARGET" },
        subdir = "crt",
        script = "crt/crt.nut"
    },
    {
        name = "nutgorp",
        brief = "Gorp: Code snipped library",
        description = "Additional libraries and code snippets used all around in NutOS and application code",
        requires = { "HW_TARGET" },
        subdir = "gorp",
        script = "gorp/gorp.nut"
    },
    {
        name = "nutnet",
        brief = "Network (general)",
        description = "Network functions",
        requires = { "HW_TARGET" },
        subdir = "net",
        script = "net/net.nut"
    },
    {
        name = "nutpro",
        brief = "Network (application layer)",
        description = "High level network protocols",
        requires = { "HW_TARGET" },
        subdir = "pro",
        script = "pro/pro.nut"
    },
    {
        name = "nutfs",
        brief = "File system",
        description = "File systems",
        requires = { "HW_TARGET" },
        subdir = "fs",
        script = "fs/fs.nut"
    },
    {
        name = "nutcpp",
        brief = "C++ runtime extensions",
        description = "C++ runtime extensions",
        requires = { "HW_TARGET", "TOOL_CXX" },
        subdir = "cpp",
        script = "cpp/cpp.nut"
    },
    {
        name = "nutlua",
        brief = "Lua Support",
        description = "Lua is a powerful, light-weight scripting language. "..
                      "It offers good support for object-oriented, "..
                      "functional as well as data-driven programming.\n\n"..
                      "Currently available with the GNU compiler only.",
        requires = { "HW_TARGET" },
        subdir = "lua",
        script = "lua/lua.nut"
    },
    {
        name = "nutcontrib",
        brief = "Non-BSDL Code",
        description = "Packages in this module are not released under BSD license.\n"..
                      "More restrictive licenses are acceptable here, but extra effort"..
                      "is demanded of anyone using the code for commercial purposes.",
        requires = { "HW_TARGET" },
        subdir = "contrib",
        script = "contrib/contrib.nut"
    },
    CheckCustomConfig()
}

--
-- Read first 8 kBytes from C source file.
--
function GetSourceFileHead(source_path)
    local fp, buf

    -- Retrieve the repository path by calling a C function provided
    -- by the Configurator and read the first 8k of the file.
    fp = io.open(c_repo_path() .. "/../" .. source_path, "r")
    if fp ~= nil then
        buf = fp:read(8192)
        fp:close()
    end
    return buf
end

--
-- Read OS Version from C source file.
--
-- First version of a dynamic item:
-- The string contains a Lua script, which is compiled and executed by the Configurator.
-- This version is deprecated.
--        brief = "--\n".. -- Strings starting with this sequence are executed.
--                "return 'Nut/OS ' .. GetNutOsVersion()\n", -- This is the executed script.

-- Second version of a dynamic item:
-- A function result is combined with a static string. Note, that the function
-- is executed when this script file is loaded and must have been defined
-- previously.
--        brief = "Nut/OS " .. GetNutOsVersion(),

-- Third version of a dynamic item:
-- The value is specified as a function returning a string. In this case any function
-- used in our function body may be defined later.
--
function GetNutOsVersion()
    local buf, p1, p2, vers, subvers

    -- Try to read the version from a string in os/version.c.
    -- This worked until 4.9.7.
    buf = GetSourceFileHead("os/version.c")
    if buf ~= nil then
        p1, p2, vers = string.find(buf, "os_version_string.+\"(.+)\"")
    end

    -- If this doesn't work, we may have 4.9.8 or later.
    -- We will find the hex coded version in include/sys/version.h.
    if vers == nil then
      buf = GetSourceFileHead("include/sys/version.h")
      p1, p2, vers = string.find(buf, "NUT_VERSION_MAJOR%s+(%d+)")
      if vers == nil then
        vers = "?"
      end
      vers = vers .. "."
      p1, p2, subvers = string.find(buf, "NUT_VERSION_MINOR%s+(%d+)")
      if subvers == nil then
        subvers = "?"
      end
      vers = vers .. subvers .. "."
      p1, p2, subvers = string.find(buf, "NUT_VERSION_RELEASE%s+(%d+)")
      if subvers == nil then
        subvers = "?"
      end
      vers = vers .. subvers .. "."
      p1, p2, subvers = string.find(buf, "NUT_VERSION_BUILD%s+(%d+)")
      if subvers == nil then
        subvers = "?"
      end
      vers = vers .. subvers
    end

    return vers or "Unknown"
end

--
-- Retrieve platform specific GPIO banks.
--
function GetGpioBanks()
    if c_is_provided("HW_MCU_AT91") then
        if c_is_provided("HW_MCU_AT91R40008") then
            return { " " }
        else
            return { " ", "NUTGPIO_PORTA", "NUTGPIO_PORTB", "NUTGPIO_PORTC" }
        end
    end
    if c_is_provided("HW_MCU_AVR32") then
        return GetAvr32PioIds()
    end
    if c_is_provided("HW_MCU_STM32") then
        return GetStm32PioIds()
    end;
    return gpio_port_choice
end

--
-- Retrieve platform specific GPIO port IDs.
--
function GetGpioPortIds()
    if c_is_provided("HW_MCU_AT91") then
        return at91_pio_id_choice
    end
    if c_is_provided("HW_MCU_AVR32") then
        return GetAvr32PioIds()
    end
    if c_is_provided("HW_MCU_STM32") then
        return GetStm32PioIds()
    end;
    if c_is_provided("HW_MCU_COLDFIRE") then
        return GetColdfirePioIds()
    end;
    return { " " }
end

--
-- Retrieve AT91 PIO IDs.
--
function GetAt91PioIds()
    if c_is_provided("HW_MCU_AT91R40008") then
        return { " ", "PIO_ID" }
    end
    return { " ", "PIOA_ID", "PIOB_ID", "PIOC_ID" }
end

--
-- Retrieve AVR32 PIO IDs.
--

function GetAvr32PioIds()
    return { " ", "PIOA_ID", "PIOB_ID", "PIOC_ID", "PIOD_ID" }
end

--
-- Retrieve STM32 PIO struct pointers.
-- These IDs represet an struct pointer value of the port.
--
function GetStm32Pio()
    return { " ", "GPIOA", "GPIOB", "GPIOC", "GPIOD", "GPIOE", "GPIOF", "GPIOG" }
end

--
-- Retrieve STM32 PIO IDs.
-- These IDs represet an struct pointer value of the port.
--
function GetStm32PioIds()
    return {
        " ",
        "NUTGPIO_PORTA",
        "NUTGPIO_PORTB",
        "NUTGPIO_PORTC",
        "NUTGPIO_PORTD",
        "NUTGPIO_PORTE",
        "NUTGPIO_PORTF",
        "NUTGPIO_PORTG",
    }
end

--
-- Retrieve STM32 PIO Base.
-- These IDs represet an uint32_t value of the base address of the port.
--
function GetStm32PioBase()
    return { " ", "GPIOA_BASE", "GPIOB_BASE", "GPIOC_BASE", "GPIOD_BASE", "GPIOE_BASE", "GPIOF_BASE", "GPIOG_BASE" }
end

--
-- Retrieve COLDFIRE PIO IDs.
-- These IDs represet an struct pointer value of the port.
--
function GetColdfirePioIds()
    if c_is_provided("HW_MCU_MCF5225X") then
        return {
            " ",
            "PORTTE",
            "PORTTF",
            "PORTTG",
            "PORTTH",
            "PORTTI",
            "PORTTJ",
            "PORTNQ",
            "PORTAN",
            "PORTAS",
            "PORTQS",
            "PORTTA",
            "PORTTC",
            "PORTUA",
            "PORTUB",
            "PORTUC",
            "PORTDD",
        }
    end
    if c_is_provided("HW_MCU_MCF51CN") then
        return {
            " ",
            "PORTA",
            "PORTB",
            "PORTC",
            "PORTD",
            "PORTE",
            "PORTF",
            "PORTG",
            "PORTH",
            "PORTJ",
        }
    end
    return { " " }
end

--
-- Retrieve IOExpander specific Ports bits.
--
function GetIoxpPorts()
    if c_is_provided("DEV_IOEXP") then
        return pca9555_port_choice
    end
    return { " " }
end

--
-- Retrieve IOExpander specific GPIO bits.
--
function GetIoxpBits()
    if c_is_provided("DEV_IOEXP") then
        return pca9555_pin_choice
    end
    return { " " }
end

--
-- Retrieve platform specific GPIO bits.
--
function GetGpioBits()
    if c_is_provided("HW_MCU_STM32") then
        return stm32_bit_choice
    end
    if c_is_provided("HW_MCU_COLDFIRE") then
        return mcf5_bit_choice
    end
    return mcu_32bit_choice
end


--
-- Retrieve platform specific GPIO header path.
--
function GetGpioHeaderPath()
    local basepath

    basepath = "include/cfg/arch/"
    if c_is_provided("HW_MCU_ARM") then
        return basepath .. "armpio.h"
    end
    if c_is_provided("HW_MCU_AVR32") then
        return basepath .. "avr32pio.h"
    end
    if c_is_provided("HW_MCU_STM32") then
        return basepath .. "stm32pio.h"
    end
    if c_is_provided("HW_MCU_COLDFIRE") then
        return basepath .. "mcf5pio.h"
    end
    return basepath .. "pio.h"
end


--
-- Retrieve platform specific GPIO Special Functions
--
function GetAlternativePinsets()
    return { "ALTERNATE_PIN_SET1", "ALTERNATE_PIN_SET2", "ALTERNATE_PIN_SET3" }
end
