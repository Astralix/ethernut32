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

-- Operating system functions
--
--

nutos =
{
    --
    -- Version Information
    --
    {
        name = "nutos_version",
        brief = function() return "Version " .. GetNutOsVersion(); end,
        description = "The version info was read from os/version.c in the current source tree.",
        sources = { "version.c" },
        options =
        {
            {
                macro = "NUT_VERSION_EXT",
                brief = "Extended",
                description = "User provided extension to the hard coded version information.",
                requires = { "NOT_AVAILABLE" },
                file = "include/cfg/os.h"
            }
        }
    },

    --
    -- Initialization
    --
    {
        name = "nutos_init",
        brief = "System Initialization",
        description = "This module is automatically called after booting the system. "..
                      "It will initialize memory and timer hardware and start the "..
                      "Nut/OS idle thread, which in turn starts the application's "..
                      "main routine in a separate thread.",
        sources = { "nutinit.c" },
        targets = { "nutinit.o" },
        options =
        {
            {
                macro = "HEARTBEAT_IDLE_PORT",
                brief = "Idle Heartbeat Port",
                description = "If a valid Port/Pin combination is given, set that Pin high when  "..
                              "ethernut is running and low when idle",
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBanks() end,
                file = "include/cfg/os.h"
            },
            {
                macro = "HEARTBEAT_IDLE_PIN",
                brief = "Idle Heartbeat PIN",
                description = "If a valid Port/Pin combination is given, set that Pin high when  "..
                              "ethernut is running and low when idle",
                flavor = "booldata",
                type = "enumerated",
                choices = function() return GetGpioBits() end,
                file = "include/cfg/os.h"
            },
            {
                macro = "HEARTBEAT_IDLE_INVERT",
                brief = "Idle Heartbeat Polarity",
                description = "Define if the LED at the HEARTBEAT Port/Pin combination is light with a Low Level",
                flavor = "booldata",
                type = "enumerated",
                file = "include/cfg/os.h"
            }
        }
    },

    --
    -- Memory management
    --
    {
        name = "nutos_heap",
        brief = "Memory management",
        provides = { "NUT_HEAPMEM" },
        sources = { "heap.c" },
        options =
        {
            {
                macro = "NUTMEM_GUARD",
                brief = "Guard Bytes",
                description = "If enabled, guard bytes will be added to both ends when "..
                              "new memory is allocated. When memory is released later, "..
                              "these guard bytes will be checked in order to detect "..
                              "memory overflows.",
                flavor = "boolean",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTDEBUG_HEAP",
                brief = "Heap Debugging",
                description = "If enabled, additional code will be added to the heap "..
                              "management to track memory allocation. This helps to "..
                              "detect memory leaks. Furthermore, problems are reported "..
                              "by calling NUTPANIC with a source code reference.",
                flavor = "boolean",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTMEM_SIZE",
                brief = "Memory Size",
                description = "Number of bytes available in fast data memory. On "..
                              "most platforms this value specifies the total number "..
                              "of bytes available in RAM.\n\n"..
                              "On Harvard architectures this value specifies the size "..
                              "of the data memory. It will be occupied by global "..
                              "variables and static data. Any remaining space will "..
                              "be added to the Nut/OS heap during system initialization.\n"..
                              "When running on an AVR MCU, set this to size of the "..
                              "on-chip SRAM, e.g. 4096 for the ATmega128 and 8192 for the ATmega2561.\n\n"..
                              "For CortexM3 architecture the size is given by the linker script. "..
                              "so only enter a value in here if you really know what you do!",
                default = "4096",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTMEM_START",
                brief = "Memory Start",
                description = "First address of fast data memory.\n\n"..
                              "For CortexM3 architecture the address is given by the linker script. "..
                              "so only enter a value in here if you really know what you do!",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTMEM_RESERVED",
                brief = "Reserved Memory Size",
                description = "Number of bytes reserved for special purposes.\n"..
                              "Right now this is used with the AVR platform only. "..
                              "The specified number of bytes may be used by a "..
                              "device driver when the external memory interface "..
                              " is disabled.",
                flavor = "booldata",
                file = "include/cfg/memory.h",
                makedefs = { "NUTMEM_RESERVED" }
            },
            {
                macro = "NUTXMEM_SIZE",
                brief = "Extended Memory Size",
                description = "Number of bytes available in external data memory.\n\n"..
                              "The result of enabling this item is platform specific."..
                              "With AVR systems it will enable the external memory "..
                              "interface of the CPU, even if the value is set to zero.",
                provides = { "NUTXDATAMEM_SIZE" },
                flavor = "booldata",
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTXMEM_START",
                brief = "Extended Memory Start",
                description = "First address of external data memory.",
                requires = { "NUTXDATAMEM_SIZE" },
                file = "include/cfg/memory.h"
            },
            {
                macro = "NUTMEM_SPLIT_FAST",
                brief = "Split Memory",
                description = "If enabled and if two memory regions are available on the "..
                              "target board, then each region is managed separately.",
                flavor = "boolean",
                file = "include/cfg/memory.h"
            },
        }
    },
    {
        name = "nutos_bankmem",
        brief = "Segmented Buffer",
        description = "The segmented buffer API can either handle a continuos "..
                      "memory space automatically allocated from heap or it can "..
                      "use banked memory hardware.",
        -- requires = { "HW_MCU_AVR" },
        provides = { "NUT_SEGBUF" },
        sources = { "bankmem.c" },
    },

    --
    -- Thread management
    --
    {
        name = "nutos_thread",
        brief = "Multithreading",
        requires = { "NUT_CONTEXT_SWITCH" },
        provides = { "NUT_THREAD" },
        sources = { "thread.c" },
        options =
        {
            {
                macro = "NUT_THREAD_IDLESTACK",
                brief = "Idle Thread Stack Size",
                description = "Number of bytes to be allocated for the stack of the idle thread.",
                flavor = "booldata",
                file = "include/cfg/os.h"
            },
            {
                macro = "NUT_THREAD_MAINSTACK",
                brief = "Main Thread Stack Size",
                description = "Number of bytes to be allocated for the stack of the main thread.",
                flavor = "booldata",
                file = "include/cfg/os.h"
            },
            {
                macro = "NUT_CRITICAL_NESTING",
                brief = "Critical Section Nesting",
                description = "The kernel avoids nesting of critical sections, but applications "..
                              "may want to use this feature. When enabled, a global counter keeps "..
                              "track of the nesting level. Disadvantages are increased code "..
                              "size and a significantly increased interrupt latency time.\n\n"..
                              "This option is currently available for ARM targets only. "..
                              "On the AVR, nesting of critical sections is available by default.",
                flavor = "boolean",
                requires = { "HW_MCU_ARM" },
                provides = { "NUT_CRITNESTING" },
                file = "include/cfg/os.h"
            },
            {
                macro = "NUT_CRITICAL_NESTING_STACK",
                brief = "Critical Section Nesting Uses Stack",
                description = "Using the stack for critical section nesting results in better "..
                              "interrupt responsiveness. However, most compilers are not able "..
                              "to deal with stack modifications. Use this option with great care.\n\n"..
                              "This option is ignored, if critical section nesting is disabled.",
                flavor = "boolean",
                requires = { "HW_MCU_ARM", "NUT_CRITNESTING" },
                file = "include/cfg/os.h"
            },
        }
    },

    --
    -- Timer management
    --
    {
        name = "nutos_timer",
        brief = "Timer management",
        requires = { "NUT_EVENT", "NUT_OSTIMER_DEV" },
        provides = { "NUT_TIMER" },
        sources = { "timer.c" },
        options =
        {
            {
                macro = "NUT_CPU_FREQ",
                brief = "Fixed MCU clock",
                description = "Frequency of the MCU clock. On some boards the system is able "..
                              "to automatically determine this value during initialization "..
                              "by using a reference clock. In this case the option may be disabled.",
                flavor = "booldata",
                file = "include/cfg/os.h"
            },
            {
                macro = "NUT_DELAYLOOPS",
                brief = "Loops per Millisecond",
                description = "Number of loops to execute per millisecond in NutMicroDelay.\n\n"..
                              "This value depends not only on the target hardware, but also on "..
                              "compiler options. If not specified, the system will try to "..
                              "determine a rough approximation.",
                flavor = "booldata",
                file = "include/cfg/os.h"
            },
            {
                macro = "NUT_USE_OLD_TIME_API",
                brief = "Use old style time calculation",
                description = "The old way of calculating the current time was based on the system "..
                              "tick counter. Unfortunately this introduced bugs when this counter value "..
                              "overflowed. On most systems (with 1Khz system tick timer) this happened "..
                              "after 49,7 days. If the system used an RTC, this bug did not show up in "..
                              "most cases, as the time was then queried from the RTC.\n\n"..
                              "Another problem was, that the time could not be corrected with an accuracy "..
                              "of more than 1 second, as the offset to epoc was calculated as a value of "..
                              "second resolution.\n\n"..
                              "The new time API re-invented the way the time is handled and calculated. "..
                              "Internally the time is now stored as a timeval struct, which allowes a "..
                              "a resolution up to 1 µs. The time is now always updated by the system "..
                              "tick timer interrupt and is always hold as software clock. It is _not_ "..
                              "automatically synced with the RTC any more.\n Syncing with a RTC has to "..
                              "be done manually by the user! "..
                              "This way we gain a much higher accuracy, as long, as the software clock is "..
                              "accurate. In other words, the system time accurracy depends on the system crystal "..
                              "accuracy and time will not be updated, if the CPU is put to deep sleep mode. "..
                              "Further more we assume a system tick timer running at a multiple of a full µs."..
                              "Uneven system tick timer frequencies would result in an increasing error of "..
                              "the software clock.\n\n"..
                              "Enable this option, if you would like to stay with the old API. This might "..
                              "be usefull if you design a low-power system, where the CPU is sleeping most "..
                              "of the time and shall not be woken up regularly, of if you use an add crystal "..
                              "frequency and can not provide a system tick timer running at an even multiple "..
                              "of 1µs",
                flavor = "boolean",
                file = "include/cfg/os.h"
            },
        }
    },

    --
    -- Event management
    --
    {
        name = "nutos_event",
        brief = "Event handling",
        description = "Events provide the core thread synchronization.",
        provides = { "NUT_EVENT" },
        sources = { "event.c" }
    },


    {
        name = "nutos_devreg",
        brief = "Device registration",
        description = "Applications need to register devices before using "..
                      "them. This serves two purposes. First it will create "..
                      "a link to the device driver code. Second it will "..
                      "initialize the device hardware.",
        provides = { "NUT_DEVREG" },
        sources = { "devreg.c" }
    },
    {
        name = "nutos_confos",
        brief = "Configuration",
        description = "Initial system settings are stored in non volatile memory."..
                      "The current version uses 3 bytes for validity check, 15 bytes "..
                      "for the host name and one last byte, which is always zero.\n\n"..
                      "The length of the host name is configurable.",
        provides = { "NUT_OSCONFIG" },
        sources = { "confos.c" },
        options =
        {
            {
                macro = "CONFOS_EE_OFFSET",
                brief = "Location",
                description = "This is the non-volatile memory address offset, where Nut/OS "..
                              "expects its configuration.\n\n"..
                              "Note, that changing this value will invalidate previously "..
                              "stored setting after upgrading to this new version. You must "..
                              "also make sure, that this memory area will not conflict with "..
                              "others, specifically the network configuration.\n\n"..
                              "This item is disabled if the system doesn't offer any "..
                              "non-volatile memory. Check the non-volatile memory"..
                              "module in the device driver section.",
                requires = { "DEV_NVMEM" },
                default = "0",
                type = "integer",
                file = "include/cfg/eeprom.h"
            },
            {
                macro = "CONFOS_EE_MAGIC",
                brief = "Magic Cookie",
                description = "Together with the length of the configuration structure "..
                              "this is used to determine that we got a valid configuration "..
                              "structure in non-volatile memory.\n",
                default = "\"OS\"",
                file = "include/cfg/eeprom.h"
            },
            {
                macro = "MAX_HOSTNAME_LEN",
                brief = "Max. Host Name Length",
                description = "The name of the local host can't grow beyond this size. "..
                              "This is just the basic name without domain information.\n\n"..
                              "Make sure that the virgin host name will fit. Further, "..
                              "keep in mind that changing this size will have an impact "..
                              "at least on the auto discovery feature.",
                default = "15",
                type = "integer",
                file = "include/cfg/eeprom.h"
            },
            {
                macro = "CONFOS_VIRGIN_HOSTNAME",
                brief = "Virgin Host Name",
                description = "This name will be used for hosts without valid configuration.\n",
                default = "\"ethernut\"",
                file = "include/cfg/eeprom.h"
            }
        }
    },

    --
    -- Additional functions, not required by the kernel.
    --
    {
        name = "nutos_semaphore",
        brief = "Semaphores",
        description = "Semaphores are not required by the kernel, but "..
                      "may be useful for applications.",
        requires = { "NUT_EVENT" },
        provides = { "NUT_SEMAPHORE" },
        sources = { "semaphore.c" }
    },
    {
        name = "nutos_mutex",
        brief = "Mutual exclusion semaphores",
        description = "Mutex semaphores are not required by the kernel, but "..
                      "may be useful for applications.",
        requires = { "NUT_EVENT" },
        provides = { "NUT_MUTEX" },
        sources = { "mutex.c" }
    },
    {
        name = "nutos_msg",
        brief = "Message queues",
        description = "Message queues are not required by the kernel, but "..
                      "may be useful for applications.",
        requires = { "NUT_EVENT" },
        provides = { "NUT_MQUEUE" },
        sources = { "msg.c" }
    },
    {
        name = "nutos_condition",
        brief = "Condition variables",
        description = "Condition variabled are not required by the kernel, but "..
                      "may be useful for applications.",
        requires = { "NUT_EVENT", "NUT_MUTEX"},
        provides = { "NUT_CONDITION_VARIABLES" },
        sources = { "condition.c" }
    },


    --
    -- Debugging
    --
    {
        name = "nutos_osdebug",
        brief = "OS Debug",
        requires = { "NUT_EVENT", "CRT_STREAM_WRITE" },
        provides = { "NUT_OSDEBUG" },
        sources = { "osdebug.c" },
        options =
        {
            {
                macro = "NUTDEBUG",
                brief = "OS Debug",
                description = "Used for kernel debugging.\n"..
                              "Enabling this functions will add a lot of "..
                              "extra code and require more RAM. In addition "..
                              "the application must register an output device "..
                              "early and redirect stdout to it.",
                flavor = "boolean",
                file = "include/cfg/os.h"
            },
            {
                macro = "NUTDEBUG_CHECK_STACK",
                brief = "Thread Stack Checking",
                description = "Used for thread stack checking.\n"..
                              "Enabling this will fill any threads stack area "..
                              "with a pattern at the point of creating the thread.\n"..
                              "By using NutThreadStackAvailable() one can now check "..
                              "for the maximum ammount of stack ever used by this "..
                              "thread.\n\n"..
                              "This functionality can be enabled without OS Debug to "..
                              "find out maximum stack usage in your final appliation.",
                flavor = "boolean",
                file = "include/cfg/os.h"
            },
            {
                macro = "NUTDEBUG_RAM",
                brief = "Placement of code",
                description = "Code is placed in ram",
                 flavor = "boolean",
                 file = "include/cfg/os.h"
            },
            {
                macro = "NUTDEBUG_LAZY",
                brief = "Initialize Stack when debugging",
                description = "When loading new compiled code, stack will be initialized with value read "..
                              "at 0x00000000 at reset. Either explicit load the value on each load "..
                              "or use this hack ...\n",
                flavor = "boolean",
                file = "include/cfg/os.h"
            }
        }
    },
    {
        name = "nutos_ostracer",
        brief = "OS Tracer",
        requires = { "HW_MCU_AVR", "TOOL_GCC" },
        provides = { "NUT_OSTRACER" },
        sources = { "tracer.c" },
        options =
        {
            {
                macro = "NUTTRACER",
                brief = "OS Tracer",
                description = "Used for kernel debugging.\n",
                flavor = "boolean",
                file = "include/cfg/os.h"
            },
            {
                macro = "NUTTRACER_CRITICAL",
                brief = "OS Critical Tracer",
                description = "Used for kernel debugging.\n",
                flavor = "boolean",
                file = "include/cfg/os.h"
            }
        }
    },
    {
        name = "nutos_fatal",
        brief = "Fatal Error Handler",
        description = "This default handler may be overridden by "..
                      "NUTFATAL and NUTPANIC routines provided by the application.",
        sources = { "fatal.c", "panic.c" }
    }
}

