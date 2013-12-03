#
# Copyright (C) 2008 by egnite GmbH
# Copyright (C) 2004-2006 by egnite Software GmbH
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holders nor the names of
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# For additional information see http://www.ethernut.de/
#

PROJ = libnutarch

top_srcdir = ..
top_blddir = ..

VPATH = $(top_srcdir)/arch

LIBDIR = $(top_blddir)/lib

include $(top_blddir)/NutConf.mk

SRCS =

#
# ARM7/9 Controllers
#

ifeq ($(ARCH), arm)

SRCS = arm/os/context.c
SRCSPATH = arm/dev/atmel

ifeq ($(DEVICE), AT91R40008)
SRCS +=	 $(SRCSPATH)/ostimer_at91.c $(SRCSPATH)/wdt_at91.c \
	 $(SRCSPATH)/ih_at91fiq.c $(SRCSPATH)/ih_at91irq0.c $(SRCSPATH)/ih_at91irq1.c $(SRCSPATH)/ih_at91irq2.c \
	 $(SRCSPATH)/ih_at91pio.c $(SRCSPATH)/ih_at91swirq.c $(SRCSPATH)/ih_at91tc0.c $(SRCSPATH)/ih_at91tc1.c \
	 $(SRCSPATH)/ih_at91tc2.c $(SRCSPATH)/ih_at91uart0.c $(SRCSPATH)/ih_at91uart1.c $(SRCSPATH)/ih_at91wdi.c \
	 $(SRCSPATH)/debug_at91.c $(SRCSPATH)/usart0at91.c $(SRCSPATH)/usart1at91.c $(SRCSPATH)/hd44780_at91.c \
	 $(SRCSPATH)/st7036_at91.c arm/dev/ax88796.c arm/dev/dm9000e.c $(SRCSPATH)/at91init.c \
	 $(SRCSPATH)/gpio_at91.c $(SRCSPATH)/gpioa_at91.c
endif

ifeq ($(DEVICE), AT91SAM7X256)
SRCS +=	 $(SRCSPATH)/ostimer_at91.c $(SRCSPATH)/ih_at91fiq.c \
	 $(SRCSPATH)/ih_at91sys.c $(SRCSPATH)/ih_at91irq0.c $(SRCSPATH)/ih_at91irq1.c $(SRCSPATH)/ih_at91irq2.c \
	 $(SRCSPATH)/ih_at91pio.c $(SRCSPATH)/ih_at91spi0.c $(SRCSPATH)/ih_at91spi1.c $(SRCSPATH)/ih_at91ssc.c \
	 $(SRCSPATH)/ih_at91swirq.c $(SRCSPATH)/ih_at91tc0.c $(SRCSPATH)/ih_at91tc1.c $(SRCSPATH)/ih_at91tc2.c \
	 $(SRCSPATH)/ih_at91twi.c $(SRCSPATH)/ih_at91adc.c $(SRCSPATH)/ih_at91uart0.c $(SRCSPATH)/ih_at91uart1.c \
	 $(SRCSPATH)/ih_at91emac.c $(SRCSPATH)/ih_at91wdi.c $(SRCSPATH)/debug_at91.c $(SRCSPATH)/usart0at91.c \
	 $(SRCSPATH)/usart1at91.c $(SRCSPATH)/at91_emac.c $(SRCSPATH)/spimmc_at91.c $(SRCSPATH)/at91init.c \
	 $(SRCSPATH)/gpio_at91.c $(SRCSPATH)/gpioa_at91.c $(SRCSPATH)/gpiob_at91.c $(SRCSPATH)/gpioc_at91.c \
	 $(SRCSPATH)/at91_efc.c $(SRCSPATH)/at91_twi.c $(SRCSPATH)/at91_adc.c $(SRCSPATH)/hd44780_at91.c \
	 $(SRCSPATH)/st7036_at91.c $(SRCSPATH)/usartDat91.c
endif

ifeq ($(DEVICE), AT91SAM7S256)
SRCS +=	 $(SRCSPATH)/ostimer_at91.c $(SRCSPATH)/ih_at91fiq.c \
	 $(SRCSPATH)/ih_at91sys.c $(SRCSPATH)/ih_at91irq0.c $(SRCSPATH)/ih_at91irq1.c $(SRCSPATH)/ih_at91irq2.c \
	 $(SRCSPATH)/ih_at91pio.c $(SRCSPATH)/ih_at91spi0.c $(SRCSPATH)/ih_at91ssc.c \
	 $(SRCSPATH)/ih_at91swirq.c $(SRCSPATH)/ih_at91tc0.c $(SRCSPATH)/ih_at91tc1.c $(SRCSPATH)/ih_at91tc2.c \
	 $(SRCSPATH)/ih_at91twi.c $(SRCSPATH)/ih_at91adc.c $(SRCSPATH)/ih_at91uart0.c $(SRCSPATH)/ih_at91uart1.c \
	 $(SRCSPATH)/ih_at91wdi.c $(SRCSPATH)/debug_at91.c $(SRCSPATH)/usart0at91.c $(SRCSPATH)/usart1at91.c \
	 $(SRCSPATH)/at91_emac.c $(SRCSPATH)/spimmc_at91.c $(SRCSPATH)/at91init.c \
	 $(SRCSPATH)/at91_efc.c $(SRCSPATH)/at91_twi.c $(SRCSPATH)/at91_adc.c $(SRCSPATH)/usartDat91.c
endif

ifeq ($(DEVICE), AT91SAM9260)
SRCS +=	 $(SRCSPATH)/ostimer_at91.c $(SRCSPATH)/ih_at91emac.c \
	 $(SRCSPATH)/ih_at91fiq.c $(SRCSPATH)/ih_at91irq0.c $(SRCSPATH)/ih_at91irq1.c $(SRCSPATH)/ih_at91irq2.c \
	 $(SRCSPATH)/ih_at91pio.c $(SRCSPATH)/ih_at91spi0.c $(SRCSPATH)/ih_at91spi1.c $(SRCSPATH)/ih_at91ssc.c \
	 $(SRCSPATH)/ih_at91swirq.c $(SRCSPATH)/ih_at91tc0.c $(SRCSPATH)/ih_at91tc1.c $(SRCSPATH)/ih_at91tc2.c \
	 $(SRCSPATH)/ih_at91uart0.c $(SRCSPATH)/ih_at91uart1.c $(SRCSPATH)/debug_at91.c $(SRCSPATH)/usart0at91.c \
	 $(SRCSPATH)/usart1at91.c $(SRCSPATH)/hd44780_at91.c $(SRCSPATH)/st7036_at91.c $(SRCSPATH)/at91_emac.c \
	 $(SRCSPATH)/spimmc_at91.c $(SRCSPATH)/at91_mci.c $(SRCSPATH)/at91init.c $(SRCSPATH)/at91_spi.c \
	 $(SRCSPATH)/gpio_at91.c $(SRCSPATH)/gpioa_at91.c $(SRCSPATH)/gpiob_at91.c $(SRCSPATH)/gpioc_at91.c \
	 $(SRCSPATH)/ih_at91pioa.c $(SRCSPATH)/ih_at91piob.c $(SRCSPATH)/ih_at91pioc.c $(SRCSPATH)/usartDat91.c
endif

OBJ1 = arm/init/crt$(LDNAME).o

endif

#
# CortexM Controllers
#

ifeq ($(ARCH), cm3)

SRCS = cm3/os/context.c cm3/cmsis/cortex_init.c
OBJ1 = cm3/cmsis/cortex_init.o

endif

#
# AV32 Controllers
#

ifeq ($(ARCH), avr32)
SRCS = avr32/os/context.c avr32/os/nutinit.c
endif

#
# Make Rules
#

OBJS = $(SRCS:.c=.o)

include $(top_srcdir)/Makedefs

all: $(PROJ).a $(OBJS) $(OBJ1)

install: $(PROJ).a $(OBJ1)
	$(CP) $(PROJ).a $(LIBDIR)/$(PROJ).a
	-$(CP) $(OBJ1) $(LIBDIR)/$(notdir $(OBJ1))

include $(top_srcdir)/Makerules

.PHONY: clean
clean: cleancc cleanedit
	@echo "  [CLEANING] $(PROJ)..."
	@-rm -f $(PROJ).a
	@-rm -f $(OBJ1)
