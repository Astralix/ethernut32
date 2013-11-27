# Automatically generated on Thu Nov 28 00:17:48 2013
#
# Do not edit, modify UserConf.mk instead!
#

PLATFORM=STM32L152C_DISCOVERY
HWDEF+=-D$(PLATFORM)
LDNAME=stm32l15XxC_flash
LDSCRIPT=$(LDNAME).ld
LDPATH=$(top_srcdir)/arch/cm3/ldscripts
MCU=cortex-m3
MFIX=-mfix-cortex-m3-ldrd
ARCH_STM32=y
HWDEF+=-DSTM32L1 -DMCU_STM32L1
HWDEF+=-DSTM32L1XX_MD
CRUROM=crurom
LICENSE_ST_GUIDANCE=y
LICENSE_MCD_ST_LIBERTY=y


include $(top_appdir)/UserConf.mk
