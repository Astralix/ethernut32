/*
 * Copyright (C) 2008 by egnite GmbH
 * Copyright (C) 2013 by Uwe Bonnes
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 *
 */

/*
 * $Id: piotran.h 5409 2013-10-17 11:59:53Z u_bonnes $
 */

#include <cfg/arch.h>

/*!
 * \addtogroup xgPiotranslate
 */
/*@{*/

/*!
 * \file include/cfg/arch/piotran.h
 * \brief Port translations.
 *
 * This header file determines the target specific GPIO register names
 * by a simple configured port identifier. In addition it provides
 * several macros to configure, set, clear or query GPIO bits. The
 * exposed functionality is only a subset of the functionality on most
 * architectures, but choosen to be available on most architectures!
 *
 * Unlike most other header files, this one may be included several
 * times within a single source file, typically once for each configured
 * identifier. This header is not supposed to be included in
 * architecture specific drivers, as it exposes only restricted
 * functionality.
 *
 * \code
 * #undef GPIO_ID
 * #define GPIO_ID MY_PORT1_ID
 * #include <cfg/arch/piotran.h>
 * static INLINE void MY_PORT1_INIT(int bit) { GPIO_INIT(bit); }
 * static INLINE void MY_PORT1_SET_HI(int bit) { GPIO_SET_HI(bit); }
 * static INLINE void MY_PORT1_SET_LO(int bit) { GPIO_SET_LO(bit); }
 *
 * #undef GPIO_ID
 * #define GPIO_ID MY_PORT2_ID
 * #include <cfg/arch/piotran.h>
 * static INLINE void MY_PORT2_INIT(void) { GPIO_INIT(MY_PIN2_ID); }
 * static INLINE void MY_PORT2_SET_HI(void) { GPIO_SET_HI(MY_PIN2_ID); }
 * static INLINE void MY_PORT2_SET_LO(void) { GPIO_SET_LO(MY_PIN2_ID); }
 * static INLINE int  MY_PORT2_GET(void) { return GPIO_GET(MY_PIN2_ID); }
 *
 * void Out1(int bit, int val)
 * {
 *   if (val)
 *     MY_PORT1_SET_HI(bit);
 *   else
 *     MY_PORT1_SET_LO(bit);
 * }
 *
 * void Toogle2(int bit)
 * {
 *   if (MY_PORT2_GET())
 *     MY_PORT2_SET_LO();
 *   else
 *     MY_PORT2_SET_HI();
 * }
 * \endcode
 *
 * In contrast to the routines in dev/gpio.h, these macros do not require
 * any function call and will therefore produce faster and smaller code,
 * with the exception of GPIO_INIT(). The following restricted macros are
 * available, allowing implementation for all NUTOS architectures:
 *
 * - GPIO_INIT Basic initialization to input, no pull-up, about 1 MHz speed
 *            and push-pull when switched to output
 * - GPIO_SET_LO Sets output low.
 * - GPIO_SET_HI Sets output high.
 * - GPIO_GET Returns input status.
 * - GPIO_OUTPUT Configures as output.
 * - GPIO_INPUT Configures an input.
 * - GPIO_PULLUP_ON Enables input pull-up resistor.
 * - GPIO_PULLUP_OFF Disables input pull-up resistor.
 *
 * \note For open-drain pin to release, use GPIO_INPUT(); GPIO_SET_HI ,
 *       for open-drain pin to drive low, use GPIO_SET_LO(); GPIO_OUTPUT()
 *
 * \note All listed macros will be available for any port identifier on
 *       any target, even if the related function is not available. You
 *       should check the target's datasheet.
 *
 * \note We use capital letters for the inline attribute to refer to a
 *       preprocessor macro. If the compiler doesn't support inlined
 *       function, then the macro will be empty. In this case a function
 *       call may be used, depending on the compiler's optimization
 *       strategy. Even if the compiler supports the inline keyword,
 *       it may decide to generate a callable function.
 */

/*
 * Remove any previously defined port macros.
 */

#undef GPIO_INIT
#undef GPIO_SET_LO
#undef GPIO_SET_HI
#undef GPIO_GET
#undef GPIO_OUTPUT
#undef GPIO_INPUT
#undef GPIO_PULLUP_ON
#undef GPIO_PULLUP_OFF

#if defined(MCU_STM32)
#if defined(MCU_STM32L1)
#define GPIO_SPEED GPIO_CFG_SPEED_MED
#else
#define GPIO_SPEED GPIO_CFG_SPEED_SLOW
#endif
#define GPIO_INIT(b)   GpioPinConfigSet(GPIO_ID, b, GPIO_CFG_INPUT|GPIO_SPEED)
#define GPIO_SET_LO(b) GpioPinSetLow(GPIO_ID, b)
#define GPIO_SET_HI(b) GpioPinSetHigh(GPIO_ID, b)
#define GPIO_GET(b)    GpioPinGet(GPIO_ID, b)
#define GPIO_OUTPUT(b) GpioPinDrive(GPIO_ID, b)
#define GPIO_INPUT(b)  GpioPinRelease(GPIO_ID, b);
#if defined(MCU_STM32F1)
/* For F1, Pullup needs to set configuration _and_ output.
 * We can't handle this in a stateless way.
 */
#define GPIO_PULLUP_ON(b)
#define GPIO_PULLUP_OFF(b)
#elif defined(MCU_STM32F3)
#define GPIO_PULLUP_ON(b)  (CM3REG(GPIO_ID, GPIO_TypeDef, PUPDR) |=  (_BV(b<<1)))
#define GPIO_PULLUP_OFF(b) (CM3REG(GPIO_ID, GPIO_TypeDef, PUPDR) &= ~(_BV(b<<1)))
#else
#define GPIO_PULLUP_ON(b)  (CM3BBREG(GPIO_ID, GPIO_TypeDef, PUPDR, (b<<1)) = 1)
#define GPIO_PULLUP_OFF(b) (CM3BBREG(GPIO_ID, GPIO_TypeDef, PUPDR, (b<<1)) = 0)
#endif

#else
#undef  GPIO_PE_REG
#undef  GPIO_PD_REG
#undef  GPIO_PS_REG

#undef  GPIO_OE_REG
#undef  GPIO_OD_REG
#undef  GPIO_OS_REG

#undef  GPIO_SOD_REG
#undef  GPIO_COD_REG
#undef  GPIO_ODS_REG

#undef  GPIO_PDS_REG

#undef  GPIO_PUE_REG
#undef  GPIO_PUD_REG
#undef  GPIO_PUS_REG

#undef  GPIO_MDE_REG
#undef  GPIO_MDD_REG
#undef  GPIO_MDS_REG

#undef  GPIO_IFE_REG
#undef  GPIO_IFD_REG
#undef  GPIO_IFS_REG


#if defined(MCU_AT91)
/*
 * Determine AT91 port names.
 */
#include <arch/arm/at91.h>

#if GPIO_ID == PIOA_ID
#define  GPIO_PE_REG    PIOA_PER
#define  GPIO_PD_REG    PIOA_PDR
#define  GPIO_PS_REG    PIOA_PSR
#define  GPIO_OE_REG    PIOA_OER
#define  GPIO_OD_REG    PIOA_ODR
#define  GPIO_OS_REG    PIOA_OSR
#define  GPIO_SOD_REG   PIOA_SODR
#define  GPIO_COD_REG   PIOA_CODR
#define  GPIO_ODS_REG   PIOA_ODSR
#define  GPIO_PDS_REG   PIOA_PDSR
#if defined(PIOA_PUER)
#define  GPIO_PUE_REG   PIOA_PUER
#if defined(PIOA_PUDR)
#define  GPIO_PUD_REG   PIOA_PUDR
#define  GPIO_PUS_REG   PIOA_PUSR
#endif /* PIOA_PUDR */
#endif /* PIOA_PUER */
#if defined(PIOA_MDER)
#define  GPIO_MDE_REG   PIOA_MDER
#if defined(PIOA_MDDR)
#define  GPIO_MDD_REG   PIOA_MDDR
#define  GPIO_MDS_REG   PIOA_MDSR
#endif /* PIOA_MDDR */
#endif /* PIOA_MDER */
#if defined(PIOA_IFER)
#define  GPIO_IFE_REG   PIOA_IFER
#if defined(PIOA_IFDR)
#define  GPIO_IFD_REG   PIOA_IFDR
#define  GPIO_IFS_REG   PIOA_IFSR
#endif /* PIOA_IFDR */
#endif /* PIOA_IFER */

#elif GPIO_ID == PIOB_ID
#define  GPIO_PE_REG    PIOB_PER
#define  GPIO_PD_REG    PIOB_PDR
#define  GPIO_PS_REG    PIOB_PSR
#define  GPIO_OE_REG    PIOB_OER
#define  GPIO_OD_REG    PIOB_ODR
#define  GPIO_OS_REG    PIOB_OSR
#define  GPIO_SOD_REG   PIOB_SODR
#define  GPIO_COD_REG   PIOB_CODR
#define  GPIO_ODS_REG   PIOB_ODSR
#define  GPIO_PDS_REG   PIOB_PDSR
#if defined(PIOB_PUER)
#define  GPIO_PUE_REG   PIOB_PUER
#if defined(PIOB_PUDR)
#define  GPIO_PUD_REG   PIOB_PUDR
#define  GPIO_PUS_REG   PIOB_PUSR
#endif /* PIOB_PUDR */
#endif /* PIOB_PUER */
#if defined(PIOB_MDER)
#define  GPIO_MDE_REG   PIOB_MDER
#if defined(PIOB_MDDR)
#define  GPIO_MDD_REG   PIOB_MDDR
#define  GPIO_MDS_REG   PIOB_MDSR
#endif /* PIOB_MDDR */
#endif /* PIOB_MDER */
#if defined(PIOB_IFER)
#define  GPIO_IFE_REG   PIOB_IFER
#if defined(PIOB_IFDR)
#define  GPIO_IFD_REG   PIOB_IFDR
#define  GPIO_IFS_REG   PIOB_IFSR
#endif /* PIOB_IFDR */
#endif /* PIOB_IFER */

#elif GPIO_ID == PIOC_ID
#define  GPIO_PE_REG    PIOC_PER
#define  GPIO_PD_REG    PIOC_PDR
#define  GPIO_PS_REG    PIOC_PSR
#define  GPIO_OE_REG    PIOC_OER
#define  GPIO_OD_REG    PIOC_ODR
#define  GPIO_OS_REG    PIOC_OSR
#define  GPIO_SOD_REG   PIOC_SODR
#define  GPIO_COD_REG   PIOC_CODR
#define  GPIO_ODS_REG   PIOC_ODSR
#define  GPIO_PDS_REG   PIOC_PDSR
#if defined(PIOC_PUER)
#define  GPIO_PUE_REG   PIOC_PUER
#if defined(PIOC_PUDR)
#define  GPIO_PUD_REG   PIOC_PUDR
#define  GPIO_PUS_REG   PIOC_PUSR
#endif /* PIOC_PUDR */
#endif /* PIOC_PUER */
#if defined(PIOC_MDER)
#define  GPIO_MDE_REG   PIOC_MDER
#if defined(PIOC_MDDR)
#define  GPIO_MDD_REG   PIOC_MDDR
#define  GPIO_MDS_REG   PIOC_MDSR
#endif /* PIOC_MDDR */
#endif /* PIOC_MDER */
#if defined(PIOC_IFER)
#define  GPIO_IFE_REG   PIOC_IFER
#if defined(PIOC_IFDR)
#define  GPIO_IFD_REG   PIOC_IFDR
#define  GPIO_IFS_REG   PIOC_IFSR
#endif /* PIOC_IFDR */
#endif /* PIOC_IFER */

#else /* GPIO_ID */
#define  GPIO_PE_REG    PIO_PER
#define  GPIO_PD_REG    PIO_PDR
#define  GPIO_PS_REG    PIO_PSR
#define  GPIO_OE_REG    PIO_OER
#define  GPIO_OD_REG    PIO_ODR
#define  GPIO_OS_REG    PIO_OSR
#define  GPIO_SOD_REG   PIO_SODR
#define  GPIO_COD_REG   PIO_CODR
#define  GPIO_ODS_REG   PIO_ODSR
#define  GPIO_PDS_REG   PIO_PDSR
#if defined(PIO_PUER)
#define  GPIO_PUE_REG   PIO_PUER
#if defined(PIO_PUDR)
#define  GPIO_PUD_REG   PIO_PUDR
#define  GPIO_PUS_REG   PIO_PUSR
#endif /* PIO_PUDR */
#endif /* PIO_PUER */
#if defined(PIO_MDER)
#define  GPIO_MDE_REG   PIO_MDER
#if defined(PIO_MDDR)
#define  GPIO_MDD_REG   PIO_MDDR
#define  GPIO_MDS_REG   PIO_MDSR
#endif /* PIO_MDDR */
#endif /* PIO_MDER */
#if defined(PIO_IFER)
#define  GPIO_IFE_REG   PIO_IFER
#if defined(PIO_IFDR)
#define  GPIO_IFD_REG   PIO_IFDR
#define  GPIO_IFS_REG   PIO_IFSR
#endif /* PIO_IFDR */
#endif /* PIO_IFER */

#endif /* GPIO_ID */

#elif defined(__AVR32__)

#include <avr32/io.h>

#define  GPIO_PE_REG    &AVR32_GPIO.port[GPIO_ID].gpers
#define  GPIO_PD_REG    &AVR32_GPIO.port[GPIO_ID].pderc
#define  GPIO_PS_REG    &AVR32_GPIO.port[GPIO_ID].pder
#define  GPIO_OE_REG    &AVR32_GPIO.port[GPIO_ID].oders
#define  GPIO_OD_REG    &AVR32_GPIO.port[GPIO_ID].oderc
#define  GPIO_OS_REG    &AVR32_GPIO.port[GPIO_ID].oder
#define  GPIO_SOD_REG   &AVR32_GPIO.port[GPIO_ID].ovrs
#define  GPIO_COD_REG   &AVR32_GPIO.port[GPIO_ID].ovrc
#define  GPIO_ODS_REG   &AVR32_GPIO.port[GPIO_ID].ovr
#define  GPIO_PDS_REG   &AVR32_GPIO.port[GPIO_ID].pvr
#define  GPIO_PUE_REG   &AVR32_GPIO.port[GPIO_ID].puers
#define  GPIO_PUD_REG   &AVR32_GPIO.port[GPIO_ID].puerc
#define  GPIO_PUS_REG   &AVR32_GPIO.port[GPIO_ID].puer
#define  GPIO_MDE_REG   &AVR32_GPIO.port[GPIO_ID].odmerc
#define  GPIO_MDD_REG   &AVR32_GPIO.port[GPIO_ID].odmers
#define  GPIO_MDS_REG   &AVR32_GPIO.port[GPIO_ID].odmer
#define  GPIO_IFE_REG   &AVR32_GPIO.port[GPIO_ID].gfers
#define  GPIO_IFD_REG   &AVR32_GPIO.port[GPIO_ID].gferc
#define  GPIO_IFS_REG   &AVR32_GPIO.port[GPIO_ID].gfer

/* Additional targets can be added here. */

#endif /* MCU */


#if defined(GPIO_COD_REG)
#define GPIO_SET_LO(b)      outr(GPIO_COD_REG, _BV(b))
#define GPIO_SET_HI(b)      outr(GPIO_SOD_REG, _BV(b))
#elif defined(GPIO_SOD_REG)
#define GPIO_SET_LO(b)      cbi(GPIO_SOD_REG, b)
#define GPIO_SET_HI(b)      sbi(GPIO_SOD_REG, b)
#else
#define GPIO_SET_LO(b)
#define GPIO_SET_HI(b)
#endif

#if defined(GPIO_PDS_REG)
#define GPIO_GET(b)         ((inr(GPIO_PDS_REG) & _BV(b)) == _BV(b))
#else
#define GPIO_GET(b)
#endif

#if defined(GPIO_OD_REG)
#define GPIO_OUTPUT(b)      outr(GPIO_OE_REG, _BV(b))
#elif defined(GPIO_OE_REG)
#define GPIO_OUTPUT(b)      sbi(GPIO_OE_REG, b)
#else
#define GPIO_OUTPUT(b)
#endif

#if defined(GPIO_OD_REG)
#define GPIO_INPUT(b)       outr(GPIO_OD_REG, _BV(b))
#elif defined(GPIO_OE_REG)
#define GPIO_INPUT(b)       cbi(GPIO_OE_REG, b)
#else
#define GPIO_INPUT(b)
#endif

#if defined(GPIO_PUD_REG)
#define GPIO_PULLUP_ON(b)   outr(GPIO_PUE_REG, _BV(b))
#elif defined(GPIO_PUE_REG)
#define GPIO_PULLUP_ON(b)   sbi(GPIO_PUE_REG, b)
#else
#define GPIO_PULLUP_ON(b)
#endif

#if defined(GPIO_PUD_REG)
#define GPIO_PULLUP_OFF(b)  outr(GPIO_PUD_REG, _BV(b))
#elif defined(GPIO_PUE_REG)
#define GPIO_PULLUP_OFF(b)  cbi(GPIO_PUE_REG, b)
#else
#define GPIO_PULLUP_OFF(b)
#endif

#endif

#if !defined(GPIO_INIT)
#warning "Define GPIO_INIT for your architecture"
#endif
/*@}*/
