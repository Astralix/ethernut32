/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * (C) 2011, 2012 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de
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
 */

/*
 * \verbatim
 * $Id: stm32_gpio.c 3182 2010-10-17 21:46:04Z Astralix $
 * \endverbatim
 */

#include <cfg/arch.h>
#include <arch/cm3.h>

#include <arch/cm3/timer.h>
#include <arch/cm3/stm/stm32_clk.h>
#include <cfg/clock.h>

#include <arch/cm3/stm/vendor/stm32l1xx.h>


/* Prepare some defaults if configuration is incomplete */
#if !defined(SYSCLK_SOURCE)
#define SYSCLK_SOURCE SYSCLK_MSI
#endif

/* Prepare MSI value as it is reset default.
 * Value has offset +1 for configuration reasons.
 * Default is 5, so we define 6 here.
 */
#if !defined(MSI_VALUE)
#define MSI_VALUE 6
#endif

static uint32_t SystemCoreClock = 0;

const uint32_t MSIFreqTable[8] = {65536, 131072, 262144,  524288, 1048000, 2097000, 4194000, 0};
static const uint8_t APBPrescTable[8]  = {1, 1, 1, 1, 2, 4, 8, 16};
static const uint8_t PLLDivTable[] = { 1, 2, 3, 4 };
static const uint8_t PLLMulTable[] = { 3, 4, 6, 8, 12, 16, 24, 32, 48 };

/*----------------  Clock Setup Procedure ------------------------------
 *
 * Clock system ist arranged like this:
 *
 *                            ,--------------------------- USB
 *                            |           ,--------------- CPU
 *                            |           +--------------- SDIO
 * 1-32MHz HSE ----+---PLLMUL-+-AHBPRES---+-- APB1PRESC--- APB1
 *                 |          |           +-- ABP2PRESC--- ABP2
 * 16MHz HSI ------+----------+-------------- ADCPRESC---- ADC
 *                            |
 *       MSI -----------------'
 *
 *
 *        ***** Setup of system clock configuration *****
 *
 * 1) Select system clock sources
 *
 * To setup system to use HSI call: SetSysClockSource( SYSCLK_HSI);
 * To setup system to use HSE call: SetSysClockSource( SYSCLK_HSE);
 *
 * To setup system to use the PLL output, first setup the PLL source:
 * SetPllClockSource( PLLCLK_HSI);
 * or
 * SetPllClockSource( PLLCLK_HSE);
 * Then call SetSysClockSource( SYSCLK_PLL);
 *
 * 2) Configure prescalers
 * After selecting the right clock sources, the prescalers need to
 * be configured:
 * Call SetSysClock(); to do this automatically.
 *
 */


/*!
 * \brief  Update SystemCoreClock according to Clock Register Values
 *
 * This function reads out the CPUs clock and PLL registers and assembles
 * the actual clock speed values into the SystemCoreClock local variable.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t pllmul = 0, plldiv = 0, msirange;
    uint32_t rcc;

    rcc = RCC->CFGR;

    /* Get SYSCLK source -------------------------------------------------------*/
    switch (rcc & RCC_CFGR_SWS)
    {
        case RCC_CFGR_SWS_MSI:  /* MSI used as system clock , value depends on RCC_ICSCR/MSIRANGE[2:0]: */
            msirange = RCC->ICSCR & RCC_ICSCR_MSIRANGE ;
            msirange = msirange >> _BI16(RCC_ICSCR_MSIRANGE_1);
            SystemCoreClock = MSIFreqTable[msirange];
            break;
        case RCC_CFGR_SWS_HSI:  /* HSI used as system clock */
            SystemCoreClock = HSI_VALUE;
            break;
        case RCC_CFGR_SWS_HSE:  /* HSE used as system clock */
            SystemCoreClock = HSE_VALUE;
            break;
        case RCC_CFGR_SWS_PLL:
#if 1
        	pllmul = PLLMulTable[(rcc & RCC_CFGR_PLLMUL) >> 18];
#else
            /* Assume that values not allowed don't occur*/
            if      (rcc & RCC_CFGR_PLLMUL4)  pllmul =  4;
            else if (rcc & RCC_CFGR_PLLMUL6)  pllmul =  6;
            else if (rcc & RCC_CFGR_PLLMUL8)  pllmul =  8;
            else if (rcc & RCC_CFGR_PLLMUL12) pllmul = 12;
            else if (rcc & RCC_CFGR_PLLMUL16) pllmul = 16;
            else if (rcc & RCC_CFGR_PLLMUL24) pllmul = 24;
            else if (rcc & RCC_CFGR_PLLMUL32) pllmul = 32;
            else if (rcc & RCC_CFGR_PLLMUL48) pllmul = 48;
            else                              pllmul =  3;
#endif
#if 1
            plldiv = PLLDivTable[(rcc & RCC_CFGR_PLLDIV) >> 22];
#else
            if((rcc & RCC_CFGR_PLLDIV4) == RCC_CFGR_PLLDIV4) plldiv = 4;
            else if (rcc & RCC_CFGR_PLLDIV3)                 plldiv = 3;
            else if (rcc & RCC_CFGR_PLLDIV2)                 plldiv = 2;
            else                                             plldiv = 1;
#endif
            if (rcc & RCC_CFGR_PLLSRC_HSE)
                SystemCoreClock = HSE_VALUE * pllmul / plldiv;
            else
                SystemCoreClock = HSI_VALUE * pllmul / plldiv;

    }

    /* Compute HCLK clock frequency ----------------*/
    if ((rcc & RCC_CFGR_HPRE_3))
        SystemCoreClock >>= ((rcc & (RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 |RCC_CFGR_HPRE_2)) +1);
}

/*!
 * \brief Re/Set RCC register bit and wait for same state of connected RDY bit or timeout
 *
 * \param  bbreg Bitband address of the bit to set
 * \param  tout timeout in delay units.
 * \return 0 on success, -1 on HSE start failed.
 */
int rcc_set_and_wait_rdy(__IO uint32_t *bbreg, int value, uint32_t tout)
{
    int state = (value)?1:0;
    *bbreg = state;
    do {
        tout--;
    } while ((bbreg[1] != state) && (tout > 0));
    return ( bbreg[1] == state)?0:-1;
}

/*!
 * \brief Control Msi clock.
 *
 * \param  val enable and select clock, 0 disable clock.
 * \return 0 on success, -1 on HSE start failed.
 */
int CtlMsiClock( uint8_t val)
{
    int rc;

    if( val == 0) {
		/* switch MSI off */
		rc = rcc_set_and_wait_rdy(
			CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_MSION)), 0, HSE_STARTUP_TIMEOUT);

    }
    else {
    	val--;
        /* select MSI range */
    	RCC->ICSCR &= ~(RCC_ICSCR_MSIRANGE);
    	RCC->ICSCR |= val << _BI16(RCC_ICSCR_MSIRANGE_1);
    	/* start MSI */
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_HSEON)), 0, HSE_STARTUP_TIMEOUT);
    }
    return rc;
}

/*!
 * \brief Control HSE clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on HSE start failed.
 */
int CtlHseClock( uint8_t ena)
{
    int rc;

    /* switch HSE off to allow to set HSE_BYPASS */
    rc = rcc_set_and_wait_rdy(
        CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_HSEON)), 0, HSE_STARTUP_TIMEOUT);

    if( ena) {
#if defined(HSE_BYPASS)
        CM3BBREG(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_HSEBYP)) = 1;
#else
        CM3BBREG(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_HSEBYP)) = 0;
#endif

#if !defined(RTCPRE) || (RTCPRE <0) || (RTCPRE >3)
#define RTCPRE 3
#endif
        RCC->CR &= ~RCC_CR_RTCPRE;
        RCC->CR |=  RTCPRE<< _BI32(RCC_CR_RTCPRE_0);

        /* Enable HSE */
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_HSEON)), 1, HSE_STARTUP_TIMEOUT);
    }
    return rc;
}

/* Functional same as F1 */
/*!
 * \brief Control HSI clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on HSI start failed.
 */
int CtlHsiClock( uint8_t ena)
{
    int rc = 0;

    if( ena) {
        /* Enable HSI */
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_HSION)), 1, HSE_STARTUP_TIMEOUT);
    }
    else {
        /* Disable HSE clock */
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_HSION)), 0, HSE_STARTUP_TIMEOUT);
    }

    return rc;
}

/* Functional same as F1 */
/*!
 * \brief Control PLL clock.
 *
 * \param  ena 0 disable clock, any other value enable it.
 * \return 0 on success, -1 on PLL start failed.
 */
int CtlPllClock( uint8_t ena)
{
    int rc = 0;

    if( ena) {
        /* Enable PLL */
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_PLLON)), 1, HSE_STARTUP_TIMEOUT);
    }
    else {
        /* Disable HSE clock */
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CR, _BI32(RCC_CR_PLLON)), 0, HSE_STARTUP_TIMEOUT);
    }

    return rc;
}


/*!
 * \brief  Configures the System clock source: HSE or HSI.
 * \note   This function should be used with PLL disables
 * \param  src is one of PLLCLK_HSE, PLLCLK_HSI.
 * \return 0 if clock is running ale -1.
 */
int SetPllClockSource( int src)
{
    int rc = -1;
    if (src == PLLCLK_HSE) {
        rc = CtlHseClock(ENABLE);
        if (rc==0) {
            CM3BBREG(RCC_BASE, RCC_TypeDef, CFGR, _BI32(RCC_CFGR_PLLSRC)) = 1;
        }
    }
    else if (src == PLLCLK_HSI) {
        rc = CtlHsiClock(ENABLE);
        /* Select HSI/2 as PLL clock source */
        if (rc==0) {
            CM3BBREG(RCC_BASE, RCC_TypeDef, CFGR, _BI32(RCC_CFGR_PLLSRC)) = 0;
        }
    }

    return rc;
}

/*!
 * \brief  Configures the System clock source: HSI, HS or PLL.
 * \note   This function should be used only after reset.
 * \param  src is one of SYSCLK_HSE, SYSCLK_HSI or SYSCLK_PLL.
 * \return 0 if selected clock is running else -1.
 */
int SetSysClockSource( int src)
{
    int rc = -1;

    /* Enable 64-bit access, Prefetch and 1 wait state */
    FLASH->ACR |= (FLASH_ACR_ACC64 | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY);

    /* Power enable */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Select the Voltage Range 1 (1.8 V) */
    PWR->CR = PWR_CR_VOS_0;

    /* Wait Until the Voltage Regulator is ready */
    while((PWR->CSR & PWR_CSR_VOSF) != RESET)
    {
    }

    /* Fixme: Set MSI source with MSI frequency parameter */
    if (src == SYSCLK_MSI) {
        rc = CtlMsiClock(MSI_VALUE);
        if (rc == 0) {
            /* Select MSI as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_MSI;

            /* Wait till MSI is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);
        }
    }
    if (src == SYSCLK_HSE) {
        rc = CtlHseClock(ENABLE);
        if (rc == 0) {
            /* Select HSE as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_HSE;

            /* Wait till HSE is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
        }
    }
    else if (src == SYSCLK_HSI) {
        rc = CtlHsiClock(ENABLE);
        if (rc == 0) {
            /* Select HSI as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_HSI;

            /* Wait till HSI is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
        }
    }
    else if (src == SYSCLK_PLL) {
        rc = CtlPllClock(ENABLE);
        if (rc == 0) {
            /* Select HSI as system clock source */
            RCC->CFGR &= ~RCC_CFGR_SW;
            RCC->CFGR |= RCC_CFGR_SW_PLL;

            /* Wait till HSI is used as system clock source */
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
        }
    }

    /* Update core clock information */
    SystemCoreClockUpdate();

    if (SystemCoreClock <= 16000000) {
    	/* Switch to 0 wait state */
        FLASH->ACR &= ~FLASH_ACR_LATENCY;
    }
    return rc;
}

#if (SYSCLK_SOURCE == SYSCLK_HSI) || (SYSCLK_SOURCE == SYSCLK_HSE)
/*!
 * \brief  Configures the System clock coming from HSE or HSI oscillator.
 *
 * Enable HSI/HSE clock and setup HCLK, PCLK2 and PCLK1 prescalers.
 *
 * \param  None.
 * \return 0 on success, -1 on fault of HSE.
 */
int SetSysClock(void)
{
    int rc = 0;
    register uint32_t cfgr;

    /* Fixme: Allow more flexible Flash Setting
     * For the moment, use 32-bit access with no prefetch . Latency has no meaning
     * for 32-bit access
     */
    cfgr = RCC->CFGR;

    cfgr &= ~(RCC_CFGR_HPRE|RCC_CFGR_PPRE1|RCC_CFGR_PPRE2);

    /* HCLK = SYSCLK */
    cfgr |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK */
    cfgr |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

    /* PCLK1 = HCLK */
    cfgr |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

    RCC->CFGR = cfgr;

    rc = SetSysClockSource(SYSCLK_SOURCE);

    return rc;
}

#elif (SYSCLK_SOURCE == SYSCLK_MSI)
int SetSysClock(void)
{
	int rc = 0;
	rc = CtlMsiClock(MSI_VALUE);
	//TODO: Configure Prescalers
	return rc;
}
#else
#if (PLLCLK_SOURCE==PLLCLK_HSE)
#define PLLCLK_IN (HSE_VALUE)
#else
#define PLLCLK_IN (HSI_VALUE)
#endif

/**
  * @brief  Sets System clock frequency to 8MHz and configure HCLK, PCLK2
  *          and PCLK1 prescalers.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
int SetSysClock(void)
{
    int rc = 0;
    /* FIXME*/
    return rc;
}
#endif /* (SYSCLK_SOURCE == SYSCLK_HSI) || (SYSCLK_SOURCE == SYSCLK_HSE) */

/**
  * @brief  Sets RTC clock to selected source.
  *
  * @param  source Clock source LSI/LSE/HSE
  * @retval -1 on error, 0 on success
  */
int SetRTCClock(int source)
{
    int rc = -1;
    /* Enable PWR Controller and access to the RTC backup domain*/
    CM3BBREG(RCC_BASE, RCC_TypeDef, APB1ENR, _BI32(RCC_APB1ENR_PWREN))=1;
    CM3BBREG(PWR_BASE, PWR_TypeDef, CR, _BI32(PWR_CR_DBP)) = 1;
    /* Reset RTC to allow selection */
    CM3BBREG(RCC_BASE, RCC_TypeDef, CSR, _BI32(RCC_CSR_RTCRST)) = 1;
    CM3BBREG(RCC_BASE, RCC_TypeDef, CSR, _BI32(RCC_CSR_RTCRST)) = 0;
    switch (source)
    {
    case RTCCLK_LSI:
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CSR, _BI32(RCC_CSR_LSION)),
            1, HSE_STARTUP_TIMEOUT*1000);
        if (rc == -1)
            return rc;
        RCC->CSR &= ~RCC_CSR_RTCSEL;
        RCC->CSR |= RCC_CSR_RTCSEL_LSI;
        break;
    case RTCCLK_LSE:
        /* LSE Bypass can only be written with LSE off*/
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CSR, _BI32(RCC_CSR_LSEON)),
            0, HSE_STARTUP_TIMEOUT*1000);
        if (rc == -1)
            return rc;
#if defined(LSE_BYPASS)
        CM3BBREG(RCC_BASE, RCC_TypeDef, CSR, _BI32(RCC_CSR_LSEBYP)) = 1;
#else
        CM3BBREG(RCC_BASE, RCC_TypeDef, CSR, _BI32(RCC_CSR_LSEBYP)) = 0;
#endif
        rc = rcc_set_and_wait_rdy(
            CM3BBADDR(RCC_BASE, RCC_TypeDef, CSR, _BI32(RCC_CSR_LSEON)),
            1, HSE_STARTUP_TIMEOUT*1000);
        if (rc == -1)
            return rc;
        RCC->CSR &= ~RCC_CSR_RTCSEL;
        RCC->CSR |= RCC_CSR_RTCSEL_LSE;
        break;
    case RTCCLK_HSE:
        RCC->CSR &= ~RCC_CSR_RTCSEL;
        RCC->CSR |= RCC_CSR_RTCSEL_HSE;
        break;
    }
    return rc;
}

/**
  * @brief  requests System clock frequency
  *
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
uint32_t SysCtlClockGet(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief  requests frequency of the given clock
  *
  * @param  idx NUT_HWCLK Index
  * @retval clock or 0 if idx points to an invalid clock
  */
uint32_t STM_ClockGet(int idx)
{
    SystemCoreClockUpdate();
    switch(idx) {
    case NUT_HWCLK_CPU:			/* SYSCLK */
        return SystemCoreClock;
        break;

    case NUT_HWCLK_AHB: {		/* AHB Clock */
        uint32_t tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> _BI32( RCC_CFGR_PPRE1_0);
        return SystemCoreClock/APBPrescTable[tmp];
        break;
    }
    case NUT_HWCLK_APB1: {		/* APB1 Clock */
        uint32_t tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> _BI32( RCC_CFGR_PPRE2_0);
        return SystemCoreClock/APBPrescTable[tmp];
        break;
    }
    case NUT_HWCLK_APB2: {		/* APB1 Clock */
        uint32_t tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> _BI32( RCC_CFGR_PPRE2_0);
        return SystemCoreClock/APBPrescTable[tmp];
        break;
    }
    case NUT_HWCLK_ADC: {		/* ADC Clock */
    	/* ADC clock is derived directly from HSI */
    	return (RCC->CR & RCC_CR_HSIRDY) ? HSI_VALUE:0;
    }
    //TODO: LSE_OSC
    //TODO: LSI_RC
    //TODO: MCO
    default:
        return 0;
        break;
    }
}
