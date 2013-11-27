/*
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 * Copyright (C) 2010 by Nikolaj Zamotaev. All rights reserved.
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
 * \verbatim
 * $Id: stm32f1_rtc.c 5381 2013-10-07 11:01:12Z u_bonnes $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>
#include <cfg/arch.h>
#include <sys/event.h>
#include <sys/timer.h>
#include <dev/rtc.h>

#include <cfg/arch/gpio.h>
#include <arch/cm3/stm/vendor/stm32f10x.h>

#include <stdlib.h>
#include <string.h>
#include <time.h>


static void Stm32RtcRtoffPoll(void)
{
    while((RTC->CRL & RTC_CRL_RTOFF)==0);
    return;
};

/*!
 * \brief Get date and time from an DS1307 hardware clock.
 *
 * \param tm Points to a structure that receives the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int Stm32RtcGetClock(NUTRTC *rtc, struct _tm *tm)
{
    int rc=-1;
    time_t time;
    RTC->CRL&= ~(RTC_CRL_RSF);
    while(!(RTC->CRL & RTC_CRL_RSF));
    time=((RTC->CNTL|(RTC->CNTH<<16)));
    localtime_r(&time,tm);
    rc=0;
    return rc;
}

/*!
 * \brief Set the DS1307 hardware clock.
 *
 * \param tm Points to a structure which contains the date and time
 *           information.
 *
 * \return 0 on success or -1 in case of an error.
 */
int Stm32RtcSetClock(NUTRTC *rtc, const struct _tm *tm)
{
   time_t time;
   Stm32RtcRtoffPoll();
   RTC->CRL|= RTC_CRL_CNF;
   time=mktime((struct _tm *)tm);
   RTC->CNTL =time& 0xffffUL;
   RTC->CNTH =(time >> 16) &0xffffUL;
   RTC->CRL&= ~(RTC_CRL_CNF);
   Stm32RtcRtoffPoll();
   return 0;
}


/*!
 * \brief Initialize the RTC in stm32f10x controller
 *
 * \return 0 on success or -1 in case of an error.
 *
 */
int Stm32RtcInit(NUTRTC *rtc)
{
    int rc=-1;
    uint32_t temp;


   RCC->APB1ENR |= RCC_APB1ENR_BKPEN|RCC_APB1ENR_PWREN;
   PWR->CR |= PWR_CR_DBP;
   RCC->BDCR|= (1<<16);//Backup domain reset;
   RCC->BDCR&= ~(1<<16);//Backup domain reset;
   PWR->CR|= PWR_CR_DBP;

   temp=RCC->BDCR;
   temp&= ~(RCC_BDCR_RTCSEL);
   /*Fixme: Don't assume fixed HSE frequency here*/
   temp |= RCC_BDCR_RTCSEL;//HSE/128
   temp |= RCC_BDCR_RTCEN;
   RCC->BDCR=temp;
   RTC->CRL&= ~(RTC_CRL_RSF);
   while(!(RTC->CRL & RTC_CRL_RSF));

   Stm32RtcRtoffPoll();
   RTC->CRL|= RTC_CRL_CNF;
   RTC->PRLH = 0;
   RTC->PRLL = (8000000ul/128)-1;//��� �������� �� 128 �� ������ 8���
   //������������� ��������� �����
   RTC->CNTL= 1;
   RTC->CNTH= 0;//1 january 1970
   RTC->CRL&= ~(RTC_CRL_CNF);
   Stm32RtcRtoffPoll();
   rc=0;
   return rc;
}

NUTRTC rtcStm32 = {
  /*.dcb           = */ NULL,               /*!< Driver control block */
  /*.rtc_init      = */ Stm32RtcInit,       /*!< Hardware initializatiuon, rtc_init */
  /*.rtc_gettime   = */ Stm32RtcGetClock,   /*!< Read date and time, rtc_gettime */
  /*.rtc_settime   = */ Stm32RtcSetClock,   /*!< Set date and time, rtc_settime */
  /*.rtc_getalarm  = */ NULL,               /*!< Read alarm date and time, rtc_getalarm */
  /*.rtc_setalarm  = */ NULL,               /*!< Set alarm date and time, rtc_setalarm */
  /*.rtc_getstatus = */ NULL,               /*!< Read status flags, rtc_getstatus */
  /*.rtc_clrstatus = */ NULL,               /*!< Clear status flags, rtc_clrstatus */
  /*.alarm         = */ NULL,               /*!< Handle for alarm event queue, not supported right now */
};
