/*
 * Copyright (C) 2012, 2013 Uwe Bonnes
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

#ifndef _INCLUDE_IAP_FLASH_H
#define _INCLUDE__IAP_FLASH_H
#include <cfg/memory.h>

typedef enum
{
    FLASH_ERASE_ALWAYS      = 0,   /*!< Always erase whole memory block before writing */
    FLASH_ERASE_FIRST_TOUCH = 1,   /*!< Only erase whole memory block with first write to that block*/
    FLASH_ERASE_NEVER       = 2,   /*!< Write without erasing */
} FLASH_ERASE_MODE;

/*!
 * brief FLASH status returns of any operation.
 */
typedef enum
{
    FLASH_BUSY          =  1,   /*!< Flash operation pending */
    FLASH_COMPLETE      =  0,   /*!< Flash operation successfull completed */
    FLASH_ERROR_PG      = -1,   /*!< Flash programming failed */
    FLASH_ERROR_WRP     = -2,   /*!< Flash write protected */
    FLASH_LOCKED        = -3,   /*!< FLASH is locked, unlocking failed */
    FLASH_TIMEOUT       = -4,   /*!< Flash operation timed out */
    FLASH_BOUNDARY      = -5,   /*!< Flash write crosses page/sector border */
    FLASH_COMPARE       = -6,   /*!< FLASH compare mismatch */
    FLASH_OUT_OF_MEMORY = -7,   /*!< Intermediate buffer allocation failed*/
    FLASH_ERR_CONF_LAYOUT = -8,   /*!< CONF_PAGE has unexpected layout*/
    FLASH_CONF_OVERFLOW = -9,   /*!< Requested CONF parameter overflow conf storage area*/
    FLASH_ERR_ALIGNMENT   = -10,   /*!< Unaligned Flash access*/
    FLASH_NOT_IMPLEMENTED = -11,   /*!< Not implemented*/
} FLASH_Status;

#if defined(IAP_FLASH)
extern FLASH_Status  IapFlashWrite( void* dst, void* src, size_t len, FLASH_ERASE_MODE mode);
extern FLASH_Status IapFlashWriteProtect(void *dst, size_t len, int ena);
extern uint32_t IapFlashEnd(void);
extern void FlashUntouch(void);
#else
FLASH_Status  IapFlashWrite( void* dst, void* src, size_t len, FLASH_ERASE_MODE mode)
{
    return FLASH_NOT_IMPLEMENTED;
}
FLASH_Status  IapFlashWriteProtect(void *dst, size_t len, int ena)
{
    return FLASH_NOT_IMPLEMENTED;
}
uint32_t IapFlashEnd(void)
{
    return 0;
}
void FlashUntouch(void){};
#endif
#endif
