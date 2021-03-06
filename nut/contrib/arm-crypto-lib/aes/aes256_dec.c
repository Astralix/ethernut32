/* aes256_dec.c */
/*
    This file is part of the ARM-Crypto-Lib.
    Copyright (C) 2006-2010  Daniel Otte (daniel.otte@rub.de)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
 * \file     aes256_dec.c
 * \email    daniel.otte@rub.de
 * \author   Daniel Otte 
 * \date     2008-12-31
 * \license  GPLv3 or later
 * 
 */

#include <crypto/aes.h>
#include <crypto/aes_dec.h>

void aes256_dec(void* buffer, aes256_ctx_t* ctx){
	aes_decrypt_core(buffer, (aes_genctx_t*)ctx, 14);
}

