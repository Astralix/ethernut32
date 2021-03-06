/* arcfour.c */
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
/*
 * File:        arcfour.c
 * Author:      Daniel Otte
 * email:       daniel.otte@rub.de
 * Date:        2006-06-07
 * License:     GPLv3 or later
 * Description: Implementation of the ARCFOUR (RC4 compatible) stream cipher algorithm.
 *
 */

#include <stdint.h>
#include <crypto/arcfour.h>

/*
 * length is length of key in bits!
 */

void arcfour_init(const void *key, uint16_t length_b, arcfour_ctx_t *ctx){
	uint8_t t;
	uint8_t x=0,y=0;
	length_b /= 8;
	const uint8_t *kptr, *limit;
	limit = (uint8_t*)key + length_b;
	kptr = key;
	do{
		ctx->s[x]=x;
	}while(++x);

	do{
		y += ctx->s[x] + *kptr++;
		if(kptr==limit){
			kptr=key;
		}
		/* ctx->s[y] <--> ctx->s[x] */
		t = ctx->s[y];
		ctx->s[y] = ctx->s[x];
		ctx->s[x] = t;
	}while(++x);
	ctx->i = ctx->j = 0;
}

uint8_t arcfour_gen(arcfour_ctx_t *ctx){
	uint8_t t;
	ctx->i++;
	ctx->j += ctx->s[ctx->i];
	/* ctx->s[i] <--> ctx->s[j] */
	t = ctx->s[ctx->j];
	ctx->s[ctx->j] = ctx->s[ctx->i];
	ctx->s[ctx->i] = t;
	return ctx->s[(ctx->s[ctx->j] + ctx->s[ctx->i]) & 0xff];
}

