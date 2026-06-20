/*
 * MIT License
 *
 * Copyright (c) 2024 huxiangjs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "os.h"
#include "crypto.h"
#include "tiny-AES-c/aes.h"

static const char *TAG = "CRYPTO";

struct algorithm {
	int (*en)(struct crypto *handle, char *buffer, int vaild_size, int buff_size);
	int (*de)(struct crypto *handle, char *buffer, int vaild_size, int buff_size);
	uint8_t head_reserve;
	uint8_t tail_reserve;
};

static int en_none(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	/* Do nothing */

	return vaild_size;
}

static int de_none(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	/* Do nothing */

	return vaild_size;
}

static int do_xor(struct crypto *handle, char *buffer, int vaild_size)
{
	int index;

	if (handle->plen == 0)
		goto out;

	for (index = 0; index < vaild_size; index++) {
		buffer[index] ^= handle->passwd[handle->count % handle->plen];
		handle->count++;
	}

out:
	return vaild_size;
}

static int en_xor(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_xor(handle, buffer, vaild_size);
}

static int de_xor(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_xor(handle, buffer, vaild_size);
}

static inline int aes128_check_key(struct crypto *handle)
{
	if (handle->plen > 16) {
		OS_LOGE(TAG, "The key length of AES128 must be 16 bytes");
		return -1;
	}

	return 0;
}

static inline int aes128_check_size(int vaild_size, int buff_size)
{
	if ((vaild_size & 0xf) && (((vaild_size >> 4) + 1) > (buff_size >> 4))) {
		OS_LOGE(TAG, "Not enough space in the buffer");
		return -1;
	}

	return 0;
}

static inline void aes128_fill_key(struct crypto *handle, uint8_t *key)
{
	if (handle->plen)
		memcpy(key, handle->passwd, handle->plen);
	if (handle->plen != 16)
		memset(key + handle->plen, 0, 16 - handle->plen);
}

static int do_aes128ecb(struct crypto *handle, char *buffer, int vaild_size, int buff_size, bool en)
{
	struct AES_ctx ctx;
	uint8_t key[16];
	int ret;

	ret = aes128_check_size(vaild_size, buff_size);
	if (ret)
		return ret;
	ret = aes128_check_key(handle);
	if (ret)
		return ret;
	aes128_fill_key(handle, key);

	ret = vaild_size & 0xf;
	if (ret)
		memset(buffer + vaild_size, 0, 16 - ret);

	AES_init_ctx(&ctx, key);
	if (en) {
		for (ret = 0; ret < vaild_size; ret += 16)
			AES_ECB_encrypt(&ctx, (uint8_t *)buffer + ret);
	} else {
		for (ret = 0; ret < vaild_size; ret += 16)
			AES_ECB_decrypt(&ctx, (uint8_t *)buffer + ret);
	}

	return ret;
}

static int do_aes128cbc(struct crypto *handle, char *buffer, int vaild_size, int buff_size, bool en)
{
	struct AES_ctx ctx;
	uint8_t key[16];
	uint8_t *iv;
	int ret;
	int i;

	iv = (uint8_t *)buffer;
	buffer += handle->head_reserve;
	vaild_size -= handle->head_reserve;
	buff_size -= handle->head_reserve;

	ret = aes128_check_size(vaild_size, buff_size);
	if (ret)
		return ret;
	ret = aes128_check_key(handle);
	if (ret)
		return ret;
	aes128_fill_key(handle, key);

	ret = vaild_size & 0xf;
	if (ret) {
		ret = 16 - ret;
		memset(buffer + vaild_size, 0, ret);
		vaild_size += ret;
	}

	if (en) {
		/* init iv */
		srand(time(NULL));
		for (i = 0; i < 16; i++)
			iv[i] = rand() % 256;
	}

	if (en) {
		AES_init_ctx_iv(&ctx, key, iv);
		AES_CBC_encrypt_buffer(&ctx, (uint8_t *)buffer, vaild_size);
		ret = handle->head_reserve + vaild_size;
	} else {
		AES_init_ctx_iv(&ctx, key, iv);
		AES_CBC_decrypt_buffer(&ctx, (uint8_t *)buffer, vaild_size);
		ret = handle->head_reserve + vaild_size;
	}

	return ret;
}

static int en_aes128ecb(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_aes128ecb(handle, buffer, vaild_size, buff_size, true);
}

static int de_aes128ecb(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_aes128ecb(handle, buffer, vaild_size, buff_size, false);
}

static int en_aes128cbc(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_aes128cbc(handle, buffer, vaild_size, buff_size, true);
}

static int de_aes128cbc(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_aes128cbc(handle, buffer, vaild_size, buff_size, false);
}

static struct algorithm list[CRYPTO_TYPE_MAX] = {
	[CRYPTO_TYPE_NONE] = { en_none, de_none, 0, 0 },		/* 0x00: No encryption */
	[CRYPTO_TYPE_XOR] = { en_xor, de_xor, 0, 0 },			/* 0x01: Simple xor replacement */
	[CRYPTO_TYPE_AES128ECB] = { en_aes128ecb, de_aes128ecb, 0, 0 },	/* 0x02: AES128-ECB */
	[CRYPTO_TYPE_AES128CBC] = { en_aes128cbc, de_aes128cbc, 16, 0 },/* 0x03: AES128-CBC */
};

int crypto_en(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return list[handle->type].en(handle, buffer, vaild_size, buff_size);
}

int crypto_de(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return list[handle->type].de(handle, buffer, vaild_size, buff_size);
}

int crypto_init(struct crypto *handle, int type, char *passwd, int plen)
{
	if (!handle) {
		OS_LOGE(TAG, "A null pointer is used");
		return -1;
	}

	if (type >= CRYPTO_TYPE_MAX) {
		OS_LOGE(TAG, "Unsupported encryption type: 0x%02x", type);
		return -1;
	}

	handle->count = 0;
	handle->type = type;
	handle->head_reserve = list[type].head_reserve;
	handle->tail_reserve = list[type].tail_reserve;

	if (passwd && plen) {
		handle->passwd = passwd;
		handle->plen = plen;
	} else {
		handle->passwd = NULL;
		handle->plen = 0;
	}

	return 0;
}
