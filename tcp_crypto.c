/*
 * MIT License
 *
 * Copyright (c) 2026 huxiangjs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, subject to the following conditions:
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "os.h"
#include "tiny-AES-c/aes.h"
#include "tcp_crypto.h"

/*
 * AES backend: on the ESP8266 use the SDK's T-table esp_aes driver
 * (components/util) -- the same implementation wpa_supplicant links
 * against, so its tables and code are already in the image.
 *
 * Root cause of the original ~4 KB/s tunnel throughput: tiny-AES-c is
 * byte-oriented and took ~50 ms per 256-byte frame (~3 ms per 16-byte
 * block) on this hardware, i.e. the crypto was the bottleneck. The
 * T-table path encrypts the same frame in ~3 ms (P7 self-test
 * measurement: 3008 us/256B), raising tunnel throughput to ~30 KB/s.
 *
 * Gotcha: esp_aes_{en,de}crypt_cbc() CONSUMES THE IV BUFFER IN PLACE
 * (on return it holds the last ciphertext block, the CBC chain state).
 * The wrappers below run it on a local copy so the frame's IV field
 * keeps the original random IV on the wire; tiny-AES-c has the same
 * semantics (AES_ctx_set_iv copies the IV into the context).
 *
 * Other targets keep tiny-AES-c.
 */
#if defined(CONFIG_IDF_TARGET_ESP8266)
# include <esp_aes.h>
#endif

static const char *TAG = "TCP-CRYPTO";

#define HANDSHAKE_STRING		"HoozzPlay"
#define HANDSHAKE_SIZE			(sizeof(HANDSHAKE_STRING) - 1)

/*
 * The handshake's own timeout, independent of the accept wait: a client
 * may take a moment to send its first message, so the caller's accept
 * wait is far too tight for the exchange.
 */
#define HANDSHAKE_TIMEOUT_MS		2000

/*
 * Whole-frame I/O buffers: one per direction, each sized by the caller
 * at tcp_crypto_accept() time (io_buf_size, bytes). The frame (2-byte
 * length, 16 IV bytes, ciphertext) is built in the send buffer and
 * written in one go, and received into the receive buffer in one go --
 * one write/read per message instead of one per 16-byte block. The
 * buffer is worth keeping: this lwIP's recv() costs ~440 us per call,
 * so reading a 1024-byte frame in one recv() instead of 64 x 16-byte
 * recv()s is the difference between ~59 KB/s and ~22 KB/s in the
 * receive direction.
 *
 * A buffer must hold the whole frame: 2 (length) + 16 (IV) + ciphertext
 * (a multiple of 16). The largest plaintext message a buffer of N bytes
 * can carry is ((N - 18) & ~15) bytes.
 *
 * The handle and the two buffers are three separate heap allocations
 * (no MMU on the ESP8266: several small blocks fragment better than
 * one ~2 KB contiguous block).
 */
/* Smallest I/O buffer: a zero-length frame (2 length + 16 IV) */
#define TCP_CRYPTO_MIN_IO_BUF_SIZE	(2 + TCP_CRYPTO_KEY_LENGTH)

/*
 * Per-connection IV generator: a 128-bit counter, seeded once per
 * connection with 128 bits of entropy (os_fill_random, the same
 * source the session key is drawn from) and incremented for every
 * frame.
 *
 * AES-CBC requires the IV to be unique per message under the same key
 * -- it does not need to be secret. The tunnel key is per-connection,
 * so a unique, incrementing IV is sufficient. This is the same
 * nonce+counter scheme TLS uses for AES-CBC (a counter, not a PRNG).
 * The entropy seed keeps IVs from ever colliding across connections
 * even if a key were somehow reused. (A xoshiro128++ PRNG used here
 * originally provided no benefit a counter lacks -- the IV only needs
 * uniqueness, not secrecy -- and cost ~25 extra lines.)
 */

struct tcp_crypto {
	int fd;
#if defined(CONFIG_IDF_TARGET_ESP8266)
	/* Separate contexts: the esp_aes decrypt context holds an
	 * inverse key schedule, the encrypt context the forward one */
	esp_aes_t enc_ctx;
	esp_aes_t dec_ctx;
#else
	struct AES_ctx aes_ctx;
#endif
	/* Per-frame IV: a 128-bit counter (see the generator docblock) */
	uint8_t iv_ctr[TCP_CRYPTO_KEY_LENGTH];
	/* Size of each whole-frame I/O buffer, chosen by the caller at
	 * tcp_crypto_accept() time (see the I/O buffer docblock) */
	uint16_t io_buf_size;
	/* The whole-frame I/O buffers, one per direction: three separate
	 * mallocs (struct + two buffers) instead of one large contiguous
	 * block, which the ESP8266 heap fragments better */
	uint8_t *send_buf;
	uint8_t *recv_buf;
};

/* Seed the per-connection IV counter with 128 bits of entropy */
static void tcp_crypto_seed_iv(struct tcp_crypto *tc)
{
	os_fill_random(tc->iv_ctr, TCP_CRYPTO_KEY_LENGTH);
}

/*
 * Fill the frame's 16-byte IV with the current counter value and advance
 * it. The counter is 128 bits, so it cannot wrap within a connection.
 */
static void tcp_crypto_fill_iv(struct tcp_crypto *tc, uint8_t *buf)
{
	int i;

	memcpy(buf, tc->iv_ctr, TCP_CRYPTO_KEY_LENGTH);
	for (i = TCP_CRYPTO_KEY_LENGTH - 1; i >= 0; i--) {
		if (++tc->iv_ctr[i])
			break;
	}
}

/*
 * AES backend wrappers: one key setup per connection and one in-place
 * AES-128-CBC operation per frame. The caller's IV buffer is never
 * touched: the esp_aes backend consumes the IV in place (updates it to
 * the last ciphertext block), so it runs on a copy. This matters on
 * the send path, where the IV field of the frame must keep the original
 * random IV on the wire. tiny-AES-c has the same semantics (its
 * AES_ctx_set_iv copies the IV into the context).
 */
static void tcp_crypto_aes_set_key(struct tcp_crypto *tc, const uint8_t *key)
{
#if defined(CONFIG_IDF_TARGET_ESP8266)
	esp_aes_set_encrypt_key(&tc->enc_ctx, key, 128);
	esp_aes_set_decrypt_key(&tc->dec_ctx, key, 128);
#else
	AES_init_ctx(&tc->aes_ctx, key);
#endif
}

static void tcp_crypto_aes_encrypt(struct tcp_crypto *tc, uint8_t *iv,
				   uint8_t *ct, int ct_len)
{
#if defined(CONFIG_IDF_TARGET_ESP8266)
	uint8_t iv_copy[TCP_CRYPTO_KEY_LENGTH];

	memcpy(iv_copy, iv, sizeof(iv_copy));
	esp_aes_encrypt_cbc(&tc->enc_ctx, ct, ct_len, ct, ct_len, iv_copy);
#else
	AES_ctx_set_iv(&tc->aes_ctx, iv);
	AES_CBC_encrypt_buffer(&tc->aes_ctx, ct, ct_len);
#endif
}

static void tcp_crypto_aes_decrypt(struct tcp_crypto *tc, uint8_t *iv,
				   uint8_t *ct, int ct_len)
{
#if defined(CONFIG_IDF_TARGET_ESP8266)
	uint8_t iv_copy[TCP_CRYPTO_KEY_LENGTH];

	memcpy(iv_copy, iv, sizeof(iv_copy));
	esp_aes_decrypt_cbc(&tc->dec_ctx, ct, ct_len, ct, ct_len, iv_copy);
#else
	AES_ctx_set_iv(&tc->aes_ctx, iv);
	AES_CBC_decrypt_buffer(&tc->aes_ctx, ct, ct_len);
#endif
}

static bool tcp_crypto_ensure_connected(struct tcp_crypto *tc)
{
	return tc != NULL && tc->fd >= 0;
}

/*
 * Drop the client connection (a failed transfer leaves a broken
 * stream); the handle stays owned by the caller, which must free it
 * with tcp_crypto_close().
 */
static void tcp_crypto_drop_client(struct tcp_crypto *tc)
{
	if (tc->fd >= 0) {
		close(tc->fd);
		tc->fd = -1;
	}
}

/*
 * Read `size` bytes from `fd`: timeout_ms bounds a single read
 * operation, not the whole transfer: as long as no individual recv()
 * stalls for longer than timeout_ms, a slow peer that trickles the data
 * down is fine.
 *
 * The bound is implemented with SO_RCVTIMEO (set once per call, on a
 * fresh local timeval) instead of a select()+recv() pair: recv() on a
 * blocking socket already waits for data, so the extra select() only
 * added per-iteration overhead. With SO_RCVTIMEO a blocking recv()
 * returns as soon as data is available, or with EAGAIN once timeout_ms
 * has passed with no data. A fresh local timeval per call also avoids
 * the SO_RCVTIMEO reuse bug seen in simple_ctrl, where a select()-
 * mutated struct was handed to setsockopt.
 *
 * Defensive note: this lwIP build makes a timed socket non-blocking on
 * the SEND path (netconn_write_partly forces dontblock, see
 * write_exact). If the same applied to recv(), a timed recv() would
 * return EAGAIN before any data rather than after the full wait. The
 * elapsed-time check below handles both behaviours: EAGAIN is only
 * treated as a timeout once the full budget is used up, otherwise it
 * yields briefly and retries.
 *
 * Returns 0 on success, 1 on timeout, or <0 on error.
 */
static int tcp_crypto_read_exact(int fd, uint8_t *buf, int size, uint32_t timeout_ms)
{
	struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
	uint32_t start = os_get_time_ms();
	int count = 0;
	int ret;

	if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
		OS_LOGE(TAG, "Set SO_RCVTIMEO failed: %d", errno);
		return -1;
	}

	while (count < size) {
		ret = recv(fd, buf + count, size - count, 0);
		if (ret > 0) {
			count += ret;
			continue;
		}
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				/* Either SO_RCVTIMEO fired (no data in time) or a
				 * non-blocking recv() returned before any data.
				 * Only a real timeout once the full budget is used
				 * up; otherwise yield a moment and retry. */
				if (os_get_time_ms() - start >= timeout_ms)
					return 1;
				OS_MSLEEP(1);
				continue;
			}
			OS_LOGE(TAG, "Recv failed: %d", errno);
			return -1;
		}
		/* ret == 0: the peer closed the connection */
		OS_LOGI(TAG, "Peer closed the connection");
		return -1;
	}

	return 0;
}

/*
 * Write `size` bytes to `fd`: timeout_ms bounds a single write
 * operation, not the whole transfer.
 *
 * SO_SNDTIMEO is set to timeout_ms once up front. In this lwIP build a
 * non-zero SO_SNDTIMEO makes send() non-blocking (netconn_write_partly
 * forces dontblock): a full buffer returns EAGAIN immediately instead of
 * blocking, so the per-op bound holds without any per-send timer math.
 *
 * On EAGAIN the send buffer is full; wait for the peer's ACKs to free
 * space with a bounded select-for-writable, then retry. The select is
 * fast here: a failed non-blocking write sets NETCONN_FLAG_CHECK_WRITESPACE,
 * so the next sent_tcp raises SENDPLUS as soon as ANY space frees
 * (tcp_sndbuf > 0, per ACK). The blocking path would only let select
 * report writable once the 2920-byte buffer drains below TCP_SNDLOWAT
 * (2919), i.e. a full ACK round trip per frame (~50 ms) -- that is the
 * ~5 KB/s ceiling this design avoids.
 *
 * Returns 0 on success, 1 on timeout, or <0 on error.
 */
static int tcp_crypto_write_exact(int fd, const uint8_t *buf, int size, uint32_t timeout_ms)
{
	struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
	int count = 0;
	fd_set writefds;
	int ret;

	if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
		OS_LOGE(TAG, "Setsockopt failed: %d", errno);
		return -1;
	}

	while (count < size) {
		ret = send(fd, buf + count, size - count, 0);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				OS_LOGE(TAG, "Send failed: %d", errno);
				return -1;
			}

			/* Buffer full: wait for an ACK to free space (bounded by
			 * timeout_ms, the per-op limit), then retry */
			FD_ZERO(&writefds);
			FD_SET(fd, &writefds);
			ret = select(fd + 1, NULL, &writefds, NULL, &tv);
			if (ret < 0) {
				if (errno == EINTR)
					continue;
				OS_LOGE(TAG, "Select failed: %d", errno);
				return -1;
			}
			if (ret == 0)
				return 1; /* the send timed out */
		} else if (ret == 0) {
			OS_LOGE(TAG, "Peer closed the connection");
			return -1;
		} else {
			count += ret;
		}
	}

	return 0;
}

/*
 * Authenticate a new client: the first message it sends must decrypt
 * to "HoozzPlay" (only possible with the right key), and the answer is
 * the same message. Any mismatch drops the client.
 *
 * The answer uses the normal whole-frame tcp_crypto_send(); the exchange
 * takes a few hundred ms at most -- covered by HANDSHAKE_TIMEOUT_MS.
 */
static bool tcp_crypto_handshake(struct tcp_crypto *tc, uint32_t timeout_ms)
{
	uint8_t buf[HANDSHAKE_SIZE];
	int size;

	size = tcp_crypto_recv(tc, buf, sizeof(buf), timeout_ms);
	if (size != HANDSHAKE_SIZE ||
	    memcmp(buf, HANDSHAKE_STRING, HANDSHAKE_SIZE) != 0) {
		OS_LOGW(TAG, "Handshake mismatch, client dropped");
		return false;
	}

	if (tcp_crypto_send(tc, (const uint8_t *)HANDSHAKE_STRING,
			HANDSHAKE_SIZE, timeout_ms) < 0) {
		OS_LOGE(TAG, "Handshake send failed");
		return false;
	}

	return true;
}

struct tcp_crypto *tcp_crypto_accept(int listen_fd,
				     const uint8_t *key,
				     uint16_t io_buf_size,
				     uint32_t timeout_ms)
{
	struct tcp_crypto *tc;
	fd_set readfds;
	struct timeval tv;
	uint32_t start = os_get_time_ms();
	uint32_t elapsed;
	uint32_t remaining;
	uint32_t hs;
	int fd;
	int ret;

	if (!key || listen_fd < 0)
		return NULL;
	if (io_buf_size < TCP_CRYPTO_MIN_IO_BUF_SIZE) {
		OS_LOGE(TAG, "io_buf_size too small: %u",
			 (unsigned int)io_buf_size);
		return NULL;
	}

	for (;;) {
		elapsed = os_get_time_ms() - start;
		if (elapsed >= timeout_ms)
			return NULL;
		remaining = timeout_ms - elapsed;

		FD_ZERO(&readfds);
		FD_SET(listen_fd, &readfds);
		tv.tv_sec = remaining / 1000;
		tv.tv_usec = (remaining % 1000) * 1000;
		ret = select(listen_fd + 1, &readfds, NULL, NULL, &tv);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			OS_LOGE(TAG, "Select failed: %d", errno);
			return NULL;
		}
		if (ret == 0)
			return NULL;

		fd = accept(listen_fd, NULL, NULL);
		if (fd < 0) {
			OS_LOGI(TAG, "Accept failed: %d", errno);
			/* Keep trying until the timeout is used up */
			continue;
		}

		/* Handle and I/O buffers are separate small allocations
		 * (see the I/O buffer docblock) */
		tc = (struct tcp_crypto *)malloc(sizeof(*tc));
		if (tc) {
			memset(tc, 0, sizeof(*tc));
			tc->fd = fd;
			tc->io_buf_size = io_buf_size;
			tc->send_buf = (uint8_t *)malloc(io_buf_size);
			tc->recv_buf = (uint8_t *)malloc(io_buf_size);
		}
		if (!tc || !tc->send_buf || !tc->recv_buf) {
			OS_LOGE(TAG, "Malloc failed");
			if (tc) {
				free(tc->send_buf);
				free(tc->recv_buf);
				free(tc);
			}
			close(fd);
			return NULL;
		}

		/* Seed the IV counter before the handshake (its answer
		 * also uses tcp_crypto_send) */
		tcp_crypto_seed_iv(tc);

		/*
		 * Disable Nagle on the data socket. Without it, each
		 * 274-byte frame is held back while the previous segment
		 * is unacknowledged (tcp_do_output_nagle: the only unsent
		 * segment is smaller than the MSS), and the peer's delayed
		 * ACK (40 ms) plus the RTT (~9 ms) paces the stream to one
		 * frame per ~50 ms (~4.5 KB/s). With NODELAY every frame
		 * goes out as soon as the send buffer has room (~10 frames
		 * in flight).
		 */
		{
			int nodelay = 1;

			if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay,
					sizeof(nodelay)) < 0)
				OS_LOGW(TAG, "Set TCP_NODELAY failed: %d", errno);
		}

		tcp_crypto_aes_set_key(tc, key);

		/* The handshake gets its own budget (see HANDSHAKE_TIMEOUT_MS),
		 * capped by the time left in the overall wait: a failed
		 * handshake drops the client and tries again instead of
		 * giving up, until timeout_ms is fully consumed */
		hs = (HANDSHAKE_TIMEOUT_MS < remaining) ? HANDSHAKE_TIMEOUT_MS
			: remaining;
		if (tcp_crypto_handshake(tc, hs))
			return tc;
		OS_LOGW(TAG, "Handshake failed, retrying until the timeout");
		tcp_crypto_close(tc);
		/* loop back: re-accept until timeout_ms is used up */
	}
}

int tcp_crypto_send(struct tcp_crypto *tc,
		    const uint8_t *data, int size, uint32_t timeout_ms)
{
	uint8_t *ct;
	int ct_total;
	int frame_size;
	int ret;

	if (!data || size < 0)
		return -1;
	if (!tcp_crypto_ensure_connected(tc))
		return -1;

	ct_total = (size + 15) & ~15;
	frame_size = 2 + TCP_CRYPTO_KEY_LENGTH + ct_total;
	if (frame_size > tc->io_buf_size) {
		OS_LOGE(TAG, "Message too large for the send buffer: %d",
			 frame_size);
		return -1;
	}

	/* 2-byte little-endian plaintext length */
	tc->send_buf[0] = (uint8_t)(size & 0xff);
	tc->send_buf[1] = (uint8_t)((size >> 8) & 0xff);

	/* 16 fresh IV bytes for the frame (per-connection counter, see
	 * tcp_crypto_fill_iv) */
	tcp_crypto_fill_iv(tc, tc->send_buf + 2);

	/* The ciphertext: the zero-padded plaintext is placed in the frame
	 * and encrypted in place in one go (the backend chains the CBC
	 * blocks internally and consumes the IV) */
	ct = tc->send_buf + 2 + TCP_CRYPTO_KEY_LENGTH;
	memcpy(ct, data, size);
	if (size < ct_total)
		memset(ct + size, 0, ct_total - size);
	tcp_crypto_aes_encrypt(tc, tc->send_buf + 2, ct, ct_total);

	/* One write of the whole frame, instead of one per 16-byte block */
	ret = tcp_crypto_write_exact(tc->fd, tc->send_buf, frame_size,
			timeout_ms);
	if (ret != 0) {
		OS_LOGE(TAG, "Send failed: %d", ret);
		tcp_crypto_drop_client(tc);
		return -1;
	}

	return 0;
}

int tcp_crypto_recv(struct tcp_crypto *tc,
		    uint8_t *buf, int buf_max, uint32_t timeout_ms)
{
	uint8_t len[2];
	int size;
	int ct_total;
	int frame_size;
	int ret;

	if (!buf || buf_max <= 0)
		return -1;
	if (!tcp_crypto_ensure_connected(tc))
		return -1;

	/* 2-byte little-endian plaintext length */
	ret = tcp_crypto_read_exact(tc->fd, len, 2, timeout_ms);
	if (ret != 0)
		goto fail;
	size = len[0] | (len[1] << 8);
	if (size > buf_max) {
		OS_LOGE(TAG, "Message too large: %d", size);
		tcp_crypto_drop_client(tc);
		return -1;
	}

	ct_total = (size + 15) & ~15;
	frame_size = TCP_CRYPTO_KEY_LENGTH + ct_total;
	if (frame_size > tc->io_buf_size) {
		OS_LOGE(TAG, "Message too large for the recv buffer: %d",
			 frame_size);
		tcp_crypto_drop_client(tc);
		return -1;
	}

	/* IV(16) + ciphertext, read in one go into the recv buffer */
	ret = tcp_crypto_read_exact(tc->fd, tc->recv_buf, frame_size,
			timeout_ms);
	if (ret != 0)
		goto fail;

	/* Decrypt the whole ciphertext in place (the backend chains the
	 * CBC blocks internally); only `size` bytes are copied to the
	 * caller, the zero padding is trimmed */
	tcp_crypto_aes_decrypt(tc, tc->recv_buf,
			tc->recv_buf + TCP_CRYPTO_KEY_LENGTH, ct_total);
	memcpy(buf, tc->recv_buf + TCP_CRYPTO_KEY_LENGTH, size);

	return size;

fail:
	tcp_crypto_drop_client(tc);
	return ret == 1 ? 0 : -1;
}

bool tcp_crypto_is_connected(struct tcp_crypto *tc)
{
	return tcp_crypto_ensure_connected(tc);
}

int tcp_crypto_peek(struct tcp_crypto *tc)
{
	fd_set readfds;
	struct timeval tv = { 0, 0 };
	int ret;

	if (!tcp_crypto_ensure_connected(tc))
		return 0;

	FD_ZERO(&readfds);
	FD_SET(tc->fd, &readfds);
	ret = select(tc->fd + 1, &readfds, NULL, NULL, &tv);
	if (ret < 0)
		return 0;

	return ret > 0 ? 1 : 0;
}

void tcp_crypto_close(struct tcp_crypto *tc)
{
	if (!tc)
		return;

	if (tc->fd >= 0) {
		close(tc->fd);
		tc->fd = -1;
	}
	free(tc->send_buf);
	free(tc->recv_buf);
	free(tc);
}
