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

#ifndef __TCP_CRYPTO_H_
#define __TCP_CRYPTO_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AES128 key length */
#define TCP_CRYPTO_KEY_LENGTH		16

/* One accepted and authenticated client connection (fd + AES context). */
struct tcp_crypto;

/*
 * Generic AES128-CBC encrypted transport over TCP: drop-in replacements
 * for accept(), send() and recv(). The module owns no listener and no
 * thread, the caller drives everything and passes a timeout to each
 * blocking call.
 *
 * A handle is not thread-safe: exactly one task must drive it.
 *
 * On connect, the first message both ways is the 9-byte string
 * "HoozzPlay" (with the normal framing). The server decrypts it: only
 * a client holding the right key can produce a matching plaintext, so
 * this authenticates the client; a client with a bad handshake is
 * disconnected.
 *
 * Every message is framed as: a 2-byte little-endian plaintext length,
 * 16 IV bytes, then the AES128-CBC ciphertext (the last partial block
 * is zero-padded). The whole frame is written or read in one go. The
 * IV is a per-connection 128-bit counter, seeded with random bytes and
 * advanced by one for every frame in that direction (a nonce+counter
 * scheme, like TLS for AES-CBC).
 */

/*
 * accept() replacement: wait up to timeout_ms for a client on
 * `listen_fd` and authenticate it: the first message the client sends
 * must decrypt to "HoozzPlay" (which proves it holds `key`, 16 bytes),
 * and the server answers with the same message.
 *
 * io_buf_size is the size, in bytes, of each whole-frame I/O buffer
 * (one for sending, one for receiving). The buffer must hold the whole
 * frame (2-byte length + 16 IV bytes + 16-byte-aligned ciphertext), so
 * the largest message the connection can carry is
 * ((io_buf_size - 18) & ~15) plaintext bytes. io_buf_size must be at
 * least 18 (a zero-length frame); a size below that returns NULL.
 *
 * A client whose handshake fails is disconnected and the function
 * keeps accepting new clients until timeout_ms is fully consumed; it
 * does not give up after a single bad handshake. Each handshake gets
 * its own budget (up to HANDSHAKE_TIMEOUT_MS, capped by the time left
 * in timeout_ms), so a slow first message is still accepted as long
 * as the overall wait has not run out.
 *
 * Returns a new connection handle, or NULL when timeout_ms is used up.
 */
struct tcp_crypto *tcp_crypto_accept(int listen_fd,
				     const uint8_t *key,
				     uint16_t io_buf_size,
				     uint32_t timeout_ms);

/*
 * send() replacement: transmit `data` as one encrypted message:
 * 2-byte little-endian plaintext length, 16 IV bytes (the per-connection
 * counter, see above), then the AES128-CBC ciphertext (the last partial
 * block is zero-padded), written as one whole frame. Blocks up to
 * timeout_ms.
 *
 * Returns 0 on success, <0 on failure (the client connection is
 * dropped, the handle stays valid, close it when done).
 */
int tcp_crypto_send(struct tcp_crypto *tc,
		    const uint8_t *data, int size, uint32_t timeout_ms);

/*
 * recv() replacement: read one encrypted message into `buf` (which must
 * be able to hold the whole plaintext): the 2-byte length header gives
 * the plaintext size, the following 16 IV bytes prime the context, and
 * the whole frame is read in one go and decrypted; the zero padding is
 * trimmed. Blocks up to timeout_ms.
 *
 * Returns the plaintext length (>0), 0 on timeout, or <0 on failure
 * (the client connection is dropped, the handle stays valid, close it
 * when done).
 */
int tcp_crypto_recv(struct tcp_crypto *tc,
		    uint8_t *buf, int buf_max, uint32_t timeout_ms);

/* Is the client connected (the connection has not been dropped)? */
bool tcp_crypto_is_connected(struct tcp_crypto *tc);

/*
 * Are there already bytes received from the client that have not been
 * consumed yet, or has the peer closed the connection?
 * Returns 1 yes, 0 no.
 */
int tcp_crypto_peek(struct tcp_crypto *tc);

/* Close the client connection and free the handle. */
void tcp_crypto_close(struct tcp_crypto *tc);

#ifdef __cplusplus
}
#endif

#endif /* __TCP_CRYPTO_H_ */
