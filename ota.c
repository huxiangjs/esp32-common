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
 *
 * The platform-agnostic OTA core.
 *
 * The transfer itself is a simple stream: the client first sends the
 * 4-byte little-endian firmware size, then the raw image in chunks. The
 * transfer task opens the transport (chosen by the url scheme), streams
 * the image into the platform image backend (struct ota_flash), lets the
 * backend verify and commit it, then restarts. One task drives everything,
 * so the simple_ctrl control channel stays free to answer progress
 * queries while the image streams in.
 *
 * This file deliberately knows nothing about a concrete platform: the
 * image storage, first-boot validation and restart all live behind the
 * ota_flash backend (ESP-IDF: esp32-common/ota_flash.c, shared by ESP8266
 * and ESP32), and the data source behind ota_transport (the "hoozz"
 * tcp_crypto tunnel is implemented here). On other platforms the same
 * core compiles as-is with a different backend (Linux: a backup-file
 * swap, planned).
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include "ota.h"
#include "os.h"
#include "tcp_crypto.h"

static const char *TAG = "OTA";

/* hoozz (tcp_crypto) transfer parameters */
/*
 * One whole tcp_crypto frame: 2-byte length + 16-byte IV + the chunk's
 * zero-padded ciphertext. Must hold the largest message the connection
 * carries: ((OTA_IO_BUF_SIZE - 18) & ~15) >= OTA_TRANSFER_CHUNK.
 */
#define OTA_IO_BUF_SIZE			(2 + 16 + ((OTA_TRANSFER_CHUNK + 15) & ~15))
#define OTA_TRANSFER_CHUNK		1024
#define OTA_READ_TIMEOUT_MS		15000
#define OTA_ACCEPT_TIMEOUT_MS		30000
#define OTA_RESTART_DELAY_MS		3000
/*
 * The transfer task's stack. It does the same work as the tunnel session
 * task (tcp_crypto accept/recv, TUNNEL_SESSION_STACK = 4096) plus the
 * backend's image write path (on ESP-IDF: the esp_ota_write() flash path,
 * static buffers inside the spi_flash driver, modest stack), so it matches
 * the tunnel session task and leaves margin.
 */
#define OTA_TASK_STACK			4096
#define OTA_URL_MAX			64

/* The current version, the supported transports and the image backend,
 * set by ota_enable(). */
static const char *ota_version = "unknown";
static const struct ota_transport *ota_transports;
static uint8_t ota_transport_count;
static const struct ota_flash *ota_flash;

/* Shared with the simple_ctrl progress query. */
static volatile uint8_t ota_progress = 0;
static volatile bool ota_busy = false;

/* The in-flight transfer (single OTA at a time, guarded by ota_busy). */
static const struct ota_transport *ota_task_transport;
static char ota_task_url[OTA_URL_MAX];

/* hoozz (tcp_crypto) transport state, driven only by the transfer task. */
static struct tcp_crypto *hoozz_client;
static int hoozz_listen_fd = -1;
static uint8_t hoozz_key[TCP_CRYPTO_KEY_LENGTH];
static uint32_t hoozz_total;
static uint32_t hoozz_received;

/*
 * Find the transport whose scheme matches the start of [url] ([len] is the
 * url length; the url need not be NUL-terminated).
 */
static const struct ota_transport *ota_find_transport(const char *url,
		uint32_t len)
{
	uint8_t i;
	size_t n;

	if (len < 3)
		return NULL;
	for (i = 0; i < ota_transport_count; i++) {
		n = strlen(ota_transports[i].name);
		if (len < n + 3)
			continue;
		if (strncmp(url, ota_transports[i].name, n) == 0 &&
		    url[n] == ':' && url[n + 1] == '/')
			return &ota_transports[i];
	}
	return NULL;
}

/* --- hoozz (tcp_crypto) transport --- */

/*
 * url = "hoozz://<key16>/<port>". The key is 16 visible characters, used
 * directly as the AES-128 key (no hex encoding). Open a listener on
 * <port>, authenticate a client (the key), then read the 4-byte
 * little-endian firmware total size the client sends first.
 */
int hoozz_ota_open(const char *url)
{
	const char *p, *slash;
	struct sockaddr_in address;
	int port;
	int operate = 1;
	uint8_t size_buf[4];
	int i;
	int n;

	p = url + strlen("hoozz://");
	slash = strchr(p, '/');
	if (!slash) {
		OS_LOGE(TAG, "hoozz url missing <port>");
		return -1;
	}

	/* The key is the 16 visible chars before the '/' */
	if ((uint32_t)(slash - p) != TCP_CRYPTO_KEY_LENGTH) {
		OS_LOGE(TAG, "hoozz url key must be %d visible chars",
			 TCP_CRYPTO_KEY_LENGTH);
		return -1;
	}
	for (i = 0; i < TCP_CRYPTO_KEY_LENGTH; i++) {
		if (p[i] < 0x21 || p[i] > 0x7e) {
			OS_LOGE(TAG, "hoozz url key has a non-visible char");
			return -1;
		}
		hoozz_key[i] = (uint8_t)p[i];
	}

	port = atoi(slash + 1);
	if (port < 1 || port > 65535) {
		OS_LOGE(TAG, "hoozz url has a bad port: %d", port);
		return -1;
	}

	hoozz_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (hoozz_listen_fd < 0) {
		OS_LOGE(TAG, "Socket failed: %d", errno);
		return -1;
	}
	setsockopt(hoozz_listen_fd, SOL_SOCKET, SO_REUSEADDR, &operate,
			sizeof(operate));
	address.sin_family = AF_INET;
	address.sin_port = htons((uint16_t)port);
	address.sin_addr.s_addr = INADDR_ANY;
	if (bind(hoozz_listen_fd, (struct sockaddr *)&address,
			sizeof(address)) < 0) {
		OS_LOGE(TAG, "Bind port %d failed: %d", port, errno);
		close(hoozz_listen_fd);
		hoozz_listen_fd = -1;
		return -1;
	}
	if (listen(hoozz_listen_fd, 1) < 0) {
		OS_LOGE(TAG, "Listen failed: %d", errno);
		close(hoozz_listen_fd);
		hoozz_listen_fd = -1;
		return -1;
	}

	/*
	 * Wait for the client and authenticate it (the handshake is done
	 * inside tcp_crypto_accept). Blocks until it connects or the timeout
	 * is used up.
	 */
	hoozz_client = tcp_crypto_accept(hoozz_listen_fd, hoozz_key,
			OTA_IO_BUF_SIZE, OTA_ACCEPT_TIMEOUT_MS);
	if (!hoozz_client) {
		OS_LOGE(TAG, "No OTA client connected on port %d", port);
		close(hoozz_listen_fd);
		hoozz_listen_fd = -1;
		return -1;
	}

	/* The client sends the 4-byte little-endian total size first. */
	n = tcp_crypto_recv(hoozz_client, size_buf, sizeof(size_buf),
			OTA_READ_TIMEOUT_MS);
	if (n != (int)sizeof(size_buf)) {
		OS_LOGE(TAG, "Failed to read firmware size (%d)", n);
		tcp_crypto_close(hoozz_client);
		hoozz_client = NULL;
		close(hoozz_listen_fd);
		hoozz_listen_fd = -1;
		return -1;
	}
	hoozz_total = (uint32_t)size_buf[0] |
		((uint32_t)size_buf[1] << 8) |
		((uint32_t)size_buf[2] << 16) |
		((uint32_t)size_buf[3] << 24);
	hoozz_received = 0;

	return 0;
}

uint32_t hoozz_ota_size(void)
{
	return hoozz_total;
}

int hoozz_ota_read(uint8_t *buf, int buf_max, uint32_t timeout_ms)
{
	int n;

	if (hoozz_received >= hoozz_total)
		return 0;
	n = tcp_crypto_recv(hoozz_client, buf, buf_max, timeout_ms);
	if (n < 0)
		return -1;
	/* A timeout here means the client stalled (the stream is not
	 * expected to end early), so treat it as an error. */
	if (n == 0)
		return -1;
	hoozz_received += (uint32_t)n;
	return n;
}

void hoozz_ota_close(void)
{
	if (hoozz_client) {
		tcp_crypto_close(hoozz_client);
		hoozz_client = NULL;
	}
	if (hoozz_listen_fd >= 0) {
		close(hoozz_listen_fd);
		hoozz_listen_fd = -1;
	}
}

/* --- the transfer task --- */

/*
 * The OTA transfer task: open the transport, stream the firmware into the
 * image backend, let the backend verify and commit it, then restart.
 */
static void ota_transfer_task(void *arg)
{
	const struct ota_flash *f = ota_flash;
	const struct ota_transport *t = ota_task_transport;
	const char *url = ota_task_url;
	void *handle = NULL;
	uint32_t total, written;
	int n;
	uint8_t *buf = NULL;
	bool opened = false;
	int ret;

	if (!f) {
		OS_LOGE(TAG, "No OTA image backend");
		goto fail;
	}

	buf = malloc(OTA_TRANSFER_CHUNK);
	if (!buf) {
		OS_LOGE(TAG, "No memory for transfer buffer");
		goto fail;
	}

	ret = t->open(url);
	if (ret < 0) {
		OS_LOGE(TAG, "Transport open failed: %d", ret);
		goto fail;
	}
	opened = true;

	total = t->size();
	if (total == 0) {
		OS_LOGE(TAG, "Invalid firmware size (0)");
		goto fail;
	}
	OS_LOGI(TAG, "OTA started: %u bytes", (unsigned)total);

	ret = f->image_begin(total, &handle);
	if (ret < 0) {
		OS_LOGE(TAG, "Image begin failed: %d", ret);
		goto fail;
	}

	written = 0;
	for (;;) {
		/* n holds the read byte count; image_write() returns a status
		 * code (0 = ok), so its result must not overwrite it. */
		n = t->read(buf, OTA_TRANSFER_CHUNK, OTA_READ_TIMEOUT_MS);
		if (n == 0)
			break;
		if (n < 0) {
			OS_LOGE(TAG, "Transfer read failed");
			goto fail;
		}
		ret = f->image_write(handle, buf, (uint32_t)n);
		if (ret < 0) {
			OS_LOGE(TAG, "Image write failed: %d", ret);
			goto fail;
		}
		written += (uint32_t)n;
		ota_progress = (uint8_t)((written * 100) / total);
	}

	ret = f->image_end(handle);
	handle = NULL;
	if (ret < 0) {
		OS_LOGE(TAG, "Image end (verify) failed: %d", ret);
		goto fail;
	}

	/*
	 * Success: the image is written and verified. Close the transport,
	 * then commit it (switch the boot target, arm the first-boot
	 * validation). (The transport is closed first so a failure below
	 * cannot double-close it.)
	 */
	t->close();
	opened = false;
	free(buf);
	buf = NULL;

	ret = f->commit();
	if (ret < 0) {
		OS_LOGE(TAG, "Image commit failed: %d", ret);
		goto fail;
	}

	/* Restart into the new image */
	ota_progress = 100;
	OS_LOGI(TAG, "OTA complete, restarting in %d ms", OTA_RESTART_DELAY_MS);
	OS_MSLEEP(OTA_RESTART_DELAY_MS);
	f->restart();

fail:
	if (opened)
		t->close();
	if (buf)
		free(buf);
	ota_progress = 0;
	ota_busy = false;
	OS_THREAD_EXIT();
}

/* --- public API --- */

int ota_upgrade(const char *url, uint32_t len)
{
	const struct ota_transport *t;
	bool ok;

	if (ota_busy) {
		OS_LOGW(TAG, "OTA already in progress, ignoring");
		return -1;
	}
	t = ota_find_transport(url, len);
	if (!t) {
		OS_LOGE(TAG, "Unknown transport in url (%u bytes)", (unsigned)len);
		return -1;
	}
	/* len + 1: the copy is NUL-terminated for the transport callbacks */
	if (len + 1 > sizeof(ota_task_url)) {
		OS_LOGE(TAG, "OTA url too long (%u bytes)", (unsigned)len);
		return -1;
	}

	memcpy(ota_task_url, url, len);
	ota_task_url[len] = '\0';
	ota_task_transport = t;
	ota_busy = true;
	ota_progress = 0;

	OS_THREAD_CREATE(ok, NULL, ota_transfer_task, NULL,
			"ota_task", OTA_TASK_STACK);
	if (!ok) {
		OS_LOGE(TAG, "Failed to create OTA task");
		ota_busy = false;
		return -1;
	}

	OS_LOGI(TAG, "OTA upgrade started (url: %s)", ota_task_url);
	return 0;
}

const char *ota_get_version(void)
{
	return ota_version;
}

uint8_t ota_get_progress(void)
{
	return ota_progress;
}

void ota_enable(const char *version, const struct ota_transport *transports,
		uint8_t count, const struct ota_flash *flash)
{
	ota_version = version ? version : "unknown";
	ota_transports = transports;
	ota_transport_count = count;
	ota_flash = flash;

	OS_LOGI(TAG, "OTA enabled, version %s, %u transport(s)",
		 ota_version, (unsigned)count);

	if (ota_flash)
		ota_flash->first_boot_check();
}
