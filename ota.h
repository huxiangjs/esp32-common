/*
 * MIT License
 *
 * Copyright (c) 2026 huxiangjs
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

#ifndef __OTA_H_
#define __OTA_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * An OTA firmware-download transport.
 *
 * One descriptor per supported download method. `name` is bound to the URL
 * scheme used in the FW_UPGRADE command:
 *
 *   name "hoozz"  ->  url "hoozz://<key>/<port>"  (our tcp_crypto tunnel)
 *   name "http"   ->  url "http://xxxx/xxxx"      (not implemented yet)
 *   name "https"  ->  url "https://xxxx/xxxxx"    (not implemented yet)
 *
 * The function pointers abstract the data transfer so the OTA core only
 * depends on this interface, never on a specific transport. A transport is
 * single-threaded: exactly one task drives it for the whole transfer.
 */
struct ota_transport {
	/* The URL scheme this transport handles (matched at the start of the url) */
	const char *name;

	/*
	 * Parse [url] and set up the transfer. For "hoozz" this opens a
	 * listening socket on the port carried in the url, authenticates a
	 * client (the key in the url is 16 visible chars, used directly as
	 * the AES key), and reads the firmware total size. Blocks until the
	 * client connects or the timeout is used up.
	 *
	 * Returns 0 on success, <0 on failure.
	 */
	int (*open)(const char *url);

	/* Total firmware size (bytes); valid after open() succeeded. */
	uint32_t (*size)(void);

	/*
	 * Read the next firmware chunk into [buf] (at most [buf_max] bytes).
	 * [timeout_ms] bounds the blocking read.
	 *
	 * Returns >0 = bytes read, 0 = end of transfer, <0 = error.
	 */
	int (*read)(uint8_t *buf, int buf_max, uint32_t timeout_ms);

	/* Tear down the transfer and release its resources. */
	void (*close)(void);
};

/*
 * A platform firmware-image backend.
 *
 * The OTA core (ota.c) is transport- AND platform-agnostic: it streams the
 * image from an ota_transport into an ota_flash. The backend knows where the
 * new image goes and how the device switches to it:
 *
 *   ESP-IDF (esp32-common/ota_flash.c, shared by ESP8266 and ESP32): the
 *           other OTA partition via esp_ota_*, first-boot validation in
 *           NVS, esp_restart()
 *   Linux  (planned): a backup file the application swaps in on its next
 *           start, no first-boot validation needed
 *
 * A backend is single-threaded: exactly one task drives it, and at most one
 * transfer runs at a time. All int-returning functions return 0 on success,
 * <0 on failure.
 */
struct ota_flash {
	/*
	 * Run the first-boot validation (called once at startup, when the
	 * backend's storage is ready). The backend owns its whole validation
	 * state machine: on ESP-IDF the new image must survive a validation
	 * window of operation, or the boot target is rolled back to the
	 * previous image after a fixed number of failed boots.
	 */
	void (*first_boot_check)(void);

	/*
	 * Start writing a new image of [image_size] bytes into the update
	 * target. [handle] receives an opaque token (may be NULL) that is
	 * passed back to image_write()/image_end().
	 */
	int (*image_begin)(uint32_t image_size, void **handle);
	int (*image_write)(void *handle, const uint8_t *data, uint32_t size);
	/* Verify the written image and end the write session. */
	int (*image_end)(void *handle);

	/*
	 * Commit the verified image: make it the next boot target and arm the
	 * first-boot validation. The caller restarts shortly after.
	 */
	int (*commit)(void);

	/*
	 * Restart (into the committed image). On ESP-IDF this is a full chip
	 * reboot (esp_restart()); on Linux it would just restart the app
	 * process.
	 */
	void (*restart)(void);
};

/*
 * Enable OTA: store the current firmware version, the list of supported
 * transports and the platform image backend, then run the first-boot
 * validation (the backend's one).
 *
 * Must be called once at startup, after the backend's storage is initialized
 * (ESP8266: after nvs_flash_init()).
 */
void ota_enable(const char *version,
		const struct ota_transport *transports, uint8_t count,
		const struct ota_flash *flash);

/* The stored firmware version string (the argument to ota_enable). */
const char *ota_get_version(void);

/*
 * Start an upgrade from [url] ([len] bytes, not necessarily
 * NUL-terminated). Only validates the url and spawns the transfer task; it
 * does not block. Returns 0 when the upgrade has been started, <0 on
 * failure (unknown scheme, already busy, ...).
 */
int ota_upgrade(const char *url, uint32_t len);

/* Current upgrade progress, 0-100 (0 = not started, 100 = done, about to restart). */
uint8_t ota_get_progress(void);

/*
 * The "hoozz" transport callbacks (our tcp_crypto tunnel). Referenced by
 * main.c to build the transport table; implemented in ota.c.
 */
int hoozz_ota_open(const char *url);
uint32_t hoozz_ota_size(void);
int hoozz_ota_read(uint8_t *buf, int buf_max, uint32_t timeout_ms);
void hoozz_ota_close(void);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_H_ */
