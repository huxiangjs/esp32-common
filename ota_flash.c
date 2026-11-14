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
 * The ESP-IDF backend for the ota_flash interface (ota.h), shared by
 * ESP8266 and ESP32: the esp_ota_* APIs have identical signatures on both
 * SDKs.
 *
 * The new image goes to the other OTA partition. The first-boot validation
 * is hand-rolled in NVS (ESP32 has a built-in rollback state machine, but
 * this validation works identically on both chips and keeps the backend
 * portable), and the restart is esp_restart().
 */

#include <stdint.h>

#include "ota.h"
#include "ota_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs.h>

static const char *TAG = "OTA-FLASH";

/* The NVS handle type differs between SDKs (uint32_t on ESP8266, pointer
 * on ESP32); the call syntax is the same. */
#if defined(CONFIG_IDF_TARGET_ESP8266)
typedef nvs_handle ota_nvs_handle_t;
#else
typedef nvs_handle_t ota_nvs_handle_t;
#endif

/*
 * First-boot validation.
 *
 * After an OTA the device reboots into the new image, which is untrusted
 * until it has actually run. Two bytes in NVS track this:
 *   - "pending": 1 while the current boot is the first boot of a new image
 *   - "attempt": how many times we have booted this new image
 *
 *   - after a successful OTA: store pending=1, attempt=0, then reboot
 *   - on a boot with pending=1: bump attempt; if it reaches
 *     OTA_MAX_BOOT_ATTEMPTS the image clearly cannot even start, so switch
 *     the boot partition back to the other one (the previous image) and
 *     reboot
 *   - if the image survives OTA_VALIDATE_TIMEOUT_MS of operation, a small
 *     task clears the flag, marking the new image valid
 *
 * There is always a valid "previous" image to roll back to: an OTA writes to
 * the partition that is NOT currently running, so the other partition still
 * holds the last-working image.
 */
#define OTA_VALIDATE_TIMEOUT_MS		10000
#define OTA_MAX_BOOT_ATTEMPTS		3

#define OTA_NVS_NAMESPACE		"ota"
#define OTA_NVS_PENDING_KEY		"pending"
#define OTA_NVS_ATTEMPT_KEY		"attempt"

/* The esp_ota handle of the in-flight write (single OTA at a time) */
static esp_ota_handle_t ota_handle;

/* --- NVS first-boot validation --- */

static void ota_nvs_mark_pending(void)
{
	ota_nvs_handle_t h;

	if (nvs_open(OTA_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return;
	nvs_set_u8(h, OTA_NVS_PENDING_KEY, 1);
	nvs_set_u8(h, OTA_NVS_ATTEMPT_KEY, 0);
	nvs_commit(h);
	nvs_close(h);
}

static void ota_nvs_clear_pending(void)
{
	ota_nvs_handle_t h;

	if (nvs_open(OTA_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return;
	nvs_set_u8(h, OTA_NVS_PENDING_KEY, 0);
	nvs_set_u8(h, OTA_NVS_ATTEMPT_KEY, 0);
	nvs_commit(h);
	nvs_close(h);
}

/*
 * The new image survived long enough: mark it valid (clear the pending flag
 * so later boots no longer run the validation).
 */
static void ota_validation_task(void *arg)
{
	vTaskDelay(pdMS_TO_TICKS(OTA_VALIDATE_TIMEOUT_MS));
	ota_nvs_clear_pending();
	ESP_LOGI(TAG, "First boot OK after %d ms, new image marked valid",
		 OTA_VALIDATE_TIMEOUT_MS);
	vTaskDelete(NULL);
}

/*
 * Run the first-boot validation. Called from ota_enable() after
 * nvs_flash_init(). If the current boot is the first boot of a new image
 * that keeps failing to start, roll back to the previous image and reboot.
 */
static void ota_first_boot_check(void)
{
	ota_nvs_handle_t h;
	const esp_partition_t *running, *prev;
	uint8_t pending = 0, attempt = 0;

	if (nvs_open(OTA_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return;
	nvs_get_u8(h, OTA_NVS_PENDING_KEY, &pending);
	nvs_get_u8(h, OTA_NVS_ATTEMPT_KEY, &attempt);
	nvs_close(h);

	if (pending != 1)
		return;

	attempt++;
	ESP_LOGI(TAG, "First boot of new image, attempt %u/%u",
		 (unsigned)attempt, OTA_MAX_BOOT_ATTEMPTS);

	if (attempt >= OTA_MAX_BOOT_ATTEMPTS) {
		/*
		 * The new image cannot even start; roll back to the other
		 * partition (the last-working image) and reboot into it.
		 */
		running = esp_ota_get_running_partition();
		prev = esp_ota_get_next_update_partition(running);
		ESP_LOGE(TAG, "Too many failed boots, rolling back to %s",
			 prev ? prev->label : "unknown");
		/* If the switch itself fails there is nothing else to do
		 * (reboot into the same broken image), just make it visible */
		if (prev && esp_ota_set_boot_partition(prev) != ESP_OK)
			ESP_LOGE(TAG, "Rollback failed: set_boot_partition error");
		ota_nvs_clear_pending();
		esp_restart();
	}

	/* Remember the attempt count and give the new image a chance. */
	if (nvs_open(OTA_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
		nvs_set_u8(h, OTA_NVS_ATTEMPT_KEY, attempt);
		nvs_commit(h);
		nvs_close(h);
	}

	/*
	 * The task only sleeps, makes a few NVS calls and deletes itself,
	 * so 2 KB is plenty. xTaskCreate takes the stack size in BYTES on
	 * both SDKs (the ESP8266 port defines StackType_t as one byte, the
	 * ESP32 port documents the parameter as "NUMBER OF BYTES"), so the
	 * same value works on both.
	 */
	if (xTaskCreate(ota_validation_task, "ota_validate", 2048, NULL,
			tskIDLE_PRIORITY + 2, NULL) != pdPASS)
		ESP_LOGE(TAG, "Failed to start first-boot validation task");
}

/* --- image write (esp_ota_* into the other OTA partition) --- */

static int ota_image_begin(uint32_t image_size, void **handle)
{
	const esp_partition_t *next;
	esp_ota_handle_t h;
	esp_err_t ret;

	next = esp_ota_get_next_update_partition(esp_ota_get_running_partition());
	if (!next) {
		ESP_LOGE(TAG, "No OTA update partition");
		return -1;
	}
	ESP_LOGI(TAG, "Writing image to partition %s", next->label);

	ret = esp_ota_begin(next, (size_t)image_size, &h);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_begin failed: 0x%x", ret);
		return -1;
	}
	ota_handle = h;
	*handle = &ota_handle;

	return 0;
}

static int ota_image_write(void *handle, const uint8_t *data, uint32_t size)
{
	esp_err_t ret;

	if (handle != &ota_handle)
		return -1;
	ret = esp_ota_write(ota_handle, data, (size_t)size);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_write failed: 0x%x", ret);
		return -1;
	}
	return 0;
}

static int ota_image_end(void *handle)
{
	esp_err_t ret;

	if (handle != &ota_handle)
		return -1;
	ret = esp_ota_end(ota_handle);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_end (verify) failed: 0x%x", ret);
		return -1;
	}
	return 0;
}

/*
 * Switch the next boot to the image we just wrote and arm the first-boot
 * validation for it.
 */
static int ota_commit(void)
{
	const esp_partition_t *next;

	next = esp_ota_get_next_update_partition(esp_ota_get_running_partition());
	if (!next)
		return -1;
	if (esp_ota_set_boot_partition(next) != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed");
		return -1;
	}
	ota_nvs_mark_pending();

	return 0;
}

/* On this platform the restart is a full chip reboot. */
static void ota_restart(void)
{
	esp_restart();
}

const struct ota_flash esp_idf_ota_flash = {
	ota_first_boot_check,
	ota_image_begin,
	ota_image_write,
	ota_image_end,
	ota_commit,
	ota_restart,
};
