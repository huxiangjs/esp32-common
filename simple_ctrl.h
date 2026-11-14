/*
 * MIT License
 *
 * Copyright (c) 2024 Hoozz (huxiangjs)
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

#ifndef __SIMPLE_CTRL_H_
#define __SIMPLE_CTRL_H_

#include "class_id.h"
#include "ota.h"

#ifdef __cplusplus
extern "C" {
#endif

void simple_ctrl_init(const char *name);
void simple_ctrl_request_register(int (*request)(char *buffer, int buf_offs, int vaild_size, int buff_size));
void simple_ctrl_notify(char *buffer, int size);
void simple_ctrl_set_name(const char *new_name);
void simple_ctrl_set_class_id(uint8_t new_id);
/*
 * Enable OTA: store the firmware version (for the FW_VERSION command), the
 * supported transports (for the FW_UPGRADE command), the platform image
 * backend (where the downloaded image goes) and the optional pre-restart
 * callback (run by the transfer task right before the reboot into the new
 * image, so the app can clean up its peripherals; see ota_enable()). Call
 * once at startup, after the backend's storage is initialized (ESP8266:
 * after nvs_flash_init()).
 */
void simple_ctrl_ota_enable(const char *version,
			    const struct ota_transport *transports,
			    uint8_t count, const struct ota_flash *flash,
			    void (*pre_restart)(void));

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLE_CTRL_H_ */
