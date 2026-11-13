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

#ifndef __OTA_FLASH_H_
#define __OTA_FLASH_H_

#include "ota.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The ESP-IDF ota_flash backend (ota_flash.c, shared by ESP8266 and ESP32):
 * the other OTA partition via esp_ota_*, first-boot validation in NVS,
 * esp_restart(). Pass it to simple_ctrl_ota_enable()/ota_enable() at
 * startup.
 */
extern const struct ota_flash esp_idf_ota_flash;

#ifdef __cplusplus
}
#endif

#endif /* __OTA_FLASH_H_ */
