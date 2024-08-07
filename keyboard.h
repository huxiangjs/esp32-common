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

#ifndef __KEYBOARD_H_
#define __KEYBOARD_H_

#include <stdbool.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KEYBOARD_EVENT_KEY_PRESS	0x0
#define KEYBOARD_EVENT_SHORT_RELEASE	0x1
#define KEYBOARD_EVENT_LONG_RELEASE	0x2

void keyboard_init(uint8_t *gpios, uint8_t num);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_H_ */
