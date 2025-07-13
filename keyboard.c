
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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include "keyboard.h"
#include "event_bus.h"

static const char *TAG = "KEYBOARD";

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define portYIELD_FROM_ISR portYIELD
#endif

#define GPIO_PIN_KEYBOARD_SEL(NUM)	(1ULL << (NUM))

#define KEYBOARD_DETECTION_INTERVA	40
#define KEYBOARD_LONG_PRESS		4000
#define KEYBOARD_LONG_PRESS_COUNT	(KEYBOARD_LONG_PRESS / KEYBOARD_DETECTION_INTERVA)

static TaskHandle_t handle;
static uint8_t *key_gpios;
static uint8_t key_num;

static int gpio_find_first_low_level(void)
{
	uint8_t index;
	int retval = -1;

	for (index = 0; index < key_num; index++) {
		if (!gpio_get_level(key_gpios[index])) {
			retval = key_gpios[index];
			break;
		}
	}

	return retval;
}

static void keyboard_task(void *arg)
{
	uint8_t count = 0;
	int gpio;
	struct event_bus_msg msg = {
		.type = EVENT_BUS_KEYBOARD,
	};

	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		gpio = gpio_find_first_low_level();
		if (gpio < 0)
			continue;
		msg.param1 = (uint32_t)gpio;

		count = 0;
		vTaskDelay(pdMS_TO_TICKS(KEYBOARD_DETECTION_INTERVA));
		count++;
		if (!gpio_get_level(gpio)) {
			msg.param2 = KEYBOARD_EVENT_KEY_PRESS;
			event_bus_send(&msg);
			ESP_LOGI(TAG, "keyboard: [%d] key press", gpio);
		} else {
			continue;
		}

		while (!gpio_get_level(gpio) &&
		       count < KEYBOARD_LONG_PRESS_COUNT) {
			count++;
			vTaskDelay(pdMS_TO_TICKS(KEYBOARD_DETECTION_INTERVA));
		}

		if(count == KEYBOARD_LONG_PRESS_COUNT){
			msg.param2 = KEYBOARD_EVENT_LONG_RELEASE;
			event_bus_send(&msg);
			ESP_LOGI(TAG, "keyboard: [%d] long release", gpio);
		} else {
			msg.param2 = KEYBOARD_EVENT_SHORT_RELEASE;
			event_bus_send(&msg);
			ESP_LOGI(TAG, "keyboard: [%d] short release", gpio);
			while(!gpio_get_level(gpio))
				vTaskDelay(pdMS_TO_TICKS(KEYBOARD_DETECTION_INTERVA));
		}
	}
}

static void gpio_isr_handle(void *param)
{
	BaseType_t high_task_wakeup;

	vTaskNotifyGiveFromISR(handle, &high_task_wakeup);

	if (high_task_wakeup != pdFALSE) {
		portYIELD_FROM_ISR();
	}
}

static void keyboard_gpio_init(uint8_t gpio)
{
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_NEGEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = GPIO_PIN_KEYBOARD_SEL(gpio),
		.pull_down_en = 0,
		.pull_up_en = 0,
	};
	ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void keyboard_init(uint8_t *gpios, uint8_t num)
{
	int ret;
	uint8_t index;

	key_gpios = (uint8_t *)malloc(sizeof(uint8_t) * num);
	ESP_ERROR_CHECK(key_gpios == NULL);

	ret = xTaskCreate(keyboard_task, "keyboard_task", 2048, NULL, tskIDLE_PRIORITY + 2, &handle);
	ESP_ERROR_CHECK(ret != pdPASS);

	// gpio_set_intr_type(GPIO_PIN_KEYBOARD, GPIO_INTR_NEGEDGE);
	ESP_ERROR_CHECK(gpio_install_isr_service(0));

	for (index = 0; index < num; index++) {
		key_gpios[index] = gpios[index];
		keyboard_gpio_init(key_gpios[index]);
		ret = gpio_isr_handler_add(key_gpios[index], gpio_isr_handle, NULL);
		ESP_ERROR_CHECK(ret);
		key_num++;
	}
}
