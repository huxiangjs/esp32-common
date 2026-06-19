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

#ifndef __OS_H_
#define __OS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__linux__)

#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#if defined(DEBUG)
#define OS_DEBUG 1
#else
#define OS_DEBUG 0
#endif /* DEBUG */

#define OS_LOGD(tag, fmt, ...) do { \
	if (OS_DEBUG) printf("[D] %s: " fmt "\n", tag, ##__VA_ARGS__); \
} while (0)
#define OS_LOGI(tag, fmt, ...) printf("[I] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define OS_LOGW(tag, fmt, ...) printf("[W] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define OS_LOGE(tag, fmt, ...) printf("[E] %s: " fmt "\n", tag, ##__VA_ARGS__)

#define OS_ERROR_CHECK(x) assert(!(x))

struct linux_thread {
	pthread_t tid;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	bool ready;
};

#define OS_THREAD struct linux_thread
#define OS_THREAD_RET void *
#define OS_THREAD_CREATE(ret, handle, code, arg, name, s_dep) do { \
	int ret_hub = 0; \
	pthread_t tid; \
	pthread_t *p = &tid; \
	if (handle != NULL) { \
		p = &((OS_THREAD *)handle)->tid; \
		ret_hub |= pthread_mutex_init(&((OS_THREAD *)handle)->mutex, NULL); \
		ret_hub |= pthread_cond_init(&((OS_THREAD *)handle)->cond, NULL); \
		((OS_THREAD *)handle)->ready = false; \
	} \
	if (pthread_create(p, NULL, code, arg)) ret = false; \
	else ret = true; \
} while (0)
#define OS_THREAD_SLEEP(handle) do { \
	pthread_mutex_lock(&((OS_THREAD *)handle)->mutex); \
	while (((OS_THREAD *)handle)->ready == false) \
		pthread_cond_wait(&((OS_THREAD *)handle)->cond, \
		&((OS_THREAD *)handle)->mutex); \
	pthread_mutex_unlock(&((OS_THREAD *)handle)->mutex); \
} while (0)
#define OS_THREAD_WAKEUP(handle) do { \
	pthread_mutex_lock(&((OS_THREAD *)handle)->mutex); \
	((OS_THREAD *)handle)->ready = true; \
	pthread_cond_signal(&((OS_THREAD *)handle)->cond); \
	pthread_mutex_unlock(&((OS_THREAD *)handle)->mutex); \
} while (0)
#define OS_THREAD_EXIT() return 0
#define OS_MSLEEP(x) usleep(x * 1000)
#define OS_MUTEX pthread_mutex_t
#define OS_MUTEX_INIT(ret, x) do { \
	if (pthread_mutex_init(x, NULL) != 0) ret = false; \
	else ret = true; \
} while (0)
#define OS_MUTEX_LOCK(x) pthread_mutex_lock(x)
#define OS_MUTEX_UNLOCK(x) pthread_mutex_unlock(x)

struct linux_pipe {
	int pipe_fd[2];
	ssize_t is;
};

#define OS_QUEUE struct linux_pipe
#define OS_QUEUE_INIT(ret, queue, count, item_size) do { \
	(queue)->is = item_size; \
	ret = pipe((queue)->pipe_fd) != 0 ? false : true; \
} while (0)
#define OS_QUEUE_SEND(queue, buff) \
	(write((queue)->pipe_fd[1], buff, (queue)->is) == (queue)->is)
#define OS_QUEUE_RECV(queue, buff) \
	(read((queue)->pipe_fd[0], buff, (queue)->is) == (queue)->is)
#define OS_QUEUE_DEL(queue) do { \
	close((queue)->pipe_fd[0]); \
	close((queue)->pipe_fd[1]); \
} while (0)

void os_get_fixed_id(uint8_t *id);

#else /* ESP32 OR ESP8266 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <lwip/sockets.h>
#include <lwip/dns.h>
#include <lwip/netdb.h>
#include <esp_log.h>
#if defined(CONFIG_IDF_TARGET_ESP8266)
# include <esp_wifi.h>
#else
#include <esp_mac.h>
#endif /* CONFIG_IDF_TARGET_ESP8266 */

#define OS_LOGD ESP_LOGD
#define OS_LOGI ESP_LOGI
#define OS_LOGW ESP_LOGW
#define OS_LOGE ESP_LOGE

#define OS_ERROR_CHECK ESP_ERROR_CHECK

#define TASK_PRIORITY (tskIDLE_PRIORITY + 2)

#define OS_THREAD TaskHandle_t
#define OS_THREAD_RET void
#define OS_THREAD_CREATE(ret, handle, code, arg, name, s_dep) do { \
	if (xTaskCreate(code, name, s_dep, arg, TASK_PRIORITY, handle) != pdPASS) \
		ret = false; \
	else ret = true; \
} while (0)
#define OS_THREAD_SLEEP(handle) ulTaskNotifyTake(pdTRUE, portMAX_DELAY)
#define OS_THREAD_WAKEUP(handle) xTaskNotifyGive(*(handle))
#define OS_THREAD_EXIT() vTaskDelete(NULL)
#define OS_MSLEEP(x) vTaskDelay(pdMS_TO_TICKS(x))
#define OS_MUTEX SemaphoreHandle_t
#define OS_MUTEX_INIT(ret, x) do { \
	*(x) = xSemaphoreCreateMutex(); \
	if (*(x) == NULL) ret = false; \
	else ret = true; \
} while (0)
#define OS_MUTEX_LOCK(x) xSemaphoreTake(*(x), portMAX_DELAY)
#define OS_MUTEX_UNLOCK(x) xSemaphoreGive(*(x))

#define OS_QUEUE QueueHandle_t
#define OS_QUEUE_INIT(ret, queue, count, item_size) do { \
	*queue = xQueueCreate(count, item_size); \
	ret = *queue == NULL ? false : true; \
} while (0)
#define OS_QUEUE_SEND(queue, buff) \
	(xQueueSend(*(queue), (void *)buff, (TickType_t)0) == pdPASS)
#define OS_QUEUE_RECV(queue, buff) \
	(xQueueReceive(*(queue), buff, portMAX_DELAY) == pdPASS)
#define OS_QUEUE_DEL(queue) vQueueDelete(*(queue))

static inline void os_get_fixed_id(uint8_t *id)
{
#if defined(CONFIG_IDF_TARGET_ESP8266)
	ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, id) != ESP_OK);
#else
	ESP_ERROR_CHECK(esp_read_mac(id, ESP_MAC_WIFI_STA) != ESP_OK);
#endif /* CONFIG_IDF_TARGET_ESP8266 */
}

#endif /* __linux__ */

#ifdef __cplusplus
}
#endif

#endif /* __OS_H_ */
