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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <malloc.h>
#include "os.h"
#include "event_bus.h"

static const char *TAG = "EVENT-BUS";

/* Maximum number of events */
#define MAX_EVENT_NUM		10

static OS_QUEUE queue;
static OS_MUTEX register_mutex;

struct event_notify {
	event_notify_callback call;
	struct event_notify *next;
};

static struct event_notify *notify_link_head = NULL;
static struct event_notify *notify_link_tail = NULL;

void event_bus_send(struct event_bus_msg *msg)
{
	if (!OS_QUEUE_SEND(&queue, msg))
		OS_LOGI(TAG, "Send failed");
}

void event_bus_register(event_notify_callback callback)
{
	struct event_notify *notify;

	notify = (struct event_notify *)malloc(sizeof(struct event_notify));
	OS_ERROR_CHECK(notify == NULL);

	notify->call = callback;
	notify->next = NULL;

	OS_MUTEX_LOCK(&register_mutex);
	if (notify_link_head) {
		notify_link_tail->next = notify;
		notify_link_tail = notify_link_tail->next;
	} else {
		notify_link_head = notify;
		notify_link_tail = notify_link_head;
	}
	OS_MUTEX_UNLOCK(&register_mutex);

	// printf("Register [%p]\n", callback);
}

static OS_THREAD_RET event_bus_task(void *pvParameters)
{
	struct event_bus_msg msg;
	struct event_notify *iter;
	bool processed;

	while(1) {
		if(OS_QUEUE_RECV(&queue, &msg)) {
			iter = notify_link_head;
			while (iter) {
				processed = iter->call(&msg);
				if (processed)
					break;
				// printf("Call [%p]\n", iter->call);
				iter = iter->next;
			}
		}
	}

	OS_QUEUE_DEL(&queue);
	OS_THREAD_EXIT();
}

void event_bus_init(void)
{
	bool ret;

	OS_LOGI(TAG, "Event Bus Init");

	/* Create message queue */
	OS_QUEUE_INIT(ret, &queue, MAX_EVENT_NUM, sizeof(struct event_bus_msg));
	OS_ERROR_CHECK(ret != true);

	OS_MUTEX_INIT(ret, &register_mutex);
	OS_ERROR_CHECK(ret != true);

	/* Start task */
	OS_THREAD_CREATE(ret, NULL, &event_bus_task, NULL,
			 "event_bus_task", 4096);
	OS_ERROR_CHECK(ret != true);
}
