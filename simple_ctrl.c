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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "os.h"
#include "event_bus.h"
#include "crypto.h"
#include "class_id.h"
#include "store.h"
#include "ota.h"

static const char *TAG = "SIMPLE-CTRL";

#define SIMPLE_CTRL_INFO_ID_LENGTH	14
#define SIMPLE_CTRL_INFO_ID_DEFAULT	"00000000000000"
#define SIMPLE_CTRL_INFO_NAME_LENGTH	64
#define SIMPLE_CTRL_INFO_NAME_DEFAULT	"Unnamed"
#define SIMPLE_CTRL_INFO_PASSWD_LENGTH	16

#define DISCOVER_RECV_PORT		54542
#define DISCOVER_SEND_PORT		54543
#define DISCOVER_SAY			"HOOZZ?"
#define DISCOVER_RESPOND		"HOOZZ:"
#define DISCOVER_BUFFER_SIZE		(sizeof(DISCOVER_RESPOND) + 2 + SIMPLE_CTRL_INFO_ID_LENGTH + SIMPLE_CTRL_INFO_NAME_LENGTH)
#define DISCOVER_BROADCAST_NUM		40
#define DISCOVER_BROADCAST_ADDRESS	"255.255.255.255"

#define BODY_TCP_PORT			54542
#define BODY_TCP_MAX_ACCEPT		5
#define BODY_TCP_BUFFER_SIZE		256
#define BODY_TCP_TIMEOUT		30

#define NOTIFY_BUFFER_SIZE		128

#define CTRL_LOAD_TYPE_PING		0x00
#define CTRL_LOAD_TYPE_INFO		0x01
#define CTRL_LOAD_TYPE_REQUEST		0x02
#define CTRL_LOAD_TYPE_NOTIFY		0x03

#define CTRL_INFO_TYPE_GETNAME		0x00
#define CTRL_INFO_TYPE_SETNAME		0x01
#define CTRL_INFO_TYPE_GETCLASSID	0x02
#define CTRL_INFO_TYPE_SETPASSWD	0x03
#define CTRL_INFO_TYPE_FW_VERSION	0x04
#define CTRL_INFO_TYPE_FW_UPGRADE	0x05
#define CTRL_INFO_TYPE_FW_PROGRESS	0x06

#define CTRL_RETURN_OK			0x00
#define CTRL_RETURN_FAIL		0x01

#define CTRL_LOAD_MAGIC			"HOOZZ"
#define CTRL_LOAD_HEADER_SIZE		16

#define SOTRE_DEVNAME_FILE_NAME		"DEVICE-NAME"
#define SOTRE_PASSWD_FILE_NAME		"CRYPTO-PASSWD"

struct simple_ctrl_handle {
	uint8_t load_type;
	uint32_t load_len;
	struct crypto in_crypto;
	struct crypto out_crypto;
	char passwd_data[SIMPLE_CTRL_INFO_PASSWD_LENGTH];
	int passwd_len;
};

static int discover_socket = -1;
static OS_THREAD discover_handle;
static OS_MUTEX send_mutex;

static const char *default_name = SIMPLE_CTRL_INFO_NAME_DEFAULT;

static char info_name[SIMPLE_CTRL_INFO_NAME_LENGTH + 1] = SIMPLE_CTRL_INFO_NAME_DEFAULT;
static uint8_t info_class_id = CLASS_ID_UNKNOWN;
static char info_id[SIMPLE_CTRL_INFO_ID_LENGTH + 1] = SIMPLE_CTRL_INFO_ID_DEFAULT;

static char info_passwd[1 + SIMPLE_CTRL_INFO_PASSWD_LENGTH];
static uint8_t crypto_type = CRYPTO_TYPE_AES128CBC;

static void simple_ctrl_init_info_id(void)
{
	uint8_t id[6];

	os_get_fixed_id(id);
	sprintf(info_id, "%02x%02x%02x%02x%02x%02x%02x",
		id[0], id[1], id[2], id[3], id[4], id[5], 0);

	OS_LOGI(TAG, "ID(%s)", info_id);
}

static inline void simple_ctrl_discover_set_respond(char *buf, size_t buf_size)
{
	char has_pwd = info_passwd[0] ? '*' : '-';

	/*
	 * MAGIC(6bytes)
	 * CLASS(2bytes)
	 * CRYPTO(2byte)
	 * HAS_PW(1byte)
	 * ID(14bytes)
	 * NAME(nbytes)
	 */
	snprintf(buf, buf_size, "%s%02x%02x%c%s%s", DISCOVER_RESPOND,
		 info_class_id, crypto_type, has_pwd, info_id, info_name);
}

static void simple_ctrl_discover_handle(void)
{
	char buffer[DISCOVER_BUFFER_SIZE];
	struct sockaddr_in addr_in;
	socklen_t addr_size = sizeof(addr_in);
	int recv_len;
	uint8_t count;
	int ret;
	int operate = 1;
	struct sockaddr_in address;

	/* Create UDP */
	discover_socket = socket(AF_INET, SOCK_DGRAM, 0);
	OS_ERROR_CHECK(discover_socket < 0);

	/* Allow binding address reuse */
	setsockopt(discover_socket, SOL_SOCKET, SO_REUSEADDR, &operate, sizeof(operate));
	/* Enable broadcast */
	setsockopt(discover_socket, SOL_SOCKET, SO_BROADCAST, &operate, sizeof(operate));

	/* Bind port */
	address.sin_family = AF_INET;
	address.sin_port = htons(DISCOVER_RECV_PORT);
	address.sin_addr.s_addr = INADDR_ANY;
	ret = bind(discover_socket, (struct sockaddr *)&address, sizeof (address));
	OS_ERROR_CHECK(ret < 0);

	/* Full address */
	addr_in.sin_family = AF_INET;
	addr_in.sin_port = htons(DISCOVER_SEND_PORT);
	ret = inet_pton(AF_INET, DISCOVER_BROADCAST_ADDRESS,
			(void *)&addr_in.sin_addr);
	OS_ERROR_CHECK(ret < 0);
	count = DISCOVER_BROADCAST_NUM;
	while (count--) {
		simple_ctrl_discover_set_respond(buffer, sizeof(buffer));
		sendto(discover_socket, buffer, strlen(buffer), 0,
		       (struct sockaddr*)&addr_in, addr_size);
		OS_MSLEEP(100);
	}
	OS_LOGI(TAG, "online broadcast has been sent");

	while (1) {
		/*
		 * Wait data
		 * Note:
		 *   On Linux, data starts being received after the `bind()` call,
		 *   so here you can receive the packet you sent yourself.
		 */
		recv_len = recvfrom(discover_socket, &buffer,
				    DISCOVER_BUFFER_SIZE - 1, 0,
				    (struct sockaddr*)&addr_in,
				    &addr_size);
		if (recv_len >= 0) {
			buffer[recv_len] = '\0';

			/* If the flag is discover, then respond */
			if(strcmp(buffer, DISCOVER_SAY))
				continue;

			OS_LOGI(TAG, "recvfrom: %s:%d",
				 inet_ntoa(addr_in.sin_addr),
				 ntohs(addr_in.sin_port));

			OS_LOGI(TAG, "recvdata: %s (%dbytes)", buffer, recv_len);

			simple_ctrl_discover_set_respond(buffer, sizeof(buffer));
			sendto(discover_socket, buffer, strlen(buffer), 0,
			       (struct sockaddr*)&addr_in, addr_size);
			OS_LOGI(TAG, "discover has responded");
		} else {
			OS_LOGI(TAG, "recv_len < 0, exit discover");
			break;
		}
	}
}

static OS_THREAD_RET simple_ctrl_discover_task(void *pvParameters)
{
	OS_LOGI(TAG, "Discover Running");

	while (1) {
		OS_LOGI(TAG, "Wait network ready...");
		OS_THREAD_SLEEP(&discover_handle);

		OS_LOGI(TAG, "Start discover...");
		simple_ctrl_discover_handle();
	}

	OS_THREAD_EXIT();
}

static bool simple_ctrl_notify_callback(struct event_bus_msg *msg)
{
	switch (msg->type) {
	case EVENT_BUS_WIFI_CONNECTED:
		OS_THREAD_WAKEUP(&discover_handle);
		break;
	case EVENT_BUS_WIFI_DISCONNECTED:
		if (discover_socket >= 0)
			close(discover_socket);
		break;
	case EVENT_BUS_STOP_SMART_CONFIG:
		OS_LOGI(TAG, "Reset name and password...");
		memset(info_name, 0, sizeof(info_name));
		strncpy(info_name, default_name, sizeof(info_name));
		store_save(SOTRE_DEVNAME_FILE_NAME, info_name,
			   sizeof(info_name));
		memset(info_passwd, 0, sizeof(info_passwd));
		store_save(SOTRE_PASSWD_FILE_NAME, info_passwd,
			   sizeof(info_passwd));
		break;
	}

	return false;
}

static void simple_ctrl_handle_reset(struct simple_ctrl_handle *handle)
{
	memset(handle, 0, sizeof(struct simple_ctrl_handle));
	memcpy(handle->passwd_data, info_passwd + 1, info_passwd[0]);
	handle->passwd_len = info_passwd[0];
	crypto_init(&handle->in_crypto, crypto_type, handle->passwd_data, handle->passwd_len);
	crypto_init(&handle->out_crypto, crypto_type, handle->passwd_data, handle->passwd_len);
}

static int simple_ctrl_none_request(char *buffer, int buf_offs, int vaild_size, int buff_size)
{
	/* Do nothing */
	return 0;
}

static int (*request_handle)(char *buffer, int buf_offs, int vaild_size, int buff_size)
	= simple_ctrl_none_request;

void simple_ctrl_request_register(int (*request)(char *buffer, int buf_offs, int vaild_size, int buff_size))
{
	request_handle = request;
}

void simple_ctrl_set_class_id(uint8_t new_id)
{
	info_class_id = new_id;
}

void simple_ctrl_set_name(const char *new_name)
{
	size_t len = strlen(new_name);
	size_t copy_size;

	copy_size = SIMPLE_CTRL_INFO_NAME_LENGTH;
	copy_size = copy_size < len ? copy_size : len;

	memcpy(info_name, new_name, copy_size);
	info_name[copy_size] = '\0';
}

static int simple_ctrl_ping(char *buffer, int buf_offs, int vaild_size, int buff_size)
{
	int ret = -1;
	uint8_t ping_interval;

	if (vaild_size != 1) {
		OS_LOGE(TAG, "Ping command is incorrect");
		return -1;
	}

	ping_interval = buffer[buf_offs];
	OS_LOGD(TAG, "Remote ping interval: %u\n", ping_interval);

	buffer[buf_offs] = CTRL_RETURN_OK;
	ret = 1;

	return ret;
}

static int simple_ctrl_info(char *buffer, int buf_offs, int vaild_size, int buff_size)
{
	size_t len;
	int ret = -1;
	int upg_ret;
	uint8_t info_cmd;

	if (vaild_size < 1) {
		OS_LOGE(TAG, "Information command is incorrect");
		return -1;
	}

	info_cmd = buffer[buf_offs];

	switch (info_cmd) {
	case CTRL_INFO_TYPE_GETNAME:
		len = strlen(info_name);
		if ((len + 2) > (buff_size - buf_offs)) {
			OS_LOGE(TAG, "Buffer does not have enough space");
			return -1;
		}
		buffer[buf_offs + 1] = CTRL_RETURN_OK;
		memcpy(buffer + buf_offs + 2, info_name, len);
		ret = 2 + (int)len;
		break;
	case CTRL_INFO_TYPE_SETNAME:
		len = vaild_size - 1;
		if (len > SIMPLE_CTRL_INFO_NAME_LENGTH) {
			buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
			ret = 2;
		} else {
			memcpy(info_name, buffer + buf_offs + 1, len);
			info_name[len] = '\0';
			ret = store_save(SOTRE_DEVNAME_FILE_NAME, info_name,
					 sizeof(info_name));
			if (ret) {
				buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
				ret = 2;
				OS_LOGE(TAG, "Device name saving failed");
			} else {
				buffer[buf_offs + 1] = CTRL_RETURN_OK;
				ret = 2;
				OS_LOGI(TAG, "Device name has been changed");
			}
		}
		break;
	case CTRL_INFO_TYPE_GETCLASSID:
		ret = 3;
		if (ret > (buff_size - buf_offs)) {
			OS_LOGE(TAG, "Buffer does not have enough space");
			return -1;
		}
		buffer[buf_offs + 1] = CTRL_RETURN_OK;
		buffer[buf_offs + 2] = info_class_id;
		break;
	case CTRL_INFO_TYPE_SETPASSWD:
		len = vaild_size - 1;
		if (len > SIMPLE_CTRL_INFO_PASSWD_LENGTH) {
			buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
			ret = 2;
			OS_LOGE(TAG, "Password is too long");
		} else {
			memcpy(info_passwd + 1, buffer + buf_offs + 1, len);
			info_passwd[0] = (char)len;
			ret = store_save(SOTRE_PASSWD_FILE_NAME, info_passwd,
					 sizeof(info_passwd));
			if (ret) {
				buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
				ret = 2;
				OS_LOGE(TAG, "Access password saving failed");
			} else {
				buffer[buf_offs + 1] = CTRL_RETURN_OK;
				ret = 2;
				OS_LOGI(TAG, "Access password has been changed");
			}
		}
		break;
	case CTRL_INFO_TYPE_FW_VERSION:
		/* Response: [cmd, ret, version...] (the git hash, set via
		 * simple_ctrl_ota_enable) */
		len = strlen(ota_get_version());
		if ((len + 2) > (buff_size - buf_offs)) {
			OS_LOGE(TAG, "Buffer does not have enough space");
			return -1;
		}
		buffer[buf_offs + 1] = CTRL_RETURN_OK;
		memcpy(buffer + buf_offs + 2, ota_get_version(), len);
		ret = 2 + (int)len;
		break;
	case CTRL_INFO_TYPE_FW_UPGRADE:
		/* Request: [cmd, url...]. The url selects the transport by
		 * scheme; for "hoozz" it is "hoozz://<key>/<port>". ota_upgrade()
		 * copies the url into its own storage (so this buffer can be
		 * reused right away) and only validates it and starts the
		 * transfer task (no blocking here).
		 * Response: [cmd, ret] */
		len = vaild_size - 1;
		if (len <= 0) {
			buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
			ret = 2;
			break;
		}
		upg_ret = ota_upgrade(buffer + buf_offs + 1, (uint32_t)len);
		buffer[buf_offs + 1] = upg_ret == 0 ? CTRL_RETURN_OK : CTRL_RETURN_FAIL;
		ret = 2;
		break;
	case CTRL_INFO_TYPE_FW_PROGRESS:
		/* Response: [cmd, ret, progress(0-100)] */
		ret = 3;
		if (ret > (buff_size - buf_offs)) {
			OS_LOGE(TAG, "Buffer does not have enough space");
			return -1;
		}
		buffer[buf_offs + 1] = CTRL_RETURN_OK;
		buffer[buf_offs + 2] = ota_get_progress();
		break;
	default:
		buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
		ret = 2;
		break;
	}

	return ret;
}

static int simple_ctrl_handle_pad(struct simple_ctrl_handle *handle, char *buffer, int buff_size)
{
	int ret;
	int vaild_size;

	ret = crypto_de(&handle->in_crypto, buffer, handle->load_len, buff_size);
	if (ret < 0)
		return ret;
	buffer += handle->in_crypto.head_reserve;
	buff_size -= handle->in_crypto.head_reserve + handle->in_crypto.tail_reserve;
	vaild_size = ret - handle->in_crypto.head_reserve - handle->in_crypto.tail_reserve;

	if ((vaild_size < CTRL_LOAD_HEADER_SIZE) ||
	    memcmp(CTRL_LOAD_MAGIC, buffer, sizeof(CTRL_LOAD_MAGIC))) {
		OS_LOGE(TAG, "Magic is incorrect");
		return -1;
	}
	/* |--Magic(6bytes)--|--Reserved(6bytes)--|--Data size(4bytes)--| */
	vaild_size = (buffer[12] << 0) |
		     (buffer[13] << 8) |
		     (buffer[14] << 16) |
		     (buffer[15] << 24);

	switch (handle->load_type) {
	case CTRL_LOAD_TYPE_PING:
		ret = simple_ctrl_ping(buffer, CTRL_LOAD_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_LOAD_TYPE_INFO:
		ret = simple_ctrl_info(buffer, CTRL_LOAD_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_LOAD_TYPE_REQUEST:
		ret = request_handle(buffer, CTRL_LOAD_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_LOAD_TYPE_NOTIFY:
		ret = 0;
		break;
	}

	if (ret > 0) {
		buffer[12] = (uint8_t)((ret >> 0) & 0xff);
		buffer[13] = (uint8_t)((ret >> 8) & 0xff);
		buffer[14] = (uint8_t)((ret >> 16) & 0xff);
		buffer[15] = (uint8_t)((ret >> 24) & 0xff);
		ret += CTRL_LOAD_HEADER_SIZE;
		ret += handle->out_crypto.head_reserve + handle->out_crypto.tail_reserve;
	}

	return ret;
}

static volatile int fds[BODY_TCP_MAX_ACCEPT];
/* Last activity (ms) of each fd, for the per-connection idle timeout */
static uint32_t fd_active_ms[BODY_TCP_MAX_ACCEPT];

void simple_ctrl_notify(char *buffer, int size)
{
	int index;
	int ret;
	uint8_t load_info[5];
	char enbuf[NOTIFY_BUFFER_SIZE];
	int ensize;
	int load_size;
	struct crypto handle;
	char *data_ptr;

	crypto_init(&handle, crypto_type, info_passwd + 1, info_passwd[0]);

	ensize = handle.head_reserve + CTRL_LOAD_HEADER_SIZE + size + handle.tail_reserve;
	if (ensize > NOTIFY_BUFFER_SIZE) {
		OS_LOGE(TAG, "Not enough space in the enbuf");
		return;
	}

	data_ptr = enbuf + handle.head_reserve;
	memset(data_ptr, 0, CTRL_LOAD_HEADER_SIZE);
	memcpy(data_ptr, CTRL_LOAD_MAGIC, sizeof(CTRL_LOAD_MAGIC));

	/* playload size */
	data_ptr[12] = (uint8_t)((size >> 0) & 0xff);
	data_ptr[13] = (uint8_t)((size >> 8) & 0xff);
	data_ptr[14] = (uint8_t)((size >> 16) & 0xff);
	data_ptr[15] = (uint8_t)((size >> 24) & 0xff);

	/* playload data */
	memcpy(data_ptr + CTRL_LOAD_HEADER_SIZE, buffer, size);

	ret = crypto_en(&handle, enbuf, ensize, sizeof(enbuf));
	if (ret < 0) {
		OS_LOGE(TAG, "Encryption failed");
		return;
	}
	load_size = ret;

	load_info[0] = CTRL_LOAD_TYPE_NOTIFY;
	load_info[1] = (uint8_t)((load_size >> 0) & 0xff);
	load_info[2] = (uint8_t)((load_size >> 8) & 0xff);
	load_info[3] = (uint8_t)((load_size >> 16) & 0xff);
	load_info[4] = (uint8_t)((load_size >> 24) & 0xff);

	OS_LOGD(TAG, "Load size: %d", load_size);

	for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
		if (fds[index] != -1) {
			OS_MUTEX_LOCK(&send_mutex);

			/* Send info */
			ret = send(fds[index], load_info, sizeof(load_info), 0);
			if (ret != sizeof(load_info)) {
				OS_LOGE(TAG, "Send load info fail: %d", ret);
				OS_MUTEX_UNLOCK(&send_mutex);
				continue;
			}

			/* Send load */
			ret = send(fds[index], enbuf, load_size, 0);
			if (ret != load_size) {
				OS_LOGE(TAG, "Send load fail: %d", ret);
				OS_MUTEX_UNLOCK(&send_mutex);
				continue;
			}

			OS_MUTEX_UNLOCK(&send_mutex);

			OS_LOGD(TAG, "Notification completed (%d)", fds[index]);
		}
	}
}

static void simple_ctrl_body_handle(void)
{
	int operate = 1;
	int ret;
	int sel_ret;
	static int body_socket;
	struct sockaddr_in address;
	struct sockaddr_in addr_in;
	socklen_t addr_size = sizeof(addr_in);
	int index;
	int max_fd;
	int newconn;
	fd_set readfds;
	char buffer[BODY_TCP_BUFFER_SIZE];
	struct simple_ctrl_handle handle;
	uint32_t count;
	uint32_t msg_deadline;
	struct timeval timeout;
	struct timeval rcv_timeout;

	/* Create UDP */
	body_socket = socket(AF_INET, SOCK_STREAM, 0);
	OS_ERROR_CHECK(body_socket < 0);

	/* Allow binding address reuse */
	setsockopt(body_socket, SOL_SOCKET, SO_REUSEADDR, &operate, sizeof(operate));

	/* Bind port */
	address.sin_family = AF_INET;
	address.sin_port = htons(BODY_TCP_PORT);
	address.sin_addr.s_addr = INADDR_ANY;
	ret = bind(body_socket, (struct sockaddr *)&address, sizeof (address));
	OS_ERROR_CHECK(ret < 0);

	/*
	 * Start listen
	 *
	 * TCP listen backlog = 1, then the maximum waiting length is 1 + 1 = 2
	 * (backlog represents the maximum length of the waiting queue)
	 */
	ret = listen(body_socket, 1);
	OS_ERROR_CHECK(ret < 0);

	/* Initialize all to invalid */
	for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
		fds[index] = -1;
		fd_active_ms[index] = 0;
	}

	while (1) {
		/* Set select */
		FD_ZERO(&readfds);
		FD_SET(body_socket, &readfds);
		max_fd = body_socket;
		/* Find all valid sockets */
		for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
			if (fds[index] != -1) {
				FD_SET(fds[index], &readfds);
				if (fds[index] > max_fd)
					max_fd = fds[index];
			}
		}

		timeout.tv_sec = BODY_TCP_TIMEOUT;
		timeout.tv_usec = 0;

		/* Wait (OK > 0; Timeout == 0; Failed < 0) */
		sel_ret = select(max_fd + 1, &readfds, NULL, NULL, &timeout);
		OS_ERROR_CHECK(sel_ret < 0);
		OS_LOGD(TAG, "Select resule: %d", sel_ret);

		/* Need to accept or receive? */
		if (FD_ISSET(body_socket, &readfds)) {
			newconn = accept(body_socket, (struct sockaddr *)&addr_in, &addr_size);
			if (newconn >= 0) {
				OS_LOGI(TAG, "%s:%d has been connected (%d)", inet_ntoa(addr_in.sin_addr),
					 ntohs(addr_in.sin_port), newconn);
			} else {
				OS_LOGI(TAG, "newconn < 0, drop it");
				continue;
			}

			for (index = 0; (index < BODY_TCP_MAX_ACCEPT) && (fds[index] != -1); index++);
			if (index < BODY_TCP_MAX_ACCEPT) {
				/* Set read timeout. Use a dedicated timeval: the select()
				 * timeout above is mutated by select(), so reusing it here
				 * could hand the new socket a near-zero SO_RCVTIMEO. */
				rcv_timeout.tv_sec = BODY_TCP_TIMEOUT;
				rcv_timeout.tv_usec = 0;
				setsockopt(newconn, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout,
						sizeof(rcv_timeout));
				fds[index] = newconn;
				fd_active_ms[index] = os_get_time_ms();
			} else {
				OS_LOGW(TAG, "The connection has reached the set maximum");
				close(newconn);
				OS_LOGI(TAG, "Closed (%d)", newconn);
			}
		}

		for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
			if (fds[index] != -1) {
				if (FD_ISSET(fds[index], &readfds)) {
					/* Read control fields */
					ret = recv(fds[index], buffer, 5, 0);
					if (ret > 0) {
						fd_active_ms[index] = os_get_time_ms();
						if (ret < 5) {
							OS_LOGW(TAG, "Control field size mismatch, closed (%d)", fds[index]);
							goto closefd;
						}

						simple_ctrl_handle_reset(&handle);

						/* Fill parameter */
						handle.load_type = buffer[0];
						handle.load_len = (buffer[1] << 0) |
								  (buffer[2] << 8) |
								  (buffer[3] << 16) |
								  (buffer[4] << 24);
						OS_LOGI(TAG, "(%d) load_type:%02x, load_len:%u", fds[index],
							handle.load_type, (unsigned int)handle.load_len);

						if (handle.load_len > (uint32_t)sizeof(buffer)) {
							OS_LOGW(TAG, "The data is too long, closed (%d)", fds[index]);
							goto closefd;
						}

						count = 0;
						msg_deadline = os_get_time_ms() + BODY_TCP_TIMEOUT * 1000U;
						while (count < handle.load_len) {
							/* Read pack data. Append at the current offset:
							 * TCP does not preserve message boundaries, so a
							 * single message may arrive in several chunks;
							 * writing every chunk at buffer[0] would let the
							 * last chunk clobber the earlier ones and break
							 * the decryption ("Magic is incorrect") */
							ret = recv(fds[index], buffer + count,
								(int)(handle.load_len - count), 0);
							if (ret == 0) {
								/* The peer closed the connection */
								OS_LOGI(TAG, "Peer closed mid-message, disconnected (%d)",
									fds[index]);
								goto closefd;
							}
							if (ret < 0) {
								if (errno == EINTR ||
										errno == EAGAIN || errno == EWOULDBLOCK) {
									/* A spurious readable event with no data
									 * yet (or an interrupted read): retry the
									 * read, but only while the whole message
									 * is still within its budget. recv() is
									 * already paced by SO_RCVTIMEO, so a peer
									 * that stalls mid-message hits the
									 * deadline on its first EAGAIN and is
									 * dropped instead of holding this fd --
									 * and the other connections -- forever */
									if ((int32_t)(os_get_time_ms() - msg_deadline) < 0) {
										OS_LOGW(TAG, "Read would block, retrying (%d)",
											fds[index]);
										continue;
									}
									OS_LOGW(TAG, "Message stalled past the deadline, disconnected (%d)",
										fds[index]);
									goto closefd;
								}
								OS_LOGE(TAG, "Read error, disconnected (%d): %d",
									fds[index], errno);
								goto closefd;
							}
							OS_LOGD(TAG, "(%d) Expect: %u; Result: %d", fds[index],
								(unsigned int)handle.load_len, ret);

							count += ret;
							OS_LOGD(TAG, "(%d) Handle [%u/%u]", fds[index],
								(unsigned int)count, (unsigned int)handle.load_len);
						}

						/* Handle data */
						ret = simple_ctrl_handle_pad(&handle, buffer, sizeof(buffer));
						if (ret > 0) {
							uint8_t load_info[5];
							int vaild_size = ret;

							/* Encrypto */
							ret = crypto_en(&handle.out_crypto, buffer, vaild_size, sizeof(buffer));
							if (ret < 0) {
								OS_LOGE(TAG, "(%d) Encryption failed: %d", fds[index], ret);
								goto closefd;
							}
							vaild_size = ret;

							load_info[0] = handle.load_type;
							load_info[1] = (uint8_t)((vaild_size >> 0) & 0xff);
							load_info[2] = (uint8_t)((vaild_size >> 8) & 0xff);
							load_info[3] = (uint8_t)((vaild_size >> 16) & 0xff);
							load_info[4] = (uint8_t)((vaild_size >> 24) & 0xff);

							OS_MUTEX_LOCK(&send_mutex);

							/* Send info */
							ret = send(fds[index], load_info, sizeof(load_info), 0);
							if (ret != sizeof(load_info)) {
								OS_LOGE(TAG, "(%d) Send load info fail: %d", fds[index], ret);
								OS_MUTEX_UNLOCK(&send_mutex);
								goto closefd;
							}
							OS_LOGD(TAG, "(%d) Send load info: %d", fds[index], ret);

							/* Send data */
							ret = send(fds[index], buffer, vaild_size, 0);
							if (ret != vaild_size) {
								OS_LOGE(TAG, "(%d) Send load data fail: %d", fds[index], ret);
								OS_MUTEX_UNLOCK(&send_mutex);
								goto closefd;
							}
							OS_LOGD(TAG, "(%d) Send load data: %d", fds[index], ret);

							OS_MUTEX_UNLOCK(&send_mutex);
						} else if (ret < 0) {
							OS_LOGI(TAG, "Handle exception, disconnected (%d)", fds[index]);
							goto closefd;
						}

						OS_LOGI(TAG, "(%d) Handle done", fds[index]);
					} else if (ret == 0) {
						/* The peer closed the connection */
						OS_LOGI(TAG, "Peer closed, disconnected (%d)", fds[index]);
						goto closefd;
					} else if (errno == EAGAIN || errno == EWOULDBLOCK) {
						/* A spurious readable event with no data yet: the
						 * peer is still there, retry on the next pass instead
						 * of dropping the connection */
						OS_LOGW(TAG, "Read would block, retrying (%d)", fds[index]);
						continue;
					} else if (errno == EINTR) {
						continue;
					} else {
						OS_LOGE(TAG, "Read error, disconnected (%d): %d",
							fds[index], errno);
						goto closefd;
					}
				} else if ((int32_t)(os_get_time_ms() - fd_active_ms[index]) >=
						(int32_t)(BODY_TCP_TIMEOUT * 1000U)) {
					/* Per-connection idle timeout: no data has arrived for
					 * BODY_TCP_TIMEOUT seconds and nothing is pending (if
					 * there were, the branch above would have read it and
					 * the timer would refresh). The old code closed every
					 * connection when the global select() timed out, but
					 * that timeout never fired while any client was active,
					 * so dead connections held their slots forever and new
					 * clients were refused ("reached the set maximum")
					 */
					OS_LOGI(TAG, "Idle timeout, disconnected (%d)", fds[index]);
					goto closefd;
				}
				/* Normal exit: the fd stays open (only the error paths and
				 * the idle timeout above close it) */
				continue;
closefd:
				close(fds[index]);
				fds[index] = -1;
			}
		}
	}

	close(body_socket);
}

static OS_THREAD_RET simple_ctrl_body_task(void *pvParameters)
{
	OS_LOGI(TAG, "Ctrl Body Running");

	while (1) {
		simple_ctrl_body_handle();
		OS_MSLEEP(1000);
	}

	OS_THREAD_EXIT();
}

void simple_ctrl_ota_enable(const char *version,
			    const struct ota_transport *transports,
			    uint8_t count, const struct ota_flash *flash,
			    void (*pre_restart)(void))
{
	/*
	 * The version is stored for the FW_VERSION command, the transports
	 * for the FW_UPGRADE command and the flash backend decides where
	 * the downloaded image goes; ota_enable() also runs the backend's
	 * first-boot validation, so this must be called after the backend's
	 * storage is initialized (ESP8266: after nvs_flash_init()).
	 */
	ota_enable(version, transports, count, flash, pre_restart);
}

void simple_ctrl_init(const char *name)
{
	bool ret;

	if (name) {
		default_name = name;
		simple_ctrl_set_name(default_name);
	}

	simple_ctrl_init_info_id();

	store_load(SOTRE_DEVNAME_FILE_NAME, info_name, sizeof(info_name));
	store_load(SOTRE_PASSWD_FILE_NAME, info_passwd, sizeof(info_passwd));

	OS_THREAD_CREATE(ret, &discover_handle, simple_ctrl_discover_task, NULL,
			 "simple_ctrl_discover_task", 2000);
	OS_ERROR_CHECK(ret != true);

	OS_MUTEX_INIT(ret, &send_mutex);
	OS_ERROR_CHECK(ret != true);

	OS_THREAD_CREATE(ret, NULL, &simple_ctrl_body_task, NULL,
			 "simple_ctrl_body_task", 3000);
	OS_ERROR_CHECK(ret != true);

	event_bus_register(simple_ctrl_notify_callback);
}
