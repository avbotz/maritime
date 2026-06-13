#include "usb.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_config.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb, LOG_LEVEL_DBG);

#define USB_MARITIME_VID 0x2fe3
#define USB_MARITIME_PID 0x0004
#define USB_MARITIME_MAX_POWER 125
#define USB_MARITIME_ATTRIBUTES 0
#define USB_ECM_PORT 7777

static int ecm_server = -1;
static int ecm_client = -1;
static bool ecm_rx_overflow = false;
static size_t ecm_rx_pos;

USBD_DEVICE_DEFINE(maritime_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   USB_MARITIME_VID, USB_MARITIME_PID);

USBD_DESC_LANG_DEFINE(maritime_lang);
USBD_DESC_MANUFACTURER_DEFINE(maritime_mfr, "AVBotz");
USBD_DESC_PRODUCT_DEFINE(maritime_product, "Pico Maritime");

USBD_DESC_CONFIG_DEFINE(maritime_fs_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(maritime_hs_desc, "HS Configuration");

USBD_CONFIGURATION_DEFINE(maritime_fs_config, USB_MARITIME_ATTRIBUTES,
			  USB_MARITIME_MAX_POWER, &maritime_fs_desc);
USBD_CONFIGURATION_DEFINE(maritime_hs_config, USB_MARITIME_ATTRIBUTES,
			  USB_MARITIME_MAX_POWER, &maritime_hs_desc);

static int add_configuration(enum usbd_speed speed)
{
	struct usbd_config_node *config = (speed == USBD_SPEED_HS) ?
		&maritime_hs_config : &maritime_fs_config;
	int err;

	err = usbd_add_configuration(&maritime_usbd, speed, config);
	if (err) {
		LOG_ERR("Failed to add USB configuration (%d)", err);
		return err;
	}

	err = usbd_register_class(&maritime_usbd, "cdc_ecm_0", speed, 1);
	if (err) {
		LOG_ERR("Failed to register USB ECM class (%d)", err);
		return err;
	}

	return usbd_device_set_code_triple(&maritime_usbd, speed,
					   USB_BCC_MISCELLANEOUS, 0x02, 0x01);
}

static int listen_ecm(void)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(USB_ECM_PORT),
		.sin_addr = {
			.s_addr = htonl(INADDR_ANY),
		},
	};
	int server = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (server < 0) {
		LOG_ERR("ECM socket failed: %d", errno);
		return -1;
	}

	if (zsock_bind(server, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		LOG_ERR("ECM bind failed: %d", errno);
		zsock_close(server);
		return -1;
	}

	if (zsock_listen(server, 1) < 0) {
		LOG_ERR("ECM listen failed: %d", errno);
		zsock_close(server);
		return -1;
	}

	LOG_INF("Listening on ECM port %d", USB_ECM_PORT);
	return server;
}

static int ensure_ecm_server(void)
{
	while (ecm_server < 0) {
		ecm_server = listen_ecm();
		if (ecm_server < 0) {
			k_sleep(K_SECONDS(1));
		}
	}

	return ecm_server;
}

static int ensure_ecm_client(void)
{
	(void)ensure_ecm_server();

	while (ecm_client < 0) {
		ecm_client = zsock_accept(ecm_server, NULL, NULL);
		if (ecm_client < 0) {
			LOG_WRN("ECM accept failed: %d", errno);
			k_sleep(K_MSEC(100));
		}
	}

	return ecm_client;
}

static void close_ecm_client(void)
{
	if (ecm_client >= 0) {
		zsock_close(ecm_client);
		ecm_client = -1;
	}

	ecm_rx_pos = 0;
	ecm_rx_overflow = false;
}

int setup_usb(void)
{
	int err;

	err = usbd_add_descriptor(&maritime_usbd, &maritime_lang);
	if (err) {
		LOG_ERR("Failed to add USB language descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&maritime_usbd, &maritime_mfr);
	if (err) {
		LOG_ERR("Failed to add USB manufacturer descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&maritime_usbd, &maritime_product);
	if (err) {
		LOG_ERR("Failed to add USB product descriptor (%d)", err);
		return err;
	}

	if (USBD_SUPPORTS_HIGH_SPEED &&
	    usbd_caps_speed(&maritime_usbd) == USBD_SPEED_HS) {
		err = add_configuration(USBD_SPEED_HS);
		if (err) {
			return err;
		}
	}

	err = add_configuration(USBD_SPEED_FS);
	if (err) {
		return err;
	}

	err = usbd_init(&maritime_usbd);
	if (err) {
		LOG_ERR("Failed to initialize USB device (%d)", err);
		return err;
	}

	err = usbd_enable(&maritime_usbd);
	if (err) {
		LOG_ERR("Failed to enable USB device (%d)", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_NET_CONFIG_SETTINGS)) {
		(void)net_config_init_app(NULL, "Initializing USB ECM network");
	}

	return 0;
}

int usb_recv_ecm_packet(char *buf, size_t buf_size)
{
	if ((buf == NULL) || (buf_size == 0)) {
		return -EINVAL;
	}

	while (true) {
		char c;
		ssize_t len;

		(void)ensure_ecm_client();
		len = zsock_recv(ecm_client, &c, 1, 0);
		if (len <= 0) {
			close_ecm_client();
			continue;
		}

		if ((c == '\n') || (c == '\r')) {
			if (ecm_rx_overflow) {
				ecm_rx_overflow = false;
				ecm_rx_pos = 0;
				return -EMSGSIZE;
			}

			if (ecm_rx_pos == 0) {
				continue;
			}

			buf[ecm_rx_pos] = '\0';
			ecm_rx_pos = 0;
			return 0;
		}

		if (ecm_rx_overflow) {
			continue;
		}

		if (ecm_rx_pos < buf_size - 1) {
			buf[ecm_rx_pos++] = c;
		} else {
			ecm_rx_overflow = true;
			ecm_rx_pos = 0;
		}
	}
}

int usb_send_ecm_packet(const char *buf)
{
	size_t len;
	size_t sent = 0;

	if (buf == NULL) {
		return -EINVAL;
	}

	len = strlen(buf);
	while (sent < len) {
		ssize_t ret;

		(void)ensure_ecm_client();
		ret = zsock_send(ecm_client, buf + sent, len - sent, 0);
		if (ret <= 0) {
			close_ecm_client();
			return -EIO;
		}

		sent += ret;
	}

	return 0;
}
