#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <zephyr/timing/timing.h>
#include <zephyr/sys/ring_buffer.h>

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

// #include <pubsub/pubsub.h>

#include "dvl.h"
#include "util.h"

LOG_MODULE_REGISTER(test_dvl, LOG_LEVEL_INF);

#define UART5_DEVICE_NODE DT_NODELABEL(uart5)

static const struct device *uart_device = DEVICE_DT_GET(UART5_DEVICE_NODE);

// upper bound of length of a single packet
#define DVL_MSG_MAX_SZ 512

RING_BUF_DECLARE_STATIC(ring_buf_tx, DVL_MSG_MAX_SZ);

struct dvl_packet_s {
    uint8_t packet[DVL_MSG_MAX_SZ];
};


K_MSGQ_DEFINE(dvl_data_msgq, sizeof(struct dvl_data_s), 1, 4);

K_MSGQ_DEFINE(dvl_packet_msgq, sizeof(struct dvl_packet_s), 2, 4);
// PUBSUB_TOPIC_DEFINE(dvl_msg_topic, sizeof(struct dvl_packet_s));
// PUBSUB_SUBSCRIBER_DEFINE(dvl_msg_topic, dvl_sub, 0);

static uint8_t buf[DVL_MSG_MAX_SZ];
uint16_t msg_sz = 0;

static const uint8_t lookup[256] = {
    0x00U,0x07U,0x0EU,0x09U,0x1CU,0x1BU,0x12U,0x15U,
    0x38U,0x3FU,0x36U,0x31U,0x24U,0x23U,0x2AU,0x2DU,
    0x70U,0x77U,0x7EU,0x79U,0x6CU,0x6BU,0x62U,0x65U,
    0x48U,0x4FU,0x46U,0x41U,0x54U,0x53U,0x5AU,0x5DU,
    0xE0U,0xE7U,0xEEU,0xE9U,0xFCU,0xFBU,0xF2U,0xF5U,
    0xD8U,0xDFU,0xD6U,0xD1U,0xC4U,0xC3U,0xCAU,0xCDU,
    0x90U,0x97U,0x9EU,0x99U,0x8CU,0x8BU,0x82U,0x85U,
    0xA8U,0xAFU,0xA6U,0xA1U,0xB4U,0xB3U,0xBAU,0xBDU,
    0xC7U,0xC0U,0xC9U,0xCEU,0xDBU,0xDCU,0xD5U,0xD2U,
    0xFFU,0xF8U,0xF1U,0xF6U,0xE3U,0xE4U,0xEDU,0xEAU,
    0xB7U,0xB0U,0xB9U,0xBEU,0xABU,0xACU,0xA5U,0xA2U,
    0x8FU,0x88U,0x81U,0x86U,0x93U,0x94U,0x9DU,0x9AU,
    0x27U,0x20U,0x29U,0x2EU,0x3BU,0x3CU,0x35U,0x32U,
    0x1FU,0x18U,0x11U,0x16U,0x03U,0x04U,0x0DU,0x0AU,
    0x57U,0x50U,0x59U,0x5EU,0x4BU,0x4CU,0x45U,0x42U,
    0x6FU,0x68U,0x61U,0x66U,0x73U,0x74U,0x7DU,0x7AU,
    0x89U,0x8EU,0x87U,0x80U,0x95U,0x92U,0x9BU,0x9CU,
    0xB1U,0xB6U,0xBFU,0xB8U,0xADU,0xAAU,0xA3U,0xA4U,
    0xF9U,0xFEU,0xF7U,0xF0U,0xE5U,0xE2U,0xEBU,0xECU,
    0xC1U,0xC6U,0xCFU,0xC8U,0xDDU,0xDAU,0xD3U,0xD4U,
    0x69U,0x6EU,0x67U,0x60U,0x75U,0x72U,0x7BU,0x7CU,
    0x51U,0x56U,0x5FU,0x58U,0x4DU,0x4AU,0x43U,0x44U,
    0x19U,0x1EU,0x17U,0x10U,0x05U,0x02U,0x0BU,0x0CU,
    0x21U,0x26U,0x2FU,0x28U,0x3DU,0x3AU,0x33U,0x34U,
    0x4EU,0x49U,0x40U,0x47U,0x52U,0x55U,0x5CU,0x5BU,
    0x76U,0x71U,0x78U,0x7FU,0x6AU,0x6DU,0x64U,0x63U,
    0x3EU,0x39U,0x30U,0x37U,0x22U,0x25U,0x2CU,0x2BU,
    0x06U,0x01U,0x08U,0x0FU,0x1AU,0x1DU,0x14U,0x13U,
    0xAEU,0xA9U,0xA0U,0xA7U,0xB2U,0xB5U,0xBCU,0xBBU,
    0x96U,0x91U,0x98U,0x9FU,0x8AU,0x8DU,0x84U,0x83U,
    0xDEU,0xD9U,0xD0U,0xD7U,0xC2U,0xC5U,0xCCU,0xCBU,
    0xE6U,0xE1U,0xE8U,0xEFU,0xFAU,0xFDU,0xF4U,0xF3U,
};

uint8_t dvl_crc8(uint8_t *packet) {
    uint8_t checksum = 0;
    while (true) {
        uint8_t byte = *packet;
        if (byte == '*' || byte == '\0') {
            break;
        }
        checksum = lookup[byte ^ checksum];
        ++packet;
    }
    return checksum;
}

bool in_packet = false;
int w = 0; 
uint64_t last_seen = 0, diff = 0;
static void uart_irq_callback(const struct device *dev, void *data) {
    ARG_UNUSED(dev);
    ARG_UNUSED(data);

    struct dvl_packet_s dvl_packet; 

    while (uart_irq_update(uart_device) && uart_irq_is_pending(uart_device)) {

        if (uart_irq_tx_complete(uart_device) && ring_buf_is_empty(&ring_buf_tx)){
            uart_irq_tx_disable(uart_device);
        }

        uint8_t tx_byte;
        if (uart_irq_tx_ready(uart_device)){
            ring_buf_get(&ring_buf_tx, &tx_byte, 1);
            uart_fifo_fill(uart_device, &tx_byte, 1);
        }

        uint8_t rx_byte;

        // uint32_t chars;

        if (uart_irq_rx_ready(uart_device)) {
            uart_fifo_read(uart_device, &rx_byte, 1); 

            // printk("before sz: %d, buf %c %c %c\n", msg_sz, buf[0], buf[1], buf[2]);
            if (!in_packet && msg_sz == 3) {
                buf[0] = buf[1];
                buf[1] = buf[2];
                --msg_sz;
            }
            buf[msg_sz] = rx_byte;
            ++msg_sz;

            if (msg_sz == 3 && buf[0] == 'w' && buf[1] == 'r' && buf[2] == 'z') {
                uint64_t cur = timing_counter_get();
                diff = timing_cycles_get(&last_seen, &cur);
                last_seen = cur;
                ++w;
                in_packet = true;
            }

            // printk("after sz: %d, buf %c %c %c\n\n", msg_sz, buf[0], buf[1], buf[2]);

            if (msg_sz == DVL_MSG_MAX_SZ) {
                msg_sz = 0;
                in_packet = false;
                continue;
            }

            if (in_packet && (rx_byte == '\r' || rx_byte == '\n')) {
                buf[msg_sz] = '\0';
                memset(&(dvl_packet.packet), 0, DVL_MSG_MAX_SZ);
                memcpy(&(dvl_packet.packet), buf, msg_sz + 1);
                // pubsub_publish(&dvl_msg_topic, 0, &dvl_packet);
                // LOG_DBG("Publishing: %s", dvl_packet.packet);
                k_msgq_put(&dvl_packet_msgq, &dvl_packet, K_NO_WAIT);

                in_packet = false;
                msg_sz = 0;
                buf[0] = buf[1] = buf[2] = '0';
            }

            // LOG_DBG("%c", rx_byte);

            /*
            if (rx_byte == 'w') {
                if (seen_start) {
                    LOG_DBG("See new start byte without previous end byte");
                }
                seen_start = true;
            }
            if ((rx_byte == '\n' || rx_byte == '\r') && seen_start) {
                seen_start = false;
                process_frame();   
            } 
            */

            // echo back byte
            // ring_buf_put(&ring_buf_tx, &rx_byte, 1);
            // uart_irq_tx_enable(uart_device);
        }
    }

}

int consume_packet(uint8_t **ptr, uint8_t delim, uint8_t total) {
    uint8_t seen = 0;
    while (seen != total) {
        uint8_t byte = **ptr;
        if (byte == '\0') {
            return -1;
        }
        if (byte == delim) {
            ++seen;
        }
        ++(*ptr);
    }
    return 0;
}

int parse_dvl_packet(struct dvl_packet_s *dvl_packet, struct dvl_data_s *dvl_data) {
    uint64_t a = timing_counter_get();
    uint8_t *start = dvl_packet->packet;
    char *end;
    int ret;

    ret = consume_packet(&start, ',', 1);
    if (ret < 0) return -1;
    
    dvl_data->velocity_x = strtof(start, &end);
    if (dvl_data->velocity_x == 0) return -1;
    start = ++end;

    dvl_data->velocity_y = strtof(start, &end);
    if (dvl_data->velocity_y == 0) return -1;
    start = ++end;

    dvl_data->velocity_z = strtof(start, &end);
    if (dvl_data->velocity_z == 0) return -1;
    start = ++end;

    if (*start == 'n') return -1;
    start += 2;

    dvl_data->altitude = strtof(start, &end);
    if (dvl_data->altitude == 0) return -1;
    start = ++end;

    ret = consume_packet(&start, ',', 1);
    if (ret < 0) return -1;

    ret = consume_packet(&start, ';', 8);
    if (ret < 0) return -1;

    ret = consume_packet(&start, ',', 3);
    if (ret < 0) return -1;
    
    dvl_data->time_since_last_ms = strtof(start, &end); 
    if (dvl_data->time_since_last_ms == 0) return -1;
    start = ++end;

    dvl_data->health = *start == '1';
    start += 2;

    uint8_t checksum = (uint8_t)strtol(start, &end, 16);
    if (dvl_crc8(dvl_packet->packet) != checksum) {
        return -1;
    }

    uint64_t b = timing_counter_get();
    uint64_t ns = timing_cycles_to_ns(timing_cycles_get(&a, &b));
    // printk("Took parse %llu ns\n", ns);
    LOG_DBG("Vx: %f, Vy: %f, Vz: %f, Altitude: %f", 
            dvl_data->velocity_x,
            dvl_data->velocity_y,
            dvl_data->velocity_z,
            dvl_data->altitude);
    return 0;
}

int total = 0, recv = 0;
void dvl_packet_handle_thread(void) {
    struct dvl_packet_s dvl_packet;
    struct dvl_data_s dvl_data;

    int ret;

    while (true) {

        uint64_t a = timing_counter_get();
        ret = k_msgq_get(&dvl_packet_msgq, &dvl_packet, K_NO_WAIT);
        if (ret != 0)
        {
            k_yield();
            continue;
        }
        // ret = pubsub_poll(&dvl_sub, K_FOREVER);
        uint64_t a1 = timing_counter_get();
        if (ret < 0) {
            if (ret == -ENOMSG) {
                // no msg
            } else if (ret == -EAGAIN) {
                // timed out
            }
            continue;
        } else {
            // pubsub_copy(&dvl_sub, &dvl_packet);
            LOG_DBG("Got packet %s", dvl_packet.packet);
        }
        ret = parse_dvl_packet(&dvl_packet, &dvl_data);
        ++recv;
        if (ret == 0) {
            ++total;
            // printk("recv: %d, ok: %d, w: %d, timing: %llu\n", recv, total, w, timing_cycles_to_ns(diff));
            while (k_msgq_put(&dvl_data_msgq, &dvl_data, K_NO_WAIT) != 0) {
                k_msgq_put(&dvl_data_msgq, &dvl_data, K_NO_WAIT);
            }
            uint64_t b = timing_counter_get();
            uint64_t ns = timing_cycles_to_ns(timing_cycles_get(&a, &b));
            uint64_t ns1 = timing_cycles_to_ns(timing_cycles_get(&a, &a1));
            // printk("Took kmsq get %llu ns\n", ns1);
            // printk("Took whole %llu ns\n\n", ns);
        }
        k_yield();
    }
}

void setup_dvl(void) {
    LOG_DBG("Setting up DVL");
    uart_irq_callback_user_data_set(uart_device, uart_irq_callback, NULL);
    uart_irq_rx_enable(uart_device);
}

K_THREAD_DEFINE(dvl_thread_id, 4096,
                dvl_packet_handle_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
