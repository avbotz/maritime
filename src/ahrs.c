#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <zephyr/timing/timing.h>

#include "ahrs.h"
#include "util.h"

LOG_MODULE_REGISTER(uart_ahrs, LOG_LEVEL_INF);

#define UART2_DEVICE_NODE DT_NODELABEL(usart1)

static const struct device *uart_device = DEVICE_DT_GET(UART2_DEVICE_NODE);

K_MSGQ_DEFINE(ahrs_data_msgq, sizeof(struct ahrs_data_s), 1, 4);

K_SEM_DEFINE(ahrs_rx_frame_buf_sem, 0, 1);

// Total bytes the message can store
#define MSG_SZ 512

// #include <pubsub/pubsub.h>

#define NS_ELAPSED(time_a, time_b) (timing_cycles_to_ns(timing_cycles_get(&time_a, &time_b)))

RING_BUF_DECLARE_STATIC(ring_buf_tx, MSG_SZ);
RING_BUF_DECLARE_STATIC(ring_buf_rx, MSG_SZ);

static uint8_t buf[MSG_SZ];

timing_t t = 0, a, b, c;

bool seen_start = false;

const uint8_t start_byte = 'w';


const float TO_RAD = M_PI / 180.0f;
const float TO_RAD_S = TO_RAD * 1e6f;

uint32_t tt; 

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
struct EulerAngles ToEulerAngles(struct Quaternion q) {
    struct EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

uint32_t prev_time = 0;
struct ahrs_data_s prev_sample = {
    .yaw   = 0,
    .pitch = 0,
    .roll  = 0
};

int process(char *buf, struct ahrs_data_s *ahrs_data) {
    LOG_DBG("Received update: %s", buf);
    char *str = buf;
    char *end;
    struct Quaternion quaternion;
    struct EulerAngles angles;
    
    uint32_t cur_time = time_us();

    uint32_t timestamp = (uint32_t)strtol(str, &end, 10);
    if (timestamp == 0) return -1;
    str = ++end;
    tt = timestamp;

    uint8_t sensor_id = (uint8_t)strtol(str, &end, 10); 
    if (sensor_id != 15 || sensor_id == 0) return -1;
    str = ++end;

    quaternion.x = strtof(str, &end);
    if (quaternion.x == 0) return -1;
    str = ++end;

    quaternion.y = strtof(str, &end);
    if (quaternion.y == 0) return -1;
    str = ++end;

    quaternion.z = strtof(str, &end);
    if (quaternion.z == 0) return -1;
    str = ++end;

    quaternion.w = strtof(str, &end);
    if (quaternion.w == 0) return -1;
    str = ++end;

    float accuracy = strtof(str, &end);

    angles = ToEulerAngles(quaternion);

    ahrs_data->yaw = -angles.yaw;
    ahrs_data->pitch = angles.roll;
    ahrs_data->roll = angles.pitch;

    // Check for nan
    if (ahrs_data->yaw != ahrs_data->yaw ||
        ahrs_data->pitch != ahrs_data->pitch ||
	ahrs_data->roll != ahrs_data->roll)
	return -1;

    ahrs_data->ang_vel_yaw   = (ahrs_data->yaw - prev_sample.yaw) / (cur_time - prev_time) * 1e6f;
    ahrs_data->ang_vel_pitch = (ahrs_data->pitch - prev_sample.pitch) / (cur_time - prev_time) * 1e6f;
    ahrs_data->ang_vel_roll  = (ahrs_data->roll - prev_sample.roll) / (cur_time - prev_time) * 1e6f;

    if (fabs(ahrs_data->ang_vel_yaw) > 3. ||
        fabs(ahrs_data->ang_vel_pitch) > 3. ||
	fabs(ahrs_data->ang_vel_roll) > 3.)
        return -1;

    LOG_DBG("Timestamp: %u\n\rYaw: %f, Pitch: %f, Roll: %f\n\rdt: %u, Vyaw: %f, Vpitch: %f, Vroll: %f",
            timestamp,
            ahrs_data->yaw,
            ahrs_data->pitch,
            ahrs_data->roll,
            cur_time - prev_time,
            ahrs_data->ang_vel_yaw,
            ahrs_data->ang_vel_pitch,
            ahrs_data->ang_vel_roll);

    timing_t cur = timing_counter_get();
    uint64_t el = NS_ELAPSED(t, cur);
    LOG_DBG("%llu ns", el);
    t = cur;
    prev_time = cur_time;
    memcpy(&prev_sample, ahrs_data, sizeof(struct ahrs_data_s));

    return 0;
}


void process_frame() {
    while (true) {
        if (k_sem_take(&ahrs_rx_frame_buf_sem, K_MSEC(100)) != 0) {
            LOG_DBG("AHRS frame rx buffer sem exceeded 100 msec timeout");
            k_yield();
            continue;
        } 

        uint8_t rx_byte;
        // uint32_t rb_len;

        uint64_t c0 = timing_counter_get();
        // Ensure ring buf starts at frame start
        while (true) {
            ring_buf_peek(&ring_buf_rx, &rx_byte, 1);
            if (rx_byte != start_byte) {
                ring_buf_get(&ring_buf_rx, &rx_byte, 1);
                // printk("misaligned frame, continuing to retrieve data %c\r\n", rx_byte);
                // LOG_INF("Misaligned frame with char %c\r\n", rx_byte);
            } else {
                break;
            }
        }

        // Get ring buf data till frame end byte
        // TODO: implement checksum before continuing processing
        uint32_t len = 0;
        while (true) {
            if (len == MSG_SZ) {
                return;
            }

            ring_buf_get(&ring_buf_rx, &rx_byte, 1);

            if (rx_byte == 'w') {
                continue;
            }
            if (rx_byte == '\r') {
                break;
            }

            *(buf + len) = rx_byte;
            ++len;
        }
        
        buf[len] = '\0'; 
        
        // printk("received total frame: %s\n", buf);
        
        c = timing_counter_get();
        LOG_DBG("Claim time: %llu ns", NS_ELAPSED(c0, c));
         // LOG_DBG("Claim time (with sem): %llu ns", NS_ELAPSED(b, c));

        struct ahrs_data_s ahrs_data;
        
        int ret = process(buf, &ahrs_data);

        if (ret == 0) {
            while (k_msgq_put(&ahrs_data_msgq, &ahrs_data, K_NO_WAIT) != 0) {
                k_msgq_put(&ahrs_data_msgq, &ahrs_data, K_NO_WAIT);
            }
        }
        // LOG_DBG("Timestamp: %u", tt);
        k_yield();
    }
}

void uart_tx_msg(char *msg){
    LOG_DBG("Sending AHRS MSG: %s", msg);
    ring_buf_put(&ring_buf_tx, msg, strlen(msg));
    uart_irq_tx_enable(uart_device);
}


static void uart_irq_callback(const struct device *dev, void *data) {
    ARG_UNUSED(dev);
    ARG_UNUSED(data);

    // uint8_t rx[3] = "000";
    // uint8_t pos = -1;
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
        if (uart_irq_rx_ready(uart_device)) {
            // printk("%c", rx_byte);
            uart_fifo_read(uart_device, &rx_byte, 1);
            ring_buf_put(&ring_buf_rx, &rx_byte, 1);

            if (rx_byte == '\r') {
                if (!seen_start) {
                    seen_start = true;
                    a = timing_counter_get();
                    ring_buf_put(&ring_buf_rx, &start_byte, 1);
                } else {
                    seen_start = false;
                    b = timing_counter_get();
                    k_sem_give(&ahrs_rx_frame_buf_sem);
                    // LOG_DBG("Get: %llu ns", NS_ELAPSED(a, b));
                }
            } 

            // echo back byte
            // ring_buf_put(&ring_buf_tx, &rx_byte, 1);
            // uart_irq_tx_enable(uart_device);
        }
    }

}

int setup_ahrs() {
    LOG_DBG("Setting up AHRS");
    if (!device_is_ready(uart_device)) {
        return -1;
    }

    uart_irq_callback_user_data_set(uart_device, uart_irq_callback, NULL);
    uart_irq_rx_enable(uart_device);

    uart_tx_msg("X");
    k_sleep(K_MSEC(1000));
    uart_tx_msg("M1\r");
    uart_tx_msg("m0");
    uart_tx_msg("V0");
    uart_tx_msg("s,15," STR(AHRS_OUTPUT_RATE) "\r");

    prev_time = time_us();

    LOG_DBG("Finished setting up AHRS");

    return 0; 
}


K_THREAD_DEFINE(ahrs_rx_frame_handle_thread_id, 4096,
                process_frame, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
