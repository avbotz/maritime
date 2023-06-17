/* @author: Kush Nayak */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "maritime/dvl.h"

LOG_MODULE_REGISTER(uart_dvl, LOG_LEVEL_INF);

#define UART5_DEVICE_NODE DT_NODELABEL(uart5)

static const struct device *uart_dev = DEVICE_DT_GET(UART5_DEVICE_NODE);

// Total bytes the message can store
#define MSG_SIZE 256

RING_BUF_DECLARE(ring_buf_tx, MSG_SIZE);
RING_BUF_DECLARE(ring_buf_rx, MSG_SIZE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

// Store data from dvl
static struct dvl_data
{
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float altitude;
};

struct dvl_data final_data;

/* queue to store up to 3 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(to_process_dvl_msgq, MSG_SIZE, 3, 4);
K_MSGQ_DEFINE(processed_dvl_msgq, sizeof(struct dvl_data), 3, 4);

/* Every thread needs its own stack in RAM */
#define DVL_PROCESS_STRING_STACK    1024
/* Define and initialize the new thread */

/*
int ring_buf_get_byte(struct ring_buf *buf, uint8_t *byte) {
    uint32_t rb_len = ring_buf_get(buf, byte, 1);
    if (rb_len == 1) {
        return 0;
    } else {
        return -1;
    }
}
*/

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue. We use this to read messages coming in 
 * from the DVL.
 */
void uart_dvl_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    while (uart_irq_rx_ready(uart_dev)) 
    {
        uart_fifo_read(uart_dev, &c, 1);

        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            /* terminate string */
            rx_buf[rx_buf_pos] = '\0';

            /* if queue is full, message is silently dropped */
            k_msgq_put(&to_process_dvl_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

extern void dvl_process_thread() 
{
    // Parse received dvl messages
    char msg[MSG_SIZE];
    while (1)
    {
        if (k_msgq_get(&to_process_dvl_msgq, &msg, K_NO_WAIT) == 0)
        {
            char *token;
            char *save_ptr;
            char delim[] = ",";
            
            // printk("received wrz packet: %s\r\n", buf);

            token = strtok_r(msg, delim, &save_ptr);   // wrz

            // If there's no 'wrz', then don't read it
            if (strcmp(token, "wrz") != 0)
            {
                continue;
            }

            token = strtok_r(NULL, delim, &save_ptr);  // velocity x
            float velocity_x = atof(token);

            token = strtok_r(NULL, delim, &save_ptr); // velocity y 
            float velocity_y = atof(token);


            token = strtok_r(NULL, delim, &save_ptr); // velocity z
            float velocity_z = atof(token);

            token = strtok_r(NULL, delim, &save_ptr); // valid
            bool valid = strcmp(token, "y") == 0;
            
            token = strtok_r(NULL, delim, &save_ptr); // altitude
            float altitude = atof(token);

            token = strtok_r(NULL, delim, &save_ptr); // figure of merit

            // covariance matrix
            for (int i = 0; i < 8; ++i) {
                token = strtok_r(NULL, ";", &save_ptr);
            }
            token = strtok_r(NULL, delim, &save_ptr);

            token = strtok_r(NULL, delim, &save_ptr); // time of validity
            
            token = strtok_r(NULL, delim, &save_ptr); // time of transmission
            
            token = strtok_r(NULL, delim, &save_ptr); // time since last report
            float time_ms = atof(token);

            token = strtok_r(NULL, "*", &save_ptr); // status
            bool status = strcmp(token, "1") == 0;

            token = strtok_r(NULL, "\0"); // checksum
            uint16_t checksum = strtol(token, NULL, 16);

            // LOG_INF("\n\rvelocity x: %f\n\ry:%f\n\rz:%f\n\rvalid: %d\n\raltitude: %f\n\rtime since last: %f\n\rstatus: %d\n\rchecksum: 0x%X\n\r",
            //         velocity_x, velocity_y, velocity_z, valid, altitude, time_ms, status, checksum);

            if (valid)
            {
                struct dvl_data processed_data;
                processed_data.velocity_x = velocity_x;
                processed_data.velocity_y = velocity_y;
                processed_data.velocity_z = velocity_z;
                processed_data.altitude = altitude;
                k_msgq_put(&processed_dvl_msgq, &processed_data, K_NO_WAIT);
            }
        }

        // Yield space to other processes
        k_yield();
    }
}

K_THREAD_DEFINE(dvl_process_tid, DVL_PROCESS_STRING_STACK,
            dvl_process_thread, NULL, NULL, NULL,
            K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

// Relative x, y, z velocities
float dvl_get_velocity_x()
{
    if (k_msgq_get(&processed_dvl_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.velocity_x;
}

float dvl_get_velocity_y()
{
    if (k_msgq_get(&processed_dvl_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.velocity_y;
}

float dvl_get_velocity_z()
{
    if (k_msgq_get(&processed_dvl_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
	return final_data.velocity_z;
}

float dvl_get_altitude()
{
    if (k_msgq_get(&processed_dvl_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.altitude;
}

void uart_tx_msg(char *msg)
{
    ring_buf_put(&ring_buf_tx, msg, strlen(msg));
    uart_irq_tx_enable(uart_dev);
}

void init_dvl() 
{
    // uart_tx_msg("HELLO WORLD");
    // if (!device_is_ready(uart_dev)) {
    //     printk("UART model not found");
    //     return;
    // } else {
    //     printk("Found UART device");
    // }

    uart_irq_callback_user_data_set(uart_dev, uart_dvl_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    k_thread_start(dvl_process_thread);
}