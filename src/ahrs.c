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

#include "maritime/ahrs.h"


LOG_MODULE_REGISTER(uart_ahrs, LOG_LEVEL_INF);

#define UART1_DEVICE_NODE DT_NODELABEL(usart1)

static const struct device *uart_dev = DEVICE_DT_GET(UART1_DEVICE_NODE);

// Total bytes the message can store
#define MSG_SIZE 256

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

// Store orientation and angvel data from ahrs
static struct ahrs_data
{
    float yaw_rad;
    float pitch_rad;
    float roll_rad;
    float yaw_rad_s;
    float pitch_rad_s;
    float roll_rad_s;
};

static struct ahrs_data final_data;

/* queue to store up to 3 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(to_process_ahrs_msgq, MSG_SIZE, 3, 4);
K_MSGQ_DEFINE(processed_ahrs_msgq, sizeof(struct ahrs_data), 3, 4);

/* Every thread needs its own stack in RAM */
#define AHRS_PROCESS_STRING_STACK    1024
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
void uart_ahrs_cb(const struct device *dev, void *user_data)
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
            k_msgq_put(&to_process_ahrs_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

extern void ahrs_process_thread() 
{
    // Parse received ahrs messages
    char msg[MSG_SIZE];
    uint32_t ahrs_time = k_uptime_get_32();
    while (1)
    {
        if (k_msgq_get(&to_process_ahrs_msgq, &msg, K_NO_WAIT) == 0)
        {
            char *token;
            char *save_ptr;
            char delim[] = ", ";
            
            // printk("received wrz packet: %s\r\n", buf);
		    token = strtok_r(msg, delim, &save_ptr);  // timestamp
		    token = strtok_r(NULL, delim, &save_ptr);  // sensor ID

		    // If the message is not what we're looking for, skip it
		    if (strcmp(token, "Orientation") != 0)
		    {
		    	continue;
		    }

		    token = strtok_r(NULL, delim, &save_ptr);  // yaw
		    float yaw = deg_to_rad(atof(token));
		    
		    token = strtok_r(NULL, delim, &save_ptr);  // get pitch
		    float pitch = deg_to_rad(atof(token));
		    
		    token = strtok_r(NULL, delim, &save_ptr);  // get roll
		    float roll = deg_to_rad(atof(token));

		    token = strtok_r(NULL, delim, &save_ptr);  // get accuracy
		    float accuracy = atof(token);

		    float t = k_uptime_get_32();
		    float dt = (t - ahrs_time) / 1000.;
		    ahrs_time = t;

		    if (true)
		    {
		    	struct ahrs_data processed_data;
		    	processed_data.yaw_rad_s = (yaw - processed_data.yaw_rad) / dt;
		    	processed_data.pitch_rad_s = (pitch - processed_data.pitch_rad) / dt;
		    	processed_data.roll_rad_s = (roll - processed_data.roll_rad) / dt;
		        processed_data.yaw_rad = yaw; 
		        processed_data.pitch_rad = pitch; 
		        processed_data.roll_rad = roll;
                k_msgq_put(&processed_ahrs_msgq, &processed_data, K_NO_WAIT);
		    }
        }

        // Yield space to other processes
        k_yield();
    }
}

K_THREAD_DEFINE(ahrs_process_tid, AHRS_PROCESS_STRING_STACK,
            ahrs_process_thread, NULL, NULL, NULL,
            K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

// Relative x, y, z velocities
float ahrs_get_yaw()
{
    if (k_msgq_get(&processed_ahrs_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.yaw_rad;
}

float ahrs_get_pitch()
{
    if (k_msgq_get(&processed_ahrs_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.pitch_rad;
}

float ahrs_get_roll()
{
    if (k_msgq_get(&processed_ahrs_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
	return final_data.roll_rad;
}

float ahrs_get_yaw_rad_s()
{
    if (k_msgq_get(&processed_ahrs_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.yaw_rad_s;
}

float ahrs_get_pitch_rad_s()
{
    if (k_msgq_get(&processed_ahrs_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.pitch_rad_s;
}

float ahrs_get_roll_rad_s()
{
    if (k_msgq_get(&processed_ahrs_msgq, &final_data, K_NO_WAIT) == 0)
    {
        // Received a parsed message from the thread
    }
    return final_data.roll_rad_s;
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

void init_ahrs() 
{
    // uart_tx_msg("HELLO WORLD");
    // if (!device_is_ready(uart_dev)) {
    //     printk("UART model not found");
    //     return;
    // } else {
    //     printk("Found UART device");
    // }
    print_uart("J3\n\r"); //setting to NED coord system
    print_uart("s 3,100\n\r"); //setting to "orientation" at 100Hz

    uart_irq_callback_user_data_set(uart_dev, uart_ahrs_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    k_thread_start(ahrs_process_thread);
}