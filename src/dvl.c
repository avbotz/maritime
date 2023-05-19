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


LOG_MODULE_REGISTER(uart_dvl, LOG_LEVEL_INF);

#define UART5_DEVICE_NODE DT_NODELABEL(uart5)

static const struct device *uart_device = DEVICE_DT_GET(UART5_DEVICE_NODE);

// Total bytes the message can store
#define MSG_SZ 512

RING_BUF_DECLARE(ring_buf_tx, MSG_SZ);
RING_BUF_DECLARE(ring_buf_rx, MSG_SZ);

uint8_t buf[MSG_SZ];

// keep track if we've seen the first 'w' byte
bool seen_start = false;

// Store data from dvl
static struct dvl_data
{
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float altitude;
} dvl_data;

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

void init_dvl() 
{
    // uart_tx_msg("HELLO WORLD");
    // if (!device_is_ready(uart_device)) {
    //     printk("UART model not found");
    //     return;
    // } else {
    //     printk("Found UART device");
    // }

    uart_irq_rx_enable(uart_device);
    uart_irq_callback_user_data_set(uart_device, uart_irq_callback, NULL);
}

void process_velocity() 
{
    char *token;
    
    // printk("received wrz packet: %s\r\n", buf);

    token = strtok(buf, ",");   // wrz

    token = strtok(NULL, ",");  // velocity x
    float velocity_x = atof(token);

    token = strtok(NULL, ","); // velocity y 
    float velocity_y = atof(token);


    token = strtok(NULL, ","); // velocity z
    float velocity_z = atof(token);

    token = strtok(NULL, ","); // valid
    bool valid = strcmp(token, "y") == 0;
    
    token = strtok(NULL, ","); // altitude
    float altitude = atof(token);

    token = strtok(NULL, ","); // figure of merit

    // covariance matrix
    for (int i = 0; i < 8; ++i) {
        token = strtok(NULL, ";");
    }
    token = strtok(NULL, ",");

    token = strtok(NULL, ","); // time of validity
    
    token = strtok(NULL, ","); // time of transmission
    
    token = strtok(NULL, ","); // time since last report
    float time_ms = atof(token);

    token = strtok(NULL, "*"); // status
    bool status = strcmp(token, "1") == 0;

    token = strtok(NULL, "\0"); // checksum
    uint16_t checksum = strtol(token, NULL, 16);

    // LOG_INF("\n\rvelocity x: %f\n\ry:%f\n\rz:%f\n\rvalid: %d\n\raltitude: %f\n\rtime since last: %f\n\rstatus: %d\n\rchecksum: 0x%X\n\r",
    //         velocity_x, velocity_y, velocity_z, valid, altitude, time_ms, status, checksum);

    if (valid)
    {
    	dvl_data.velocity_x = velocity_x;
    	dvl_data.velocity_y = velocity_y;
    	dvl_data.velocity_z = velocity_z;
    	dvl_data.altitude = altitude;
    }
}

// Relative x, y, z velocities
float dvl_get_velocity_x()
{
    return dvl_data.velocity_x;
}

float dvl_get_velocity_y()
{
    return dvl_data.velocity_y;
}

float dvl_get_velocity_z()
{
	return dvl_data.velocity_z;
}

float dvl_get_altitude()
{
    return dvl_data.altitude;
}


void process_frame() 
{
    uint8_t rx_byte;
    uint32_t rb_len;

    // Ensure ring buf starts at frame start rx_byte
    while (true) {
        ring_buf_peek(&ring_buf_rx, &rx_byte, 1);
        if (rx_byte != 'w') {
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
        if (rx_byte == '\n' || rx_byte == '\r') {
            break;
        }
        *(buf + len) = rx_byte;
        ++len;
    }
    
    buf[len] = 0; // for ending string, can remove
    
    printk("received total frame: %s\r\n", buf);
    if (buf[1] == 'r') {
        if (buf[2] == 'z') {
            process_velocity();
        }
    }
    

}

void uart_tx_msg(char *msg)
{
    ring_buf_put(&ring_buf_tx, msg, strlen(msg));
    uart_irq_tx_enable(uart_device);
}


void uart_irq_callback(const struct device *dev, void *data) 
{
    ARG_UNUSED(dev);
    ARG_UNUSED(data);


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
            uart_fifo_read(uart_device, &rx_byte, 1);
            ring_buf_put(&ring_buf_rx, &rx_byte, 1);

            if (rx_byte == 'w') {
                if (seen_start) {
                    printk("See new start byte without previous end byte");
                }
                seen_start = true;
            }
            if ((rx_byte == '\n' || rx_byte == '\r') && seen_start) {
                seen_start = false;
                process_frame();   
            } 

            // echo back byte
            // ring_buf_put(&ring_buf_tx, &rx_byte, 1);
            // uart_irq_tx_enable(uart_device);
        }
    }

}