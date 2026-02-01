#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include <zephyr/timing/timing.h>

#include "ahrs.h"
#include "util.h"

LOG_MODULE_REGISTER(uart_ahrs, LOG_LEVEL_INF);

static const struct device *uart_device = DEVICE_DT_GET(DT_NODELABEL(uart1));

K_MSGQ_DEFINE(ahrs_data_msgq, sizeof(struct ahrs_data_s), 1, 4);

RING_BUF_DECLARE(ring_buf_tx, MSG_SZ);

unsigned char ucRxBuffer[250];
unsigned char ucRxCnt = 0;

struct SAngle
{
    short Angle[3];
    short T;
} stcAngle;

void process_frame() {
    uint32_t dt_us;
    struct ahrs_data_s ahrs_data;

    struct ahrs_data_s prev_sample = {
        .ts_us = time_us(),
        .yaw   = 0,
        .pitch = 0,
        .roll  = 0
    };

    while (true) {
        for (int i=0; i<11; i++) {
		// printk("%02X ", ucRxBuffer[i]);
	}

        // Witmotion ahrs uses ENU coordinate system, convert to NED system in rads
        ahrs_data.yaw = -deg_to_rad(stcAngle.Angle[2] / 32768. * 180);
        ahrs_data.pitch = -deg_to_rad(stcAngle.Angle[1] / 32768. * 180);
        ahrs_data.roll = deg_to_rad(stcAngle.Angle[0] / 32768. * 180 + 90);

        dt_us = time_us() - prev_sample.ts_us;

        ahrs_data.ang_vel_yaw = (ahrs_data.yaw - prev_sample.yaw) / dt_us * 1e6f;
        ahrs_data.ang_vel_pitch = (ahrs_data.pitch - prev_sample.pitch) / dt_us * 1e6f;
        ahrs_data.ang_vel_roll = (ahrs_data.roll - prev_sample.roll) / dt_us * 1e6f;
        ahrs_data.ts_us = time_us();

        while (k_msgq_put(&ahrs_data_msgq, &ahrs_data, K_NO_WAIT) != 0) {
            k_msgq_put(&ahrs_data_msgq, &ahrs_data, K_NO_WAIT);
        }

        // LOG_DBG("Timestamp: %u", tt);
        prev_sample = ahrs_data;

        LOG_DBG("Dt: %u\n\rYaw: %f, Pitch: %f, Roll: %f\n\rVyaw: %f, Vpitch: %f, Vroll: %f",
                dt_us,
                (double)ahrs_data.yaw,
                (double)ahrs_data.pitch,
                (double)ahrs_data.roll,
                (double)ahrs_data.ang_vel_yaw,
                (double)ahrs_data.ang_vel_pitch,
                (double)ahrs_data.ang_vel_roll);

        // Publish new data every 20 msec
        k_sleep(K_MSEC(1000 / AHRS_OUTPUT_RATE));
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

    printk("UART IRQ Callback\n");

    ucRxBuffer[5] = 0x12;

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

            // Discard bytes until the start of the packet
            ucRxBuffer[ucRxCnt++] = rx_byte;
            // if (ucRxBuffer[0] != 0x55)
            // {
            //     ucRxCnt = 0;
            //     return;
            // }

            // Wait until all 11 bytes of the packet are received
            if (ucRxCnt < 11)
            {
                return;
            }

        	// // Store the angular data
            // // Second byte indicates the type of data
            // // Third - tenth bytes contain the data
            // // Eleventh byte is checksum, ignored for now
        	// switch (ucRxBuffer[1])
        	// {
        	//     // case 0x50:  memcpy(&stcTime,&ucRxBuffer[2],8);break;
        	//     // case 0x51:  memcpy(&stcAcc,&ucRxBuffer[2],8);break;
        	//     // case 0x52:  memcpy(&stcGyro,&ucRxBuffer[2],8);break;
        	//     case 0x53:  memcpy(&stcAngle,&ucRxBuffer[2],8);break;
        	//     // case 0x54:  memcpy(&stcMag,&ucRxBuffer[2],8);break;
        	//     // case 0x55:  memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
        	//     // case 0x56:  memcpy(&stcPress,&ucRxBuffer[2],8);break;
        	//     // case 0x57:  memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
        	//     // case 0x58:  memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
            //     // case 0x59:  memcpy(&stcQuater,&ucRxBuffer[2],8);break;
        	//     // case 0x5a:  memcpy(&stcSN,&ucRxBuffer[2],8);break;
        	// }
        	ucRxCnt = 0;
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

    LOG_DBG("Finished setting up AHRS");

    return 0;
}

K_THREAD_DEFINE(ahrs_rx_frame_handle_thread_id, 1024,
                process_frame, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
