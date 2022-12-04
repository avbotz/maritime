//Script to report angular velocity, attitude, state, linear velocity, and depth mode
#include <zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <drivers/gpio.h>
#include <string.h>

#include <ahrs.h>
#include <dvl.h>
#include <thruster.h>
#include <pressure_sensor.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 4096

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);

		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
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
//For printing doubles, not characters
void print_double_over_uart(double num){
  char arr[sizeof(num)];
  memcpy(arr, &num, sizeof(num));
  for(int i = 0; i < sizeof(arr); i++){
    uart_poll_out(uart_dev, arr[i]);
  }
}

void main(void)
{
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	//print_uart("Hello! I'm your echo bot.\r\n");
	//print_uart("Tell me something and press enter:\r\n");

  struct ahrs_sample ahrs_sample;
  struct dvl_sample dvl_sample;

	/* indefinitely wait for input from the user */
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
    //Sample IMU on AHRS
    int imu_ret = sample_imu(dev, &ahrs_sample);
  	if (imu_ret != 0) {
  		LOG_ERR("Error sampling IMU: %d", imu_ret);
  	}

    //Sample DVL
    int dvl_ret = sample_dvl(dev, &dvl_sample);
  	if (dvl_ret != 0) {
  		LOG_ERR("Error sampling DVL: %d", dvl_ret);
  	}

    if(tx_buf == 'a'){
      //Print attitude over uart
      print_double_over_uart(ahrs_sample->rotation);
      print_uart('\n');
      print_uart();
    } else if(tx_buf == 'b'){
      //Print angular velocity over uart
      print_double_over_uart(ahrs_sample->gyro[0]);
      print_uart(' ');
      print_double_over_uart(ahrs_sample->gyro[1]);
      print_uart(' ');
      print_double_over_uart(ahrs_sample->gyro[2]);
      print_uart('\n');
    } else if(tx_buf == 'd'){
      //Print depth mode over uart
    } else if (tx_buf == 'p'){
      //Print position over uart
    } else if (tx_buf == 'v'){
      //Print linear velocity over uart
      print_double_over_uart(dvl_sample->changeX);
      print_uart(' ');
      print_double_over_uart(dvl_sample->changeY);
      print_uart(' ');
      print_double_over_uart(dvl_sample->changeZ);
      print_uart('\n');
    }
	}
}
