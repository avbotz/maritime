#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <math.h>

#include "ahrs.h"

LOG_MODULE_REGISTER(ahrs, LOG_LEVEL_DBG);

/*
 * WT901 (WitMotion) standard UART protocol: 11-byte frames of
 * 0x55, <type>, 8 data bytes (little-endian int16), checksum (low byte of the
 * sum of the first 10 bytes). We only use the 6-DOF frames (accel + gyro) and
 * run our own fusion, so the magnetometer (0x54) and onboard angle (0x53)
 * frames are ignored.
 */
#define WIT_FRAME_SIZE 11
#define WIT_HEADER     0x55
#define WIT_TYPE_ACCEL 0x51
#define WIT_TYPE_GYRO  0x52

/* Raw int16 full scale maps to +/-16 g and +/-2000 deg/s (WT901 manual) */
#define WIT_ACCEL_SCALE (16.0f / 32768.0f)
#define WIT_GYRO_SCALE  (2000.0f / 32768.0f)

#define DEG_TO_RAD (3.14159265f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265f)

/*
 * Mahony filter gains. KP sets how aggressively roll/pitch are pulled toward
 * the accelerometer's gravity vector, KI compensates slow gyro bias.
 */
#define MAHONY_KP 1.0f
#define MAHONY_KI 0.05f

static const struct device *wit_uart = DEVICE_DT_GET(DT_NODELABEL(uart1));

RING_BUF_DECLARE(wit_rx_ring, 256);
static K_SEM_DEFINE(wit_rx_sem, 0, 1);

static K_MUTEX_DEFINE(quat_lock);
/* Orientation quaternion {w, x, y, z}, protected by quat_lock */
static float q0 = 1.0f, q1, q2, q3;

/* Latest accel sample in g, only touched by the AHRS thread */
static float accel[3];
static bool have_accel;

/* Mahony integral feedback, only touched by the AHRS thread */
static float integral_fb[3];

static void wit_rx_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uint8_t buf[32];

	uart_irq_update(dev);
	while (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int len = uart_fifo_read(dev, buf, sizeof(buf));
			if (len > 0) {
				ring_buf_put(&wit_rx_ring, buf, len);
				k_sem_give(&wit_rx_sem);
			}
		}
		uart_irq_update(dev);
	}
}

static void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	float norm = sqrtf(ax * ax + ay * ay + az * az);

	/*
	 * Only trust the accelerometer when it is measuring roughly 1 g;
	 * during hard thruster accelerations it is not a gravity reference.
	 */
	if (norm > 0.5f && norm < 1.5f) {
		ax /= norm;
		ay /= norm;
		az /= norm;

		/* Gravity direction estimated from the current quaternion */
		float vx = 2.0f * (q1 * q3 - q0 * q2);
		float vy = 2.0f * (q0 * q1 + q2 * q3);
		float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		/* Error is the cross product of measured and estimated gravity */
		float ex = ay * vz - az * vy;
		float ey = az * vx - ax * vz;
		float ez = ax * vy - ay * vx;

		integral_fb[0] += MAHONY_KI * ex * dt;
		integral_fb[1] += MAHONY_KI * ey * dt;
		integral_fb[2] += MAHONY_KI * ez * dt;

		gx += MAHONY_KP * ex + integral_fb[0];
		gy += MAHONY_KP * ey + integral_fb[1];
		gz += MAHONY_KP * ez + integral_fb[2];
	}

	/* Integrate quaternion rate: q_dot = 0.5 * q * (0, gx, gy, gz) */
	float half_dt = 0.5f * dt;
	float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
	float dq1 = (q0 * gx + q2 * gz - q3 * gy) * half_dt;
	float dq2 = (q0 * gy - q1 * gz + q3 * gx) * half_dt;
	float dq3 = (q0 * gz + q1 * gy - q2 * gx) * half_dt;

	float nq0 = q0 + dq0;
	float nq1 = q1 + dq1;
	float nq2 = q2 + dq2;
	float nq3 = q3 + dq3;

	norm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
	if (norm == 0.0f) {
		return;
	}

	k_mutex_lock(&quat_lock, K_FOREVER);
	q0 = nq0 / norm;
	q1 = nq1 / norm;
	q2 = nq2 / norm;
	q3 = nq3 / norm;
	k_mutex_unlock(&quat_lock);
}

static void handle_frame(const uint8_t *frame)
{
	static uint32_t last_cycles;
	static bool have_last_cycles;

	int16_t x = (int16_t)((frame[3] << 8) | frame[2]);
	int16_t y = (int16_t)((frame[5] << 8) | frame[4]);
	int16_t z = (int16_t)((frame[7] << 8) | frame[6]);

	if (frame[1] == WIT_TYPE_ACCEL) {
		accel[0] = x * WIT_ACCEL_SCALE;
		accel[1] = y * WIT_ACCEL_SCALE;
		accel[2] = z * WIT_ACCEL_SCALE;
		have_accel = true;
	} else if (frame[1] == WIT_TYPE_GYRO) {
		uint32_t now = k_cycle_get_32();
		float dt = k_cyc_to_us_floor32(now - last_cycles) * 1e-6f;

		bool dt_valid = have_last_cycles && dt > 0.0f && dt < 0.5f;

		last_cycles = now;
		have_last_cycles = true;

		if (!have_accel || !dt_valid) {
			return;
		}

		mahony_update(x * WIT_GYRO_SCALE * DEG_TO_RAD, y * WIT_GYRO_SCALE * DEG_TO_RAD,
			      z * WIT_GYRO_SCALE * DEG_TO_RAD, accel[0], accel[1], accel[2], dt);
	}
}

static void ahrs_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t frame[WIT_FRAME_SIZE];
	size_t pos = 0;

	while (true) {
		uint8_t byte;

		if (ring_buf_get(&wit_rx_ring, &byte, 1) != 1) {
			k_sem_take(&wit_rx_sem, K_FOREVER);
			continue;
		}

		if (pos == 0 && byte != WIT_HEADER) {
			continue;
		}

		/* All WitMotion frame types are 0x5x; anything else means we
		 * synced on a data byte, so restart the search */
		if (pos == 1 && (byte & 0xF0) != 0x50) {
			pos = byte == WIT_HEADER ? 1 : 0;
			continue;
		}

		frame[pos++] = byte;
		if (pos < WIT_FRAME_SIZE) {
			continue;
		}
		pos = 0;

		uint8_t sum = 0;
		for (int i = 0; i < WIT_FRAME_SIZE - 1; i++) {
			sum += frame[i];
		}

		if (sum != frame[WIT_FRAME_SIZE - 1]) {
			LOG_WRN("Bad checksum on frame type 0x%02x", frame[1]);
			continue;
		}

		handle_frame(frame);
	}
}

K_THREAD_DEFINE(ahrs, 2048, ahrs_thread, NULL, NULL, NULL, 7, 0, 0);

void get_quaternion(float q[4])
{
	k_mutex_lock(&quat_lock, K_FOREVER);
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
	k_mutex_unlock(&quat_lock);
}

void get_rpy(float *roll, float *pitch, float *yaw)
{
	float q[4];

	get_quaternion(q);

	float sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);

	if (sin_pitch > 1.0f) {
		sin_pitch = 1.0f;
	} else if (sin_pitch < -1.0f) {
		sin_pitch = -1.0f;
	}

	*roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
		       1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) *
		RAD_TO_DEG;
	*pitch = asinf(sin_pitch) * RAD_TO_DEG;
	*yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
		      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) *
	       RAD_TO_DEG;
}

int setup_ahrs(void)
{
	if (!device_is_ready(wit_uart)) {
		LOG_ERR("WT901 UART %s not ready", wit_uart->name);
		return -ENODEV;
	}

	int ret = uart_irq_callback_user_data_set(wit_uart, wit_rx_handler, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to set WT901 UART callback: %d", ret);
		return ret;
	}

	uart_irq_rx_enable(wit_uart);

	return 0;
}
