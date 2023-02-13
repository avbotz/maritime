#pragma once

struct gyro_raw_s {
	int64_t timestamp;

	float x;
	float y;
	float z;

	float temperature;
};
