#ifndef _MARITIME_UTIL_H
#define _MARITIME_UTIL_H

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define STR(x) STR_LIT(x)
#define STR_LIT(x) #x

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

#define RING_BUF_DECLARE_STATIC(name, size8) \
	BUILD_ASSERT(size8 < RING_BUFFER_MAX_SIZE,\
		RING_BUFFER_SIZE_ASSERT_MSG); \
	static uint8_t __noinit _ring_buffer_data_##name[size8]; \
	static struct ring_buf name = { \
		.buffer = _ring_buffer_data_##name, \
		.size = size8 \
	}

int parse_int(char *delim, char **save_ptr);
float parse_float(char *delim, char **save_ptr);
float rad_to_deg(float rad);
float deg_to_rad(float deg);

uint32_t time_us();

#endif
