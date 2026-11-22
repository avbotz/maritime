#include "thruster.h"

#include <zephyr/drivers/pwm/pwm_fake.h>
#include <zephyr/ztest.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NEUTRAL    1500000 /* [ns] */
#define HALF_RANGE 400000  /* [ns] from neutral to full throttle */
#define MAX_MA     24000
/* The search finds the scale to within 1/1024, and throttle is at most 0.8 */
#define STEP_NS    (HALF_RANGE * 8 / 10 / 1024 + 2)
#define ROUND_NS   2

DEFINE_FFF_GLOBALS;

static int32_t pulses[NUM_THRUSTERS];

static int capture_pulse(const struct device *dev, uint32_t channel, uint32_t period,
			 uint32_t pulse, pwm_flags_t flags)
{
	zassert_true(channel < NUM_THRUSTERS);
	pulses[channel] = pulse;
	return 0;
}

static void send(const float throttles[NUM_THRUSTERS])
{
	memset(pulses, 0, sizeof(pulses));
	thruster_send_all(throttles);
}

static void expect_unlimited(const float throttles[NUM_THRUSTERS])
{
	send(throttles);
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		int32_t want = NEUTRAL + (int32_t)(throttles[i] * HALF_RANGE);

		zassert_within(pulses[i], want, ROUND_NS, "thruster %d: got %d, want %d", i,
			       pulses[i], want);
	}
}

/* A limited pulse may stop one search step short of the ideal, but never go past it */
static void expect_limited(int i, int32_t ideal)
{
	int32_t got = abs(pulses[i] - NEUTRAL);
	int32_t most = abs(ideal - NEUTRAL);

	zassert_true((pulses[i] > NEUTRAL) == (ideal > NEUTRAL), "thruster %d reversed", i);
	zassert_between_inclusive(got, most - STEP_NS, most + ROUND_NS,
				  "thruster %d: got %d, want %d", i, pulses[i], ideal);
}

/* The curve from app.overlay, written out independently of thruster.c */
static double milliamps(int32_t pulse)
{
	double us = (pulse - NEUTRAL) / 1000.0;

	if (us < -200) {
		return 6000 + (-us - 200) * 70;
	} else if (us < 0) {
		return -us * 30;
	} else if (us <= 200) {
		return us * 20;
	}
	return 4000 + (us - 200) * 60;
}

ZTEST(thruster_limits, test_throttle_clamped)
{
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		float throttles[NUM_THRUSTERS] = {0};

		throttles[i] = 1.0f;
		send(throttles);
		zassert_within(pulses[i], 1820000, ROUND_NS);

		throttles[i] = -5.0f;
		send(throttles);
		zassert_within(pulses[i], 1180000, ROUND_NS);
	}
}

ZTEST(thruster_limits, test_non_finite_is_neutral)
{
	const float throttles[NUM_THRUSTERS] = {NAN, INFINITY, -INFINITY, 0.25f};

	send(throttles);
	for (int i = 0; i < 3; i++) {
		zassert_equal(pulses[i], NEUTRAL);
	}
	zassert_within(pulses[3], 1600000, ROUND_NS);
}

ZTEST(thruster_limits, test_under_limit_untouched)
{
	const float throttles[NUM_THRUSTERS] = {0.8f, 0.8f};

	/* 2 * 11.2 A */
	expect_unlimited(throttles);
}

ZTEST(thruster_limits, test_at_limit_untouched)
{
	const float throttles[NUM_THRUSTERS] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

	/* 6 * 4 A is exactly the limit */
	expect_unlimited(throttles);
}

ZTEST(thruster_limits, test_just_over_limit)
{
	const float throttles[NUM_THRUSTERS] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.01f};

	/* 24.08 A */
	send(throttles);
	zassert_true(pulses[0] < 1700000);
}

ZTEST(thruster_limits, test_forward_limited)
{
	const float throttles[NUM_THRUSTERS] = {0.8f, 0.8f, 0.8f};

	/* 3 * 11.2 A is over, so each gets 8 A */
	send(throttles);
	for (int i = 0; i < 3; i++) {
		expect_limited(i, 1766667);
	}
}

ZTEST(thruster_limits, test_reverse_limited)
{
	const float throttles[NUM_THRUSTERS] = {-0.8f, -0.8f};

	/* 2 * 14.4 A is over, so each gets 12 A. Forward would not have been limited */
	send(throttles);
	for (int i = 0; i < 2; i++) {
		expect_limited(i, 1214286);
	}
}

ZTEST(thruster_limits, test_all_limited)
{
	float throttles[NUM_THRUSTERS];

	for (int i = 0; i < NUM_THRUSTERS; i++) {
		throttles[i] = 1.0f;
	}

	/* 3 A each */
	send(throttles);
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		expect_limited(i, 1650000);
	}
}

ZTEST(thruster_limits, test_ratios_preserved)
{
	const float throttles[NUM_THRUSTERS] = {0.8f, -0.8f, 0.4f, -0.2f, 0, 0.6f, -0.4f, 0.1f};

	send(throttles);

	float scale = (pulses[0] - NEUTRAL) / (0.8f * HALF_RANGE);

	zassert_true(scale < 1.0f);
	for (int i = 0; i < NUM_THRUSTERS; i++) {
		int32_t want = NEUTRAL + (int32_t)(scale * throttles[i] * HALF_RANGE);

		zassert_within(pulses[i], want, ROUND_NS, "thruster %d", i);
	}
}

ZTEST(thruster_limits, test_limit_not_sticky)
{
	const float over[NUM_THRUSTERS] = {0.8f, 0.8f, 0.8f};
	const float under[NUM_THRUSTERS] = {0.25f};

	send(over);
	expect_unlimited(under);
}

/* Every combination of five throttles on eight thrusters */
ZTEST(thruster_limits, test_all_combinations)
{
	static const float levels[] = {-1.0f, -0.5f, 0.0f, 0.25f, 1.0f};

	for (int n = 0; n < 390625; n++) { /* 5^8 */
		float throttles[NUM_THRUSTERS];
		double wanted = 0;
		double sent = 0;

		for (int i = 0, rest = n; i < NUM_THRUSTERS; i++, rest /= ARRAY_SIZE(levels)) {
			float t = levels[rest % ARRAY_SIZE(levels)];

			throttles[i] = t;
			t = CLAMP(t, -0.8f, 0.8f);
			wanted += milliamps(NEUTRAL + (int32_t)(t * HALF_RANGE));
		}

		send(throttles);
		for (int i = 0; i < NUM_THRUSTERS; i++) {
			sent += milliamps(pulses[i]);
		}

		zassert_true(sent <= MAX_MA + 1, "case %d: %d mA is over", n, (int)sent);
		/* One search step is worth under 200 mA, so less means it limited too much */
		zassert_true(sent >= MIN(wanted, MAX_MA) - 200, "case %d: %d mA is too low", n,
			     (int)sent);
	}
}

static void before(void *fixture)
{
	fake_pwm_set_cycles_fake.custom_fake = capture_pulse;
}

ZTEST_SUITE(thruster_limits, NULL, NULL, before, NULL, NULL);
