#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include "lackc/brake_lights.h"

#define SLEEP_TIME_MS   10

// static const struct gpio_dt_spec in_imu_interrupt =
// 	GPIO_DT_SPEC_GET(DT_ALIAS(in_imu_interrupt), gpios);

static const struct gpio_dt_spec out_brake_relay =
	GPIO_DT_SPEC_GET(DT_ALIAS(out_brake_relay), gpios);
static const struct gpio_dt_spec out_brake_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(out_brake_led), gpios);
static const struct gpio_dt_spec out_ok_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(out_ok_led), gpios);
static const struct gpio_dt_spec out_nok_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(out_nok_led), gpios);

static const struct device *const dev_imu =
	DEVICE_DT_GET_ONE(invensense_mpu6050);

typedef struct imu {
	bool clock;
	double accel[3];
	double gyro[4];
} imu_t;

static imu_t latest_imu;

static Top$Brakes$state brakes_state;

static void lackc_init() {
	Top$Brakes$reset(&brakes_state);
}

static void lackc_step(imu_t imu) {
	Top$Brakes$step$out out;
	Top$Brakes$step(&brakes_state,
		imu.clock,
		imu.accel[0], imu.accel[1], imu.accel[2],
		imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.gyro[3],
		&out);

#define RELAY_EN
#ifdef RELAY_EN
	gpio_pin_set_dt(&out_brake_relay, out.light);
#endif
	gpio_pin_set_dt(&out_brake_led, out.light);
	gpio_pin_set_dt(&out_ok_led,    out.ok);
	gpio_pin_set_dt(&out_nok_led,   out.nok_stuck);
}

static void process_imu() {
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	// TODO is this necessarily ensuring fresh data?
	int nok = sensor_sample_fetch(dev_imu);
	if (!nok) {
		nok = sensor_channel_get(dev_imu, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (!nok) {
		nok = sensor_channel_get(dev_imu, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (!nok) {
		latest_imu.clock = true;
		latest_imu.accel[0] = sensor_value_to_double(&accel[0]);
		latest_imu.accel[1] = sensor_value_to_double(&accel[1]);
		latest_imu.accel[2] = sensor_value_to_double(&accel[2]);
		// TODO I want to convert the gyro to a quaternion so the control system can apply it with linear ops
		latest_imu.gyro[0] = sensor_value_to_double(&gyro[0]);
		latest_imu.gyro[1] = sensor_value_to_double(&gyro[1]);
		latest_imu.gyro[2] = sensor_value_to_double(&gyro[2]);
		latest_imu.gyro[3] = 0.0;
	} else {
		latest_imu.clock = false;
	}
}


static void main_timer_work_step(struct k_work *work) {
	lackc_step(latest_imu);
	process_imu();
}

K_WORK_DEFINE(main_timer_work, main_timer_work_step);

static void main_timer_step(struct k_timer *timer) {
	k_work_submit(&main_timer_work);
}

K_TIMER_DEFINE(main_timer, main_timer_step, NULL);


void main(void) {
	int ret;

	if (!device_is_ready(out_brake_relay.port)) {
		return;
	}

	ret =
		// gpio_pin_configure_dt(&in_imu_interrupt, GPIO_INPUT) ||
		gpio_pin_configure_dt(&out_brake_relay, GPIO_OUTPUT_INACTIVE) ||
		gpio_pin_configure_dt(&out_brake_led, GPIO_OUTPUT_ACTIVE) ||
		gpio_pin_configure_dt(&out_ok_led, GPIO_OUTPUT_ACTIVE) ||
		gpio_pin_configure_dt(&out_nok_led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	gpio_pin_set_dt(&out_nok_led, true);

	if (!device_is_ready(dev_imu)) {
		return;
	}

	lackc_init();
	latest_imu.clock = false;

	k_timer_start(&main_timer, K_MSEC(10), K_MSEC(10));
}