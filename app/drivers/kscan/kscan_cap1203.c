/*
 * Copyright (c) 2022 Keiya Nobuta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zmk_kscan_cap1203

#define USE_POLLING IS_ENABLED(CONFIG_ZMK_KSCAN_CAP1203_POLL)
#define USE_INTERRUPTS (!USE_POLLING)

#include <kernel.h>
#include <drivers/kscan.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(kscan_cap1203, CONFIG_ZMK_LOG_LEVEL);

/* List of important registers and associated commands */
#define REG_MAIN_CONTROL 0x0
#define CONTROL_INT 0x1

#define REG_INPUT_STATUS 0x03

#define REG_INTERRUPT_ENABLE 0x27
#define INTERRUPT_ENABLE 0x7
#define INTERRUPT_DISABLE 0x0

#define CONFIGURATION_2 0x44
#define RELEASE_INT_POS 0

/* device driver data */
struct kscan_cap1203_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec int_gpio;
};

struct kscan_cap1203_data {
	struct device *dev;
	kscan_callback_t callback;
	struct k_work work; // actual processing work
  struct k_work_delayable release_work_a;
  struct k_work_delayable release_work_b;
  struct k_work_delayable release_work_c;
	/* Interrupt GPIO callback. */
	struct gpio_callback int_gpio_cb;
#if USE_POLLING
	/* Timer (polling mode). */
	struct k_timer timer;
#endif
};

/* Write a single bit in a register without touching other bits */
static int kscan_cap1203_bit_write(const struct i2c_dt_spec *i2c, uint8_t reg, \
                                   uint8_t pos, bool enable)
{
  uint8_t val;
  int err;

  if(err=i2c_reg_read_byte_dt(i2c, reg, &val)) {
    return err;
  }

  WRITE_BIT(val, pos, enable);

  return i2c_reg_write_byte_dt(i2c, reg, val);
}

/* Enable/disable generation of release interrupt */
static int kscan_cap1203_enable_release_interrupt(const struct i2c_dt_spec *i2c,\
                                                  bool enable)
{
  return kscan_cap1203_bit_write(i2c, CONFIGURATION_2, RELEASE_INT_POS, !enable);
}


/* De-assert alert line and clear the sensor status input register */
static int kscan_cap1203_clear_interrupt(const struct i2c_dt_spec *i2c)
{
	uint8_t ctrl;
	int r;

	r = i2c_reg_read_byte_dt(i2c, REG_MAIN_CONTROL, &ctrl);
	if (r < 0) {
		return r;
	}

	ctrl = ctrl & ~CONTROL_INT;

	return i2c_reg_write_byte_dt(i2c, REG_MAIN_CONTROL, ctrl);
}

/* Enable/disable the alert line */
static int kscan_cap1203_enable_interrupt(const struct i2c_dt_spec *i2c, bool enable)
{
	uint8_t intr = enable ? INTERRUPT_ENABLE : INTERRUPT_DISABLE;

	return i2c_reg_write_byte_dt(i2c, REG_INTERRUPT_ENABLE, intr);
}

/* read the current touch status */
static int kscan_cap1203_read(const struct device *dev)
{
	const struct kscan_cap1203_config *config = dev->config;
	struct kscan_cap1203_data *data = dev->data;
	int r;
	uint8_t input;
	bool pressed;

  /* todo: release previus presses if not yet released */
  /* k_work_flush_delayable(&data->release_work_a, NULL); */
  /* k_work_flush_delayable(&data->release_work_b, NULL); */
  /* k_work_flush_delayable(&data->release_work_c, NULL); */

	r = i2c_reg_read_byte_dt(&config->i2c, REG_INPUT_STATUS, &input);
	if (r < 0) {
		return r;
	}

  //
	LOG_INF("event: input: %d", input);
	pressed = !!input; // !! can turn an arbitraty integer into 1 or 0

	if (input & BIT(2)) {
    data->callback(dev, 0, 2, pressed);

    LOG_INF("scheduled to release for button c");
    k_work_schedule(&data->release_work_c, K_MSEC(1));
	}
	if (input & BIT(1)) {
    data->callback(dev, 0, 1, pressed);

    LOG_INF("scheduled to release for button b");
    k_work_schedule(&data->release_work_b, K_MSEC(1));
	}
	if (input & BIT(0)) {
    data->callback(dev, 0, 0, pressed);

    LOG_INF("scheduled to release for button a");
    k_work_schedule(&data->release_work_a, K_MSEC(1));
	}


	/*
	 * Clear INT bit to clear SENSOR INPUT STATUS bits.
	 * Note that this is also required in polling mode.
	 */
	r = kscan_cap1203_clear_interrupt(&config->i2c);
	if (r < 0) {
		return r;
	}

	return 0;
}

static void kscan_cap1203_release_handler_a(struct k_work *work)
{
	struct kscan_cap1203_data *data = CONTAINER_OF(work, struct kscan_cap1203_data, release_work_a);

  data->callback(data->dev, 0, 0, false);
  LOG_INF("rlease button a\n");
}

static void kscan_cap1203_release_handler_b(struct k_work *work)
{
	struct kscan_cap1203_data *data = CONTAINER_OF(work, struct kscan_cap1203_data, release_work_b);

  data->callback(data->dev, 0, 1, false);
  LOG_INF("rlease button b\n");
}

static void kscan_cap1203_release_handler_c(struct k_work *work)
{
	struct kscan_cap1203_data *data = CONTAINER_OF(work, struct kscan_cap1203_data, release_work_c);

  data->callback(data->dev, 0, 2, false);
  LOG_INF("rlease button c\n");
}

/* the actual work to process each alert or polling */
static void kscan_cap1203_work_handler(struct k_work *work)
{
	struct kscan_cap1203_data *data = CONTAINER_OF(work, struct kscan_cap1203_data, work);

	kscan_cap1203_read(data->dev);
}

/* ISR handler: immediatly dispatch the job to the work handler */
static void kscan_cap1203_irq_handler(const struct device *dev,
				struct gpio_callback *cb, uint32_t pins)
{
	struct kscan_cap1203_data *data = CONTAINER_OF(cb, struct kscan_cap1203_data, int_gpio_cb);

	k_work_submit(&data->work);
}

/* Timer handler in poll mode: immediately dispatch the job to the actual work handler */
#if USE_POLLING
static void kscan_cap1203_timer_handler(struct k_timer *timer)
{
	struct kscan_cap1203_data *data = CONTAINER_OF(timer, struct kscan_cap1203_data, timer);

	k_work_submit(&data->work);
}
#endif

/* api callbacks */
static int kscan_cap1203_configure(const struct device *dev,
			     kscan_callback_t callback)
{
	struct kscan_cap1203_data *data = dev->data;
	const struct kscan_cap1203_config *config = dev->config;

	data->callback = callback;

	if (config->int_gpio.port != NULL) {
		int r;

		/* Clear pending interrupt */
		r = kscan_cap1203_clear_interrupt(&config->i2c);
		if (r < 0) {
			LOG_ERR("Could not clear interrupt");
			return r;
		}

		r = kscan_cap1203_enable_interrupt(&config->i2c, true);
		if (r < 0) {
			LOG_ERR("Could not configure interrupt");
			return r;
		}
	}

	return 0;
}

static int kscan_cap1203_enable_callback(const struct device *dev)
{
	struct kscan_cap1203_data *data = dev->data;

	const struct kscan_cap1203_config *config = dev->config;

	if (config->int_gpio.port != NULL) {
		gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
	}
#if USE_POLLING
	else {
		k_timer_start(&data->timer, K_MSEC(CONFIG_ZMK_KSCAN_CAP1203_PERIOD),
			      K_MSEC(CONFIG_ZMK_KSCAN_CAP1203_PERIOD));
	}
#endif
	return 0;
}

static int kscan_cap1203_disable_callback(const struct device *dev)
{
	struct kscan_cap1203_data *data = dev->data;

	const struct kscan_cap1203_config *config = dev->config;

	if (config->int_gpio.port != NULL) {
		gpio_remove_callback(config->int_gpio.port, &data->int_gpio_cb);
	}
#if USE_POLLING
	else {
		k_timer_stop(&data->timer);
	}
#endif
	return 0;
}

static int kscan_cap1203_init(const struct device *dev)
{
	const struct kscan_cap1203_config *config = dev->config;
	struct kscan_cap1203_data *data = dev->data;
	int r;

  // check readiness of i2c bus
	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C controller device not ready");
		return -ENODEV;
	}

  // init data structure
	data->dev = dev;

	k_work_init(&data->work, kscan_cap1203_work_handler);

	k_work_init_delayable(&data->release_work_a, kscan_cap1203_release_handler_a);
	k_work_init_delayable(&data->release_work_b, kscan_cap1203_release_handler_b);
	k_work_init_delayable(&data->release_work_c, kscan_cap1203_release_handler_c);


  // init alert interrupt pin or poll timer
	if (config->int_gpio.port != NULL) {
		if (!device_is_ready(config->int_gpio.port)) {
			LOG_ERR("Interrupt GPIO controller device not ready");
			return -ENODEV;
		}

		r = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
		if (r < 0) {
			LOG_ERR("Could not confighure interrupt GPIO pin");
			return r;
		}

		r = gpio_pin_interrupt_configure_dt(&config->int_gpio,
						   GPIO_INT_EDGE_TO_ACTIVE);
		if (r < 0) {
			LOG_ERR("Could not configure interrupt GPIO interrupt");
			return r;
		}

		gpio_init_callback(&data->int_gpio_cb, kscan_cap1203_irq_handler,
				   BIT(config->int_gpio.pin));
	}
#if USE_POLLING
	else {
		k_timer_init(&data->timer, kscan_cap1203_timer_handler, NULL);

    // disable alert function in poll mode
		r = kscan_cap1203_enable_interrupt(&config->i2c, false);
		if (r < 0) {
			LOG_ERR("Could not configure interrupt");
			return r;
		}
	}
#endif

  // other configuration
  r = kscan_cap1203_enable_release_interrupt(&config->i2c, false);
  if(r) {
    LOG_ERR("Could not disable release interrupt");
    return r;
  }

  LOG_INF("Init success");

	return 0;
}

static const struct kscan_driver_api kscan_cap1203_driver_api = {
	.config = kscan_cap1203_configure,
	.enable_callback = kscan_cap1203_enable_callback,
	.disable_callback = kscan_cap1203_disable_callback,
};

#define KSCAN_CAP1203_INIT(index)							\
	static const struct kscan_cap1203_config kscan_cap1203_config_##index = {		\
		.i2c = I2C_DT_SPEC_INST_GET(index),				\
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(index, int_gpios, {0}),	\
	};									\
	static struct kscan_cap1203_data kscan_cap1203_data_##index;			\
	DEVICE_DT_INST_DEFINE(index, kscan_cap1203_init, NULL,			\
			      &kscan_cap1203_data_##index, &kscan_cap1203_config_##index,	\
			      APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY ,		\
			      &kscan_cap1203_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_CAP1203_INIT)
