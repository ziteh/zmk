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
#define REG_PRODUCT_ID 0xFD
#define PRODUCT_ID 0x6D
#define REG_VENDOR_ID 0xFE
#define VENDOR_ID 0x5D
#define REG_REVISION 0xFF
#define REVISION_ID 0x00

#define REG_MAIN_CONTROL 0x0
#define CONTROL_INT 0x1

#define REG_GENERAL_STATUS 0x02
#define REG_INPUT_STATUS 0x03

#define REG_INTERRUPT_ENABLE 0x27
#define INTERRUPT_ENABLE 0x7
#define INTERRUPT_DISABLE 0x0

#define REG_REPEAT_ENABLE 0x28
#define REPEAT_ENABLE 0x7
#define REPEAT_DISABLE 0x0

#define REG_MULT_CONFIG 0x2A
#define MULT_BLK_EN_POS 7
#define B_MULT_T1_POS   3
#define B_MULT_T0_POS 2

#define REG_CONFIGURATION_2 0x44
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
	/* Interrupt GPIO callback. */
	struct gpio_callback int_gpio_cb;
#if USE_POLLING
	/* Timer (polling mode). */
	struct k_timer timer;
#endif

  uint8_t touch_state;

#ifdef CONFIG_CAP1203_SLIDER_MODE || CONFIG_CAP1203_MIX_MODE
  uint8_t slider_position;
  int16_t delta_position;
#endif
};

#ifdef CONFIG_CAP1203_SLIDER_MODE || CONFIG_CAP1203_MIX_MODE
/* array of the status patterns, the array_index+1 is the position indicator */
static uint8_t const slider_pattern[5] = {1, 3, 2, 6, 4};
/* array of slider positions with slider pattern as array index */
/* The position of unknown slider patterns is defined to be 0 */
static uint8_t const slider_position[8] = {0, 1, 3, 2, 5, 0, 4, 0};
static int     update_counter;
#endif

/* Functions and variables for debugging usage */
#ifdef CONFIG_ZMK_USB_LOGGING
static inline void print_register(const struct i2c_dt_spec *i2c, uint8_t reg, const char* prefix)
{
  uint8_t status;
	int err = i2c_reg_read_byte_dt(i2c, reg, &status);
	if (err < 0) {
    LOG_ERR("Debug can's read register: 0x%x", reg);
		return;
	}
	LOG_INF("%s: register 0x%x = 0x%x", prefix, reg, status);
  return;
}
#endif

/* Write a single bit in a register without touching other bits */
static inline int kscan_cap1203_bit_write(const struct i2c_dt_spec *i2c, uint8_t reg, \
                                   uint8_t pos, bool enable)
{
  uint8_t val;
  int err;

  if(err=i2c_reg_read_byte_dt(i2c, reg, &val)) {
    return err;
  }

  if( enable )
    WRITE_BIT(val, pos, 1);
  else
    WRITE_BIT(val, pos, 0);

  return i2c_reg_write_byte_dt(i2c, reg, val);
}

/* Check the status of the sensor by comparing the vid, revision and pid with the datasheet */
static inline int kscan_cap1203_check_firmware(const struct device *dev)
{
	const struct kscan_cap1203_config *config = dev->config;

  uint8_t val;
  int err;

  if(err=i2c_reg_read_byte_dt(&config->i2c, REG_PRODUCT_ID, &val)) {
    LOG_INF("Can't read register: product id");
    return err;
  }
  else if( val != PRODUCT_ID ) {
    LOG_INF("Unequal product id 0x%x (expected: 0x%x)", val, PRODUCT_ID);
    return -EIO;
  }

  if(err=i2c_reg_read_byte_dt(&config->i2c, REG_VENDOR_ID, &val)) {
    LOG_INF("Can't read register: vendor id");
    return err;
  }
  else if( val != VENDOR_ID ) {
    LOG_INF("Unequal vendor id 0x%x (expected: 0x%x)", val, VENDOR_ID);
    return -EIO;
  }

  return 0;
}

/* Config multiple touch circuitry */
inline static int kscan_cap1203_configure_multiple_touch(const struct i2c_dt_spec *i2c, \
                                                         bool enable, \
                                                         int num)
{
  uint8_t val=0;

  if(enable) {
    WRITE_BIT(val, MULT_BLK_EN_POS, 1);

    switch (num) {
    case 1:
      WRITE_BIT(val, B_MULT_T1_POS, 0);
      WRITE_BIT(val, B_MULT_T0_POS, 0);
      break;
    case 2:
      WRITE_BIT(val, B_MULT_T1_POS, 0);
      WRITE_BIT(val, B_MULT_T0_POS, 1);
      break;
    case 3:
      WRITE_BIT(val, B_MULT_T1_POS, 1);
      WRITE_BIT(val, B_MULT_T0_POS, 0);
      break;
    default:
      LOG_ERR("Number of channels is out of range: max=3");
      return -EINVAL;
    }
  }
  else {
    WRITE_BIT(val, MULT_BLK_EN_POS, 0);
  }

  return i2c_reg_write_byte_dt(i2c, REG_MULT_CONFIG, val);
}
/* Enable/disable generation of release interrupt */
static int kscan_cap1203_enable_release_interrupt(const struct i2c_dt_spec *i2c,\
                                                  bool enable)
{
  return kscan_cap1203_bit_write(i2c, REG_CONFIGURATION_2, RELEASE_INT_POS, !enable);
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

/* determine the press status based on latest sensor input */
// arg1 new_input: the current status input register value
// arg2 ch: the sensor channel to be tested
// arg3 pressed: whether the state change is pressed or released
// return: true if there is state change, otherwise false
inline static bool kscan_cap1203_change_state(uint8_t *old_input,  \
                                              uint8_t *new_input, \
                                              int  ch, \
                                              bool *pressed)
{

  if((*old_input & BIT(ch) ) != (*new_input & BIT(ch))) {
    *pressed = *new_input & BIT(ch);
    return true;
  }
  return false;
}

/* read the current touch status */
static int kscan_cap1203_read(const struct device *dev)
{
	const struct kscan_cap1203_config *config = dev->config;
	struct kscan_cap1203_data *data = dev->data;
	int r;
	uint8_t input;

  LOG_INF("Beginning of ISR:");
#ifdef CONFIG_CAP1203_BUTTON_MODE
	bool pressed;

  // read general status
#ifdef CONFIG_ZMK_USB_LOGGING
  print_register(&config->i2c, REG_GENERAL_STATUS, "General Status");
#endif

  // read sensor input status
	r = i2c_reg_read_byte_dt(&config->i2c, REG_INPUT_STATUS, &input);
	if (r < 0) {
		return r;
	}
	LOG_INF("Initial input status: 0x%x (old: 0x%x)", input, data->touch_state);

  // check the state change, only pressed event can be discovered at this stage
  for( int ch=0; ch < 3; ch++ ) {
    if(kscan_cap1203_change_state(&data->touch_state, &input, ch, &pressed)) {
      data->callback(dev, 0, ch, pressed);
      LOG_INF("Pad %d %s", ch+1, pressed ? "pressed" : "released");
    }
  }

	// Clear INT bit to clear SENSOR INPUT STATUS bits and dis-claim interrupt line
	r = kscan_cap1203_clear_interrupt(&config->i2c);
	if (r < 0) {
		return r;
	}

  // read sensor input status again, this time to discover release event
	r = i2c_reg_read_byte_dt(&config->i2c, REG_INPUT_STATUS, &data->touch_state);
	if (r < 0) {
		return r;
	}
	LOG_INF("Input status after clearing: 0x%x (old: 0x%x)", data->touch_state, input);

  // if interrput is triggered by release, there will be state change after claering
  for( int ch=0; ch < 3; ch++ ) {
    if(kscan_cap1203_change_state(&input, &data->touch_state, ch, &pressed)) {
      data->callback(dev, 0, ch, pressed);
      LOG_INF("Pad %d %s", ch+1, pressed ? "pressed" : "released");
    }
  }

#elif defined(CONFIG_CAP1203_SLIDER_MODE)
  // read general status
#ifdef CONFIG_ZMK_USB_LOGGING
  print_register(&config->i2c, REG_GENERAL_STATUS, "General Status");
#endif

  // First reading of sensor input status, only pressed info updated
	r = i2c_reg_read_byte_dt(&config->i2c, REG_INPUT_STATUS, &input);
	if (r < 0) {
		return r;
	}

  // Clear INT bit to update release info
  r = kscan_cap1203_clear_interrupt(&config->i2c);
  if (r < 0) {
    return r;
  }

  // Get the updated status
  update_counter++;
  if(data->touch_state == 0) { // initial pos
    LOG_INF("Update 1 status: 0x%x", input);
  }
  else { // later state change
    // Second reading of sensor input status, which includes release info
    r = i2c_reg_read_byte_dt(&config->i2c, REG_INPUT_STATUS, &input);
    if (r < 0) {
      return r;
    }
    LOG_INF("Update %d status: 0x%x", update_counter, input);
  }

  // Process the slider pattern, unknown patterns are skipped
  if(input && data->touch_state != 0) { // the first and last update should not be used
    data->delta_position = slider_position[input] - data->slider_position;
    if (data->delta_position != 0) {
      // todo: callback to process the deltas
    }
    LOG_INF("Delta: %d", data->delta_position);
  }

  // book-keeping stuff
  data->touch_state = input;

  if(!input) { // reach the end of slide gesture
    data->slider_position = 0;
    data->delta_position = 0;
    update_counter = 0;
    LOG_INF("Last update");
  }
  else { // for all other updates before last one
    // keep the current status for next update usage
    data->slider_position = slider_position[input];
  }

#elif defined(CONFIG_CAP1203_MIX_MODE)

#endif

  LOG_INF("End of ISR.\n");
	return 0;
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

  // configure release interrupt
  r = kscan_cap1203_enable_release_interrupt(&config->i2c, true);
  if(r) {
    LOG_ERR("Could not configure release interrupt");
    return r;
  }

  // configure repeat interrupt
	r = i2c_reg_write_byte_dt(&config->i2c, REG_REPEAT_ENABLE, REPEAT_DISABLE);
	if (r < 0) {
    LOG_ERR("Could not configure repeat-rate interrupt");
		return r;
	}

  // configure multi-touch circuitry
  r = kscan_cap1203_configure_multiple_touch(&config->i2c, true, 3);
	if (r < 0) {
    LOG_ERR("Could not configure multi-touch circuitry");
		return r;
	}

  // End of init
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
