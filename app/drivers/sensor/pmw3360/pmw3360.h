/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ZEPHYR_INCLUDE_PMW3360_H_
#define ZEPHYR_INCLUDE_PMW3360_H_

#include <drivers/sensor.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

/**
 * @file pmw3360.h
 *
 * @brief Header file for the pmw3360 driver.
 */

#include <drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

enum async_init_step {
	ASYNC_INIT_STEP_POWER_UP, // power up reset
	ASYNC_INIT_STEP_FW_LOAD_START, // clear motion registers, disable REST mode, enable SROM register
	ASYNC_INIT_STEP_FW_LOAD_CONTINUE, // start SROM download
	ASYNC_INIT_STEP_FW_LOAD_VERIFY, // verify SROM pid and fid, enable REST mode
	ASYNC_INIT_STEP_CONFIGURE, // set cpi and donwshift time (run, rest1, rest2)

	ASYNC_INIT_STEP_COUNT // end flag
};

/* device data structure */
struct pmw3360_data {
	const struct device          *dev;
	struct gpio_callback         irq_gpio_cb;
	struct k_spinlock            lock;
	int16_t                      x;
	int16_t                      y;
	sensor_trigger_handler_t     data_ready_handler;
	struct k_work                trigger_handler_work;
	struct k_work_delayable      init_work;

	enum async_init_step         async_init_step;
	int                          err;
	bool                         ready;
	bool                         last_read_burst;

  /* the design of the driver is based on interrupt purely, to add polling upon it
   the following work and timer maybe used in application code */
	struct k_work                poll_work;
  struct k_timer               poll_timer;
};

struct pmw3360_config {
	struct gpio_dt_spec irq_gpio;
	struct spi_dt_spec bus;
	struct gpio_dt_spec cs_gpio;
};

/**
 * @defgroup pmw3360 PMW3360 motion sensor driver
 * @{
 * @brief PMW3360 motion sensor driver.
 */

/** @brief Sensor specific attributes of PMW3360. */
enum pmw3360_attribute {
	/** Sensor CPI for both X and Y axes. */
	PMW3360_ATTR_CPI = SENSOR_ATTR_PRIV_START,

	/** Enable or disable sleep modes. */
	PMW3360_ATTR_REST_ENABLE,

	/** Entering time from Run mode to REST1 mode [ms]. */
	PMW3360_ATTR_RUN_DOWNSHIFT_TIME,

	/** Entering time from REST1 mode to REST2 mode [ms]. */
	PMW3360_ATTR_REST1_DOWNSHIFT_TIME,

	/** Entering time from REST2 mode to REST3 mode [ms]. */
	PMW3360_ATTR_REST2_DOWNSHIFT_TIME,

	/** Sampling frequency time during REST1 mode [ms]. */
	PMW3360_ATTR_REST1_SAMPLE_TIME,

	/** Sampling frequency time during REST2 mode [ms]. */
	PMW3360_ATTR_REST2_SAMPLE_TIME,

	/** Sampling frequency time during REST3 mode [ms]. */
	PMW3360_ATTR_REST3_SAMPLE_TIME,
};


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PMW3360_H_ */
