#ifndef ZEPHYR_INCLUDE_PAW3395_H_
#define ZEPHYR_INCLUDE_PAW3395_H_

#include <drivers/sensor.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

/**
 * @file paw3395.h
 *
 * @brief Header file for the paw3395 driver.
 */

#include <drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

  enum paw3395_run_mode {
    HP_MODE, // high performance mode (the default mode using standard power-up register sets)
    LP_MODE, // low power mode
    OFFICE_MODE, // office mode (reduced to 10 ips, most power-efficient)
    GAME_MODE, // corded game mode (with best performance and highest power consumption)

    RUN_MODE_COUNT // end flag
  };

  /** Sensor specific attributes of PAW3395. */
  enum paw3395_attribute {
    /** Sensor CPI for both X and Y axes. */
    PAW3395_ATTR_CPI = SENSOR_ATTR_PRIV_START,

    /** Enable or disable sleep modes. */
    PAW3395_ATTR_REST_ENABLE,

    /** Entering time from Run mode to REST1 mode [ms]. */
    PAW3395_ATTR_RUN_DOWNSHIFT_TIME,

    /** Entering time from REST1 mode to REST2 mode [ms]. */
    PAW3395_ATTR_REST1_DOWNSHIFT_TIME,

    /** Entering time from REST2 mode to REST3 mode [ms]. */
    PAW3395_ATTR_REST2_DOWNSHIFT_TIME,

    /** Sampling frequency time during REST1 mode [ms]. */
    PAW3395_ATTR_REST1_SAMPLE_TIME,

    /** Sampling frequency time during REST2 mode [ms]. */
    PAW3395_ATTR_REST2_SAMPLE_TIME,

    /** Sampling frequency time during REST3 mode [ms]. */
    PAW3395_ATTR_REST3_SAMPLE_TIME,

    /** Select the running mode. */
    // todo: implement it
    PAW3395_ATTR_RUN_MODE,
  };

  /* device data structure */
  struct paw3395_data {
    const struct device          *dev;
    int16_t                      x;
    int16_t                      y;

    // lock is needed to keep atomic of the trigger handler upadting
    struct k_spinlock            lock;
    // motion interrupt isr
    struct gpio_callback         irq_gpio_cb;
    // the actual trigger handler. This handler also used to flag whether resuming the motion interrupt line
    sensor_trigger_handler_t     data_ready_handler; 
    // the work structure holding the trigger handler job
    struct k_work                trigger_handler_work;

    // the work structure for delayable init steps
    struct k_work_delayable      init_work;
    enum async_init_step         async_init_step;

    //
    bool                         ready; // whether init is finished successfully
    bool                         last_read_burst; // todo: needed?
    int                          err; // error code during async init

    /* the design of the driver is based on interrupt purely, to add polling upon it
       the following work and timer maybe used in application code */
    struct k_work                poll_work;
    struct k_timer               poll_timer;
  };

  /* device config structure */
  struct paw3395_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec cs_gpio;
  };

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_PAW3395_H_ */
