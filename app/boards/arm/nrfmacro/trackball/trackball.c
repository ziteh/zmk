#define DT_DRV_COMPAT pixart_pmw33xx

#include <drivers/sensor.h>
#include <logging/log.h>

#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <dt-bindings/zmk/mouse.h>

#define SCROLL_DIV_FACTOR 5
/* #define SCROLL_LAYER_INDEX 4 */
#define SCROLL_LAYER_INDEX COND_CODE_0(DT_INST_NODE_HAS_PROP(0, scroll_layer), (255), \
                                       (DT_INST_PROP(0, scroll_layer)))


/* #if IS_ENABLED(CONFIG_SENSOR_LOG_LEVEL_DBG) */
// in us
static int64_t last_interrupt_time = 0;
static int64_t current_interrupt_time = 0;
static int64_t interrupt_interval = 0;
static int64_t handler_duration = 0;
static int64_t send_report_duration = 0;
static int64_t idle_interval = 0;
static int64_t time_buffer = 0;
/* #endif */

// TEST: for controlling the update frq
#define MIN_UPDATE_INTERVAL 12000 // in us
static int64_t acc_interval = 0;
static int acc_interrupt_count = 0;

//
static struct sensor_value dx, dy;

const struct device *trackball = DEVICE_DT_GET(DT_DRV_INST(0));

LOG_MODULE_REGISTER(trackball, CONFIG_SENSOR_LOG_LEVEL);

/* update and send report */
static int64_t trackball_update_handler(struct k_work *work) {
/* #if IS_ENABLED(CONFIG_SENSOR_LOG_LEVEL_DBG) */
  int64_t start_time = k_ticks_to_us_floor64(k_uptime_ticks());
/* #endif */

  // remaining scroll from last update
  static int8_t scroll_ver_rem = 0, scroll_hor_rem = 0;

  // update report with latest position
  zmk_hid_mouse_movement_set(0, 0);
  zmk_hid_mouse_scroll_set(0, 0);

  const uint8_t layer = zmk_keymap_highest_layer_active();
  if (layer == SCROLL_LAYER_INDEX) {   // lower
    const int16_t total_hor = dx.val1 + scroll_hor_rem, total_ver = -(dy.val1 + scroll_ver_rem);
    scroll_hor_rem = total_hor % SCROLL_DIV_FACTOR;
    scroll_ver_rem = total_ver % SCROLL_DIV_FACTOR;
    zmk_hid_mouse_scroll_update(total_hor / SCROLL_DIV_FACTOR, total_ver / SCROLL_DIV_FACTOR);
  } else {
    zmk_hid_mouse_movement_update(CLAMP(dx.val1, INT8_MIN, INT8_MAX), CLAMP(dy.val1, INT8_MIN, INT8_MAX));
  }

  // send the report to host
  zmk_endpoints_send_mouse_report();

/* #if IS_ENABLED(CONFIG_SENSOR_LOG_LEVEL_DBG) */
  return start_time = k_ticks_to_us_floor64(k_uptime_ticks()) - start_time;
/* #else */
/*   return 0; */
/* #endif */
}

/* trigger handler, invoked by gpio interrupt */
static void handle_trackball(const struct device *dev, const struct sensor_trigger *trig) {
/* #if IS_ENABLED(CONFIG_SENSOR_LOG_LEVEL_DBG) */
  current_interrupt_time = k_ticks_to_us_floor64(k_uptime_ticks());
  interrupt_interval = current_interrupt_time - last_interrupt_time;
  idle_interval = current_interrupt_time - time_buffer;
/* #endif */

  // TEST: control update frq
  acc_interval += interrupt_interval;
  acc_interrupt_count++;


    // fetch latest position from sensor
  if(acc_interval > MIN_UPDATE_INTERVAL) {
    int ret = sensor_sample_fetch(dev);
    if (ret < 0) {
        LOG_ERR("fetch: %d", ret);
        return;
    }

    // get the x, y delta
    ret = sensor_channel_get(dev, SENSOR_CHAN_POS_DX, &dx);
    if (ret < 0) {
        LOG_ERR("get dx: %d", ret);
        return;
    }
    ret = sensor_channel_get(dev, SENSOR_CHAN_POS_DY, &dy);
    if (ret < 0) {
        LOG_ERR("get dy: %d", ret);
        return;
    }

    // process the updated position and send to host
    /* k_work_submit_to_queue(zmk_mouse_work_q(), &trackball_update); */
/* #if IS_ENABLED(CONFIG_SENSOR_LOG_LEVEL_DBG) */
    send_report_duration = trackball_update_handler(NULL);
/* #else */
/*     trackball_update_handler(NULL); */
/* #endif */
    
    LOG_INF("Update interval (us): %lld ; Nr of interrupts: %d ;  Position: %d %d",\
            acc_interval, acc_interrupt_count, dx.val1, dy.val1);
    LOG_INF("Send report time cost: %lld", send_report_duration);

    acc_interval = 0;
    acc_interrupt_count = 0;
  }

/* #if IS_ENABLED(CONFIG_SENSOR_LOG_LEVEL_DBG) */
    time_buffer = k_ticks_to_us_floor64(k_uptime_ticks());
    handler_duration = time_buffer - current_interrupt_time;
    /* LOG_DBG("interrupt interval (us): %lld ; idle: %lld ; handler time cost: %lld",\ */
    /*         interrupt_interval, idle_interval, handler_duration); */

    last_interrupt_time = current_interrupt_time;
/* #endif */
}

static int trackball_init() {
    struct sensor_trigger trigger = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    printk("trackball");
    if (sensor_trigger_set(trackball, &trigger, handle_trackball) < 0) {
        LOG_ERR("can't set trigger");
        return -EIO;
    };
    return 0;
}

SYS_INIT(trackball_init, APPLICATION, CONFIG_ZMK_KSCAN_INIT_PRIORITY);
