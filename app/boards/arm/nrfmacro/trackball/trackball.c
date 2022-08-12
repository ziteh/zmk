#include <drivers/sensor.h>
#include <logging/log.h>

#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <dt-bindings/zmk/mouse.h>

#define SCROLL_DIV_FACTOR 5


#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
// in ms
static int64_t last_update_time = 0;
static int64_t current_update_time = 0;
#endif

static struct sensor_value dx, dy;

const struct device *trackball = DEVICE_DT_GET(DT_INST(0, pixart_pmw33xx));

LOG_MODULE_REGISTER(trackball, CONFIG_SENSOR_LOG_LEVEL);

static void handle_trackball(const struct device *dev, const struct sensor_trigger *trig) {
#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
    current_update_time = k_uptime_get();
#endif

    // fetch latest position from sensor
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
    LOG_DBG("trackball %d %d", dx.val1, dy.val1);

    // update x, y position
    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_scroll_set(0, 0);

    const uint8_t layer = zmk_keymap_highest_layer_active();
    static int8_t scroll_ver_rem = 0, scroll_hor_rem = 0;
    if (layer == 2) {   // lower
      const int16_t total_hor = dx.val1 + scroll_hor_rem, total_ver = -(dy.val1 + scroll_ver_rem);
      scroll_hor_rem = total_hor % SCROLL_DIV_FACTOR;
      scroll_ver_rem = total_ver % SCROLL_DIV_FACTOR;
      zmk_hid_mouse_scroll_update(total_hor / SCROLL_DIV_FACTOR, total_ver / SCROLL_DIV_FACTOR);
    } else {
      zmk_hid_mouse_movement_update(CLAMP(dx.val1, INT8_MIN, INT8_MAX), CLAMP(dy.val1, INT8_MIN, INT8_MAX));
    }
    zmk_endpoints_send_mouse_report();
      
      /* test */ 
#if IS_ENABLED(CONFIG_ZMK_USB_LOGGING)
    LOG_DBG("update interval: %lld ; trackball: %d %d", (current_update_time-last_update_time), dx.val1, dy.val1);
    last_update_time = current_update_time;
#endif
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
