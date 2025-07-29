/* Caps‑ and Num‑Lock LED indicator listener for ZMK
 * Works with any devicetree that aliases led‑caps / led‑num
 * SPDX‑License‑Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>

#include <zmk/events/hid_indicators_changed.h>
#include <zmk/hid_indicators.h>

/* Devicetree helpers ---------------------------------------------------- */
#define LEDS_NODE DT_NODELABEL(leds)
#define CAPS_ALIAS DT_ALIAS(led_caps)
#define NUM_ALIAS  DT_ALIAS(led_num)

static const struct device *const led_dev = DEVICE_DT_GET(LEDS_NODE);

/* HID‑indicator event callback ----------------------------------------- */
static int hid_led_cb(const zmk_event_t *eh)
{
    const zmk_hid_indicators_t ind =
        zmk_hid_indicators_get_current_profile();

#if DT_NODE_EXISTS(CAPS_ALIAS)
    if (ind & BIT(HID_USAGE_LED_CAPS_LOCK - 1)) {
        led_on(led_dev, DT_NODE_CHILD_IDX(CAPS_ALIAS));
    } else {
        led_off(led_dev, DT_NODE_CHILD_IDX(CAPS_ALIAS));
    }
#endif

#if DT_NODE_EXISTS(NUM_ALIAS)
    if (ind & BIT(HID_USAGE_LED_NUM_LOCK - 1)) {
        led_on(led_dev, DT_NODE_CHILD_IDX(NUM_ALIAS));
    } else {
        led_off(led_dev, DT_NODE_CHILD_IDX(NUM_ALIAS));
    }
#endif
    return 0;
}

ZMK_LISTENER(hid_led_listener, hid_led_cb);
ZMK_SUBSCRIPTION(hid_led_listener, zmk_hid_indicators_changed);

/* Ensure LED device is ready ------------------------------------------- */
static int leds_init(void)
{
    return device_is_ready(led_dev) ? 0 : -ENODEV;
}
SYS_INIT(leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
