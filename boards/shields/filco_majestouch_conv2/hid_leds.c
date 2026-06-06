/*
 * Caps‑ and Num‑Lock indicator LEDs (USB‑only version)
 * SPDX‑License‑Identifier: MIT
 */
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>

#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/hid_indicators.h>
#include <zmk/endpoints.h>
#include <zmk/usb.h>

/* ──────────── Devicetree handles ────────────────────────────────────── */
#define LEDS_NODE DT_NODELABEL(leds)
#define CAPS_ALIAS DT_ALIAS(led_caps)
#define NUM_ALIAS  DT_ALIAS(led_num)

static const struct device *const led_dev = DEVICE_DT_GET(LEDS_NODE);

/* ──────────── Helpers ───────────────────────────────────────────────── */
static inline bool usb_active(void)
{
    struct zmk_endpoint_instance ep = zmk_endpoint_get_selected();
    return ep.transport == ZMK_TRANSPORT_USB && zmk_usb_is_powered();
}

static inline void leds_off(void)
{
#if DT_NODE_EXISTS(CAPS_ALIAS)
    led_off(led_dev, DT_NODE_CHILD_IDX(CAPS_ALIAS));
#endif
#if DT_NODE_EXISTS(NUM_ALIAS)
    led_off(led_dev, DT_NODE_CHILD_IDX(NUM_ALIAS));
#endif
}

/* ──────────── HID indicator listener ────────────────────────────────── */
static int hid_led_cb(const zmk_event_t *eh)
{
    if (!usb_active()) { leds_off(); return 0; }

    const zmk_hid_indicators_t ind =
        zmk_hid_indicators_get_current_profile();

#if DT_NODE_EXISTS(CAPS_ALIAS)
    (ind & BIT(HID_USAGE_LED_CAPS_LOCK - 1))
        ? led_on (led_dev, DT_NODE_CHILD_IDX(CAPS_ALIAS))
        : led_off(led_dev, DT_NODE_CHILD_IDX(CAPS_ALIAS));
#endif
#if DT_NODE_EXISTS(NUM_ALIAS)
    (ind & BIT(HID_USAGE_LED_NUM_LOCK - 1))
        ? led_on (led_dev, DT_NODE_CHILD_IDX(NUM_ALIAS))
        : led_off(led_dev, DT_NODE_CHILD_IDX(NUM_ALIAS));
#endif
    return 0;
}
ZMK_LISTENER(hid_led_listener, hid_led_cb);
ZMK_SUBSCRIPTION(hid_led_listener, zmk_hid_indicators_changed);

/* ──────────── Endpoint change listener (USB ↔ BLE) ──────────────────── */
static int usb_power_listener_cb(const zmk_event_t *eh)
{
    if (!usb_active()) { leds_off(); }
    return 0;
}
ZMK_LISTENER(usb_power_listener, usb_power_listener_cb);
ZMK_SUBSCRIPTION(usb_power_listener, zmk_endpoint_changed);

/* ──────────── Ensure LED device ready ───────────────────────────────── */
static int leds_init(void)
{
    return device_is_ready(led_dev) ? 0 : -ENODEV;
}
SYS_INIT(leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
