/*
 * Status LEDs for the Filco Majestouch Convertible 2 daughterboard.
 *
 * Two discrete LEDs (blue = P1.13, red = P1.10) serve two roles, arbitrated by
 * the active endpoint so the two modes can never fight over the GPIOs:
 *
 *   USB endpoint  -> LOCK mode   : Caps Lock -> red, Num Lock -> blue.
 *   BLE endpoint  -> STATUS mode : recreate the original Convertible 2 BT signals
 *                                  plus a low-battery indicator.
 *
 * Recreated BLE signals (see project notes):
 *   Connecting (reconnecting to a bonded host) -> blue<->red alternate, ~4 s.
 *   Advertising / pairing (open slot, no bond) -> blue blink, up to 60 s.
 *   Connection succeeded                       -> blue+red flash 3x together.
 *   Pairing/connect failed (timeout)           -> LEDs go dark.
 *   Device picker (Ctrl+Alt+Fn, latched layer) -> blue+red lit solid.
 *   Low battery (<= 10%)                       -> brief red pulse every 10 s.
 *
 * Concurrency model: event listeners ONLY update input flags and then poke a
 * single delayed-work handler. That handler is the only code that touches the
 * LEDs or the animation state, so there is no cross-context race on LED state.
 *
 * SPDX-License-Identifier: MIT
 */
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zmk/activity.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>
#include <zmk/keymap.h>
#include <zmk/usb.h>

#include <zmk/events/activity_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/layer_state_changed.h>

/* ──────────── Tunables ──────────────────────────────────────────────── */
#define MENU_LAYER_INDEX 2     /* dedicated BT picker layer (Ctrl+Alt+Fn)     */
#define MENU_TIMEOUT_MS  20000 /* auto-cancel the latched picker after this   */
#define LOW_BATT_PCT     10    /* red pulses at or below this state-of-charge */

#define CONNECTING_MS    4000  /* alternate this long while reconnecting      */
#define ADV_TIMEOUT_MS   60000 /* blue blinks this long while advertising     */
#define ALT_STEP_MS      1000  /* blue<->red alternation: 1 s per colour       */
#define ADV_ON_MS        1000  /* advertising blue blink: 1 s on / 1 s off    */
#define ADV_OFF_MS       1000
#define FLASH_ON_MS      1000  /* success flash: 1 s both-off / 1 s both-on   */
#define FLASH_OFF_MS     1000
#define SUCCESS_FLASHES  3
#define LOWBATT_PULSE_MS 60    /* low-battery: brief red pulse ...            */
#define LOWBATT_GAP_MS   (10000 - LOWBATT_PULSE_MS) /* ... once every ~10 s   */

/* ──────────── Devicetree handles ────────────────────────────────────── */
#define LEDS_NODE DT_NODELABEL(leds)

static const struct device *const led_dev = DEVICE_DT_GET(LEDS_NODE);

#define BLUE_IDX DT_NODE_CHILD_IDX(DT_NODELABEL(blue_led))
#define RED_IDX  DT_NODE_CHILD_IDX(DT_NODELABEL(red_led))

/* ──────────── Inputs (written by listeners, read by the work handler) ─── */
static volatile bool v_usb_mode;   /* USB endpoint selected & powered        */
static volatile bool v_connected;  /* active BLE profile connected           */
static volatile bool v_open;       /* active BLE profile open (no bond)       */
static volatile bool v_menu;       /* device-picker layer active             */
static volatile bool v_sleeping;   /* deep sleep                             */
static volatile bool v_low_batt;   /* battery <= threshold                   */
static volatile bool v_success;    /* one-shot: connection just succeeded     */
static volatile zmk_hid_indicators_t v_indicators; /* last HID lock state     */

static bool prev_connected;        /* edge detector for v_success            */

/* ──────────── Animation state (handler-owned) ──────────────────────────── */
enum ind_mode {
    M_SLEEP,
    M_LOCK,
    M_MENU,
    M_SUCCESS,
    M_CONNECTING,
    M_ADVERTISING,
    M_REST,
};

static enum ind_mode cur_mode = M_REST;
static int step;             /* step within the current animation            */
static int64_t mode_start;   /* k_uptime_get() when cur_mode was entered      */

static void indicator_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(indicator_work, indicator_work_handler);

/* ──────────── Low-level LED helper ─────────────────────────────────────── */
static inline void set_leds(bool blue, bool red) {
    blue ? led_on(led_dev, BLUE_IDX) : led_off(led_dev, BLUE_IDX);
    red ? led_on(led_dev, RED_IDX) : led_off(led_dev, RED_IDX);
}

static inline bool usb_active(void) {
    struct zmk_endpoint_instance ep = zmk_endpoint_get_selected();
    return ep.transport == ZMK_TRANSPORT_USB && zmk_usb_is_powered();
}

/* Render the USB lock indicators from the last HID state (led-num -> blue,
 * led-caps -> red), preserving the prior USB-only behavior. */
static void render_locks(void) {
    const zmk_hid_indicators_t ind = v_indicators;
    bool caps = ind & BIT(HID_USAGE_LED_CAPS_LOCK - 1);
    bool num = ind & BIT(HID_USAGE_LED_NUM_LOCK - 1);
    set_leds(num, caps);
}

/* Decide which mode the current inputs call for. Pure function of inputs +
 * elapsed time; earlier checks win. */
static enum ind_mode target_mode(int64_t now) {
    if (v_sleeping) {
        return M_SLEEP;
    }
    /* The picker is an explicit user gesture: acknowledge it even on USB,
     * where the LEDs are otherwise the lock indicators. */
    if (v_menu) {
        return M_MENU;
    }
    if (v_usb_mode) {
        return M_LOCK;
    }
    /* Success flash: latched until its 3 cycles finish. */
    if (v_success || (cur_mode == M_SUCCESS && step < 2 * SUCCESS_FLASHES)) {
        return M_SUCCESS;
    }
    if (v_connected) {
        return M_REST; /* connected & idle: quiet (plus low-batt pulse) */
    }
    if (v_open) {
        /* Advertising for a new pairing; give up after the timeout. */
        if (cur_mode == M_ADVERTISING && (now - mode_start) >= ADV_TIMEOUT_MS) {
            return M_REST;
        }
        return M_ADVERTISING;
    }
    /* Bonded but not connected: reconnecting. Alternate, then give up. */
    if (cur_mode == M_CONNECTING && (now - mode_start) >= CONNECTING_MS) {
        return M_REST;
    }
    return M_CONNECTING;
}

static void indicator_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    const int64_t now = k_uptime_get();

    enum ind_mode target = target_mode(now);

    /* A higher-priority mode consumes any pending success edge so it can't
     * fire stale later (e.g. connected while on USB, then switched to BLE). */
    if (target != M_SUCCESS) {
        v_success = false;
    }

    if (target != cur_mode) {
        cur_mode = target;
        step = 0;
        mode_start = now;
        if (target == M_SUCCESS) {
            v_success = false; /* consume the edge */
        }
        if (target == M_MENU) {
            /* Lock the picker layer so it survives the momentary chord's
             * release: &mo's non-locking deactivate cannot clear a locked
             * layer (see set_layer_state in keymap.c), which latches the
             * picker from the confirmed-working momentary combo. */
            zmk_keymap_layer_activate(zmk_keymap_layer_index_to_id(MENU_LAYER_INDEX), true);
        }
    }

    switch (cur_mode) {
    case M_SLEEP:
        set_leds(false, false);
        return; /* no reschedule; an event will wake us */

    case M_LOCK:
        render_locks();
        return; /* event-driven only */

    case M_MENU:
        set_leds(true, true);
        /* Auto-cancel the latched picker after the dwell. Force-deactivate
         * (locking=true) clears the lock we set on entry. Doing this from a
         * work handler mirrors how ZMK's own sticky-key timer releases
         * layers; the layer-off event re-evaluates us to the BLE state. */
        if (now - mode_start >= MENU_TIMEOUT_MS) {
            zmk_keymap_layer_deactivate(zmk_keymap_layer_index_to_id(MENU_LAYER_INDEX), true);
        } else {
            k_work_reschedule(&indicator_work, K_MSEC(MENU_TIMEOUT_MS - (now - mode_start)));
        }
        return;

    case M_SUCCESS: {
        /* Start dark: off, on, off, on, ... (SUCCESS_FLASHES on-phases).
         * target_mode() drops us out of M_SUCCESS once step reaches the
         * count, so the final on-phase still keeps its full duration. */
        bool on = (step % 2) == 1;
        set_leds(on, on);
        step++;
        k_work_reschedule(&indicator_work, K_MSEC(on ? FLASH_ON_MS : FLASH_OFF_MS));
        return;
    }

    case M_CONNECTING: {
        /* Start dark, then alternate blue<->red. */
        if (step == 0) {
            set_leds(false, false);
        } else {
            bool blue_phase = (step % 2) == 1;
            set_leds(blue_phase, !blue_phase);
        }
        step++;
        k_work_reschedule(&indicator_work, K_MSEC(ALT_STEP_MS));
        return;
    }

    case M_ADVERTISING: {
        bool on = (step % 2) == 1; /* start dark: off, blue, off, blue, ... */
        set_leds(on, false);
        step++;
        k_work_reschedule(&indicator_work, K_MSEC(on ? ADV_ON_MS : ADV_OFF_MS));
        return;
    }

    case M_REST:
    default:
        if (v_low_batt) {
            bool pulse = (step % 2) == 1; /* start with the off gap */
            set_leds(false, pulse);
            step++;
            k_work_reschedule(&indicator_work,
                              K_MSEC(pulse ? LOWBATT_PULSE_MS : LOWBATT_GAP_MS));
        } else {
            set_leds(false, false);
        }
        return;
    }
}

static inline void poke(void) { k_work_reschedule(&indicator_work, K_NO_WAIT); }

/* ──────────── Event listeners (input only, then poke) ──────────────────── */
static int endpoint_cb(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    v_usb_mode = usb_active();
    poke();
    return 0;
}
ZMK_LISTENER(ind_endpoint, endpoint_cb);
ZMK_SUBSCRIPTION(ind_endpoint, zmk_endpoint_changed);

static int hid_cb(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    v_indicators = zmk_hid_indicators_get_current_profile();
    poke();
    return 0;
}
ZMK_LISTENER(ind_hid, hid_cb);
ZMK_SUBSCRIPTION(ind_hid, zmk_hid_indicators_changed);

static int ble_cb(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    bool connected = zmk_ble_active_profile_is_connected();
    v_open = zmk_ble_active_profile_is_open();
    if (connected && !prev_connected) {
        v_success = true; /* rising edge: connection just succeeded */
    }
    prev_connected = connected;
    v_connected = connected;
    poke();
    return 0;
}
ZMK_LISTENER(ind_ble, ble_cb);
ZMK_SUBSCRIPTION(ind_ble, zmk_ble_active_profile_changed);

static int battery_cb(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev) {
        v_low_batt = ev->state_of_charge <= LOW_BATT_PCT;
        poke();
    }
    return 0;
}
ZMK_LISTENER(ind_battery, battery_cb);
ZMK_SUBSCRIPTION(ind_battery, zmk_battery_state_changed);

static int activity_cb(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev) {
        v_sleeping = ev->state == ZMK_ACTIVITY_SLEEP;
        poke();
    }
    return 0;
}
ZMK_LISTENER(ind_activity, activity_cb);
ZMK_SUBSCRIPTION(ind_activity, zmk_activity_state_changed);

static int layer_cb(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    v_menu = zmk_keymap_layer_active(zmk_keymap_layer_index_to_id(MENU_LAYER_INDEX));
    poke();
    return 0;
}
ZMK_LISTENER(ind_layer, layer_cb);
ZMK_SUBSCRIPTION(ind_layer, zmk_layer_state_changed);

/* ──────────── Init ─────────────────────────────────────────────────────── */
static int indicator_init(void) {
    if (!device_is_ready(led_dev)) {
        return -ENODEV;
    }
    v_usb_mode = usb_active();
    v_connected = zmk_ble_active_profile_is_connected();
    v_open = zmk_ble_active_profile_is_open();
    prev_connected = v_connected; /* no spurious success flash at boot */
    v_indicators = zmk_hid_indicators_get_current_profile();
    poke();
    return 0;
}
SYS_INIT(indicator_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
