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
 *   Device picker (Ctrl+Alt+Fn, latched ~20s)  -> blue+red lit solid.
 *   Low battery (<= 10%)                       -> brief red pulse every 10 s.
 *
 * The picker is implemented entirely here (no keymap layer/combo): a position
 * listener detects the Ctrl+Alt+Fn gesture, the 1-4 profile selection, 5 for
 * the USB/BLE transport toggle, and ESC cancel.
 *
 * Concurrency model: everything that matters runs on the system workqueue.
 * Event listeners only set one-byte flags and poke a single delayed-work
 * handler; the handler re-reads all keyboard state fresh (endpoint, BLE
 * profile, lock indicators, battery) on every run, decides the mode, and
 * renders the animation as a pure function of elapsed time. Renders are
 * idempotent, so an extra poke can never glitch a blink phase, and no cached
 * value can go stale (e.g. BLE profile state before settings load at boot).
 * Position, BLE and activity events are raised on (or marshalled to) the
 * system workqueue; battery and hid-indicator events may arrive from other
 * threads, but those listeners only call k_work_reschedule, which is safe
 * from any context and cannot lose a poke (a re-queued running item survives
 * the handler's own terminal reschedule). The int64 timestamps are touched
 * from the system workqueue only, so they cannot tear on this 32-bit MCU.
 *
 * Timeout model: the connecting (4 s) / advertising (60 s) blinks are bounded
 * from the start of the current "disconnected episode" (boot, link loss,
 * profile switch, bond add/clear, falling back from USB) - NOT from when the
 * blink mode was last entered - so an unrelated event cannot restart an
 * expired blink. ZMK itself advertises forever; these timeouts only govern
 * the LEDs, recreating the original keyboard's "give up and go dark".
 *
 * SPDX-License-Identifier: MIT
 */
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <dt-bindings/zmk/modifiers.h>

#include <zmk/activity.h>
#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>
#include <zmk/hid_indicators.h>
#include <zmk/usb.h>

#include <zmk/events/activity_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/position_state_changed.h>

/* ──────────── Tunables ──────────────────────────────────────────────── */
#define FN_POSITION      98    /* matrix position of the Fn key (&mo 1)       */
#define POS_ESC          0     /* ESC: cancels the picker                     */
#define POS_N1           18    /* number-row 1-4: select BT profile 0-3       */
#define POS_N2           19
#define POS_N3           20
#define POS_N4           21
#define POS_N5           22    /* 5: toggle output transport (BLE <-> USB)    */
#define MENU_TIMEOUT_MS  20000 /* auto-cancel the latched picker after this   */
#define LOW_BATT_PCT     10    /* red pulses at or below this state-of-charge */
/* The studio_unlock combo shares position 98: it can capture the Fn press and
 * re-raise it from the top of the listener chain up to 250 ms (its timeout)
 * later, which would toggle the picker twice on one physical press. Ignore
 * any Fn press inside this window after one we already acted on. */
#define FN_REPLAY_GUARD_MS 300
/* After a deliberate BT action (picker entry / profile switch) show the
 * connecting/advertising blink even on USB for this long; outside it, passive
 * advertising is hidden on USB so it doesn't mask the Caps/Num lock LEDs. */
#define BT_USER_WINDOW_MS (CONNECTING_MS + 5000)

#define CONNECTING_MS    4000  /* alternate this long while reconnecting      */
#define ADV_TIMEOUT_MS   60000 /* blue blinks this long while advertising     */
#define ALT_STEP_MS      1000  /* blue<->red alternation: 1 s per colour      */
#define ADV_STEP_MS      1000  /* advertising blink: 1 s off / 1 s blue       */
#define FLASH_STEP_MS    1000  /* success flash: 1 s both-off / 1 s both-on   */
#define SUCCESS_FLASHES  3
#define SUCCESS_TOTAL_MS (2 * SUCCESS_FLASHES * FLASH_STEP_MS)
#define LOWBATT_PULSE_MS 60    /* low-battery: brief red pulse ...            */
#define LOWBATT_PERIOD_MS 10000 /* ... at the end of every 10 s period        */

/* ──────────── Devicetree handles ────────────────────────────────────── */
#define LEDS_NODE DT_NODELABEL(leds)

static const struct device *const led_dev = DEVICE_DT_GET(LEDS_NODE);

#define BLUE_IDX DT_NODE_CHILD_IDX(DT_NODELABEL(blue_led))
#define RED_IDX  DT_NODE_CHILD_IDX(DT_NODELABEL(red_led))

/* ──────────── Inputs (written by listeners, read by the work handler) ─── */
static volatile bool v_menu;     /* device-picker mode active                 */
static volatile bool v_sleeping; /* deep sleep                                */
static volatile bool v_success;  /* force a success flash (picker re-select)  */

/* k_uptime of the last deliberate BT action; init "expired" so passive
 * advertising right after boot is hidden on USB. (sysworkq only) */
static int64_t bt_user_action_at = -BT_USER_WINDOW_MS;

/* ──────────── Animation state (handler-owned, sysworkq only) ───────────── */
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
static int64_t mode_start;    /* k_uptime_get() when cur_mode was entered     */
static int64_t bt_episode_at; /* when the current disconnected episode began
                               * (also re-armed by deliberate picker actions
                               * in position_cb - same sysworkq thread)       */
static bool prev_connected;   /* previous poll results, for edge detection    */
static bool prev_open;
static bool prev_usb;
static int prev_index;

static void indicator_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(indicator_work, indicator_work_handler);

static void picker_timeout_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(picker_timeout, picker_timeout_handler);

/* ──────────── Low-level LED helper ─────────────────────────────────────── */
static inline void set_leds(bool blue, bool red) {
    blue ? led_on(led_dev, BLUE_IDX) : led_off(led_dev, BLUE_IDX);
    red ? led_on(led_dev, RED_IDX) : led_off(led_dev, RED_IDX);
}

static inline bool usb_active(void) {
    struct zmk_endpoint_instance ep = zmk_endpoint_get_selected();
    return ep.transport == ZMK_TRANSPORT_USB && zmk_usb_is_powered();
}

/* Render the USB lock indicators (led-num -> blue, led-caps -> red),
 * preserving the prior USB-only behavior. */
static void render_locks(void) {
    const zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    bool caps = ind & BIT(HID_USAGE_LED_CAPS_LOCK - 1);
    bool num = ind & BIT(HID_USAGE_LED_NUM_LOCK - 1);
    set_leds(num, caps);
}

/* True only for a short window after a deliberate BT action, so the passive
 * advertising/reconnect blink can show on USB just after the user acts. */
static inline bool bt_user_window(int64_t now) {
    return (now - bt_user_action_at) < BT_USER_WINDOW_MS;
}

/* Decide which mode the current inputs call for; earlier checks win. */
static enum ind_mode target_mode(int64_t now, bool success_edge, bool usb, bool connected,
                                 bool open) {
    if (v_sleeping) {
        return M_SLEEP;
    }
    /* Picker: explicit user gesture, shown even on USB. */
    if (v_menu) {
        return M_MENU;
    }
    /* BT signals OUTRANK the USB lock indicators, so a connection/pairing is
     * visible even while wired (caps/num would otherwise mask them). They are
     * all transient/time-bounded, so the lock indicators reappear afterwards. */
    if (success_edge || (cur_mode == M_SUCCESS && (now - mode_start) < SUCCESS_TOTAL_MS)) {
        return M_SUCCESS;
    }
    /* Show the passive advertising/reconnect blink on BLE always, but on USB
     * only briefly after a deliberate BT action. Both blinks are bounded by
     * the episode clock, so once they expire nothing can restart them short
     * of a new disconnected episode. */
    if (!connected && (!usb || bt_user_window(now))) {
        if (open) {
            /* Advertising for a new pairing; give up after the timeout. */
            if ((now - bt_episode_at) < ADV_TIMEOUT_MS) {
                return M_ADVERTISING;
            }
        } else if ((now - bt_episode_at) < CONNECTING_MS) {
            /* Bonded but not connected: reconnecting. Alternate, then give up. */
            return M_CONNECTING;
        }
    }
    /* No active BT signal: USB -> Caps/Num lock indicators; else idle (off or
     * the low-battery pulse). */
    if (usb) {
        return M_LOCK;
    }
    return M_REST;
}

static void indicator_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    const int64_t now = k_uptime_get();

    /* Poll fresh state every run; no cache to go stale. All getters are safe
     * before BT/USB are up (they return false/0 until then). */
    const bool usb = usb_active();
    const bool connected = zmk_ble_active_profile_is_connected();
    const bool open = zmk_ble_active_profile_is_open();
    const int index = zmk_ble_active_profile_index();

    /* Edge detection, single-threaded here. A success edge is consumed every
     * run, so it can neither fire stale later nor latch a mode forever. */
    bool success_edge = v_success;
    v_success = false;
    if (connected && !prev_connected) {
        success_edge = true; /* connection just succeeded */
    }
    /* A new disconnected episode (re)arms the connecting/advertising clock:
     * link loss, profile switch, bond add/clear, or falling back from USB. */
    if (!connected && (prev_connected || open != prev_open || index != prev_index ||
                       (prev_usb && !usb))) {
        bt_episode_at = now;
    }
    /* A success flash can mask a disconnect that happened beneath it; when the
     * flash ends still disconnected, give the reconnect blink a full window. */
    if (cur_mode == M_SUCCESS && !success_edge && !connected &&
        (now - mode_start) >= SUCCESS_TOTAL_MS) {
        bt_episode_at = now;
    }
    prev_connected = connected;
    prev_open = open;
    prev_usb = usb;
    prev_index = index;

    enum ind_mode target = target_mode(now, success_edge, usb, connected, open);
    if (target != cur_mode) {
        cur_mode = target;
        mode_start = now;
    } else if (success_edge && cur_mode == M_SUCCESS) {
        mode_start = now; /* new success during an ongoing flash: restart it */
    }

    /* Render purely from elapsed time and reschedule for the next phase
     * boundary. Event pokes just repeat the current phase harmlessly. */
    const int64_t elapsed = now - mode_start;

    switch (cur_mode) {
    case M_SLEEP:
        set_leds(false, false);
        return; /* no reschedule; an event will wake us */

    case M_LOCK:
        render_locks();
        return; /* event-driven only */

    case M_MENU:
        /* Solid both. Entry/exit and the 20s auto-cancel are owned by
         * picker_set()/picker_timeout; the handler just renders. */
        set_leds(true, true);
        return;

    case M_SUCCESS: {
        /* Phases: off, on, off, on, ... (SUCCESS_FLASHES on-phases), then
         * target_mode() drops the mode once SUCCESS_TOTAL_MS has elapsed. */
        bool on = ((elapsed / FLASH_STEP_MS) % 2) == 1;
        set_leds(on, on);
        k_work_reschedule(&indicator_work, K_MSEC(FLASH_STEP_MS - (elapsed % FLASH_STEP_MS)));
        return;
    }

    case M_CONNECTING: {
        /* Start dark, then alternate blue<->red until the episode clock runs
         * out (wake exactly then, even mid-phase). */
        int64_t phase = elapsed / ALT_STEP_MS;
        if (phase == 0) {
            set_leds(false, false);
        } else {
            bool blue_phase = (phase % 2) == 1;
            set_leds(blue_phase, !blue_phase);
        }
        int64_t next = ALT_STEP_MS - (elapsed % ALT_STEP_MS);
        int64_t left = bt_episode_at + CONNECTING_MS - now;
        k_work_reschedule(&indicator_work, K_MSEC(MAX(1, MIN(next, left))));
        return;
    }

    case M_ADVERTISING: {
        bool on = ((elapsed / ADV_STEP_MS) % 2) == 1; /* off, blue, off, ... */
        set_leds(on, false);
        int64_t next = ADV_STEP_MS - (elapsed % ADV_STEP_MS);
        int64_t left = bt_episode_at + ADV_TIMEOUT_MS - now;
        k_work_reschedule(&indicator_work, K_MSEC(MAX(1, MIN(next, left))));
        return;
    }

    case M_REST:
    default:
        /* Idle on battery: dark, plus a brief red pulse at the end of every
         * 10 s period while the battery is low. Polled (not event-cached) so
         * a boot with already-flat cells - which never raises a state-changed
         * event because the SoC never moves off 0 - still warns. */
        if (zmk_battery_state_of_charge() <= LOW_BATT_PCT) {
            int64_t pos = elapsed % LOWBATT_PERIOD_MS;
            bool pulse = pos >= (LOWBATT_PERIOD_MS - LOWBATT_PULSE_MS);
            set_leds(false, pulse);
            k_work_reschedule(&indicator_work,
                              K_MSEC(pulse ? LOWBATT_PERIOD_MS - pos
                                           : LOWBATT_PERIOD_MS - LOWBATT_PULSE_MS - pos));
        } else {
            set_leds(false, false); /* no reschedule until the next event */
        }
        return;
    }
}

static inline void poke(void) { k_work_reschedule(&indicator_work, K_NO_WAIT); }

/* ──────────── Picker control (fully firmware-owned, no keymap layer) ────── */
/* The whole picker lives in the LED firmware, because layer-based selection
 * proved unreliable on this board (an externally-locked layer didn't survive
 * the Fn release - the LED flag latched but the layer did not). position_cb
 * below detects entry (Fn + Ctrl+Alt), selection (1-4 -> zmk_ble_prof_select),
 * 5 (transport toggle) and cancel (ESC). picker_set() owns the LED menu flag
 * and the 20s auto-cancel - both keyed on v_menu, so the LED and the function
 * can't decouple. */
static void picker_set(bool on) {
    if (on == v_menu) {
        return;
    }
    v_menu = on;
    if (on) {
        bt_user_action_at = k_uptime_get(); /* deliberate BT action */
        set_leds(true, true); /* render now, independent of the work handler */
        k_work_reschedule(&picker_timeout, K_MSEC(MENU_TIMEOUT_MS));
    } else {
        k_work_cancel_delayable(&picker_timeout);
    }
    poke();
}

static void picker_toggle(void) { picker_set(!v_menu); }

static void picker_timeout_handler(struct k_work *work) {
    ARG_UNUSED(work);
    picker_set(false);
}

/* ──────────── Picker: firmware-observed gesture + selection (no combo) ──── */
/* ENTRY: Fn pressed while Ctrl+Alt held. No combo, so Ctrl/Alt are never
 * captured -> zero latency (Ctrl+click works) and no timing window; they reach
 * the host during the gesture (harmless).
 * SELECTION/CANCEL: while the picker is latched, 1-4 pick a BT profile, 5
 * toggles the output transport (BLE <-> USB), and ESC cancels - all in
 * firmware (no layer needed). We swallow both the press AND the matching
 * release of those keys (ZMK_EV_EVENT_HANDLED stops propagation; events are
 * stack-allocated in this ZMK, so nothing needs freeing), so the keymap never
 * sees a stray digit or an unbalanced release. This relies on this listener
 * being linked before the keymap listener (shield sources are added during
 * find_package(Zephyr), before zmk's own) - a build-system convention, not a
 * contract; if it ever flipped, the action would still register but the digit
 * would also reach the host. */

/* Picker keys we may swallow, mapped to a bit each. */
static int picker_slot(uint32_t position) {
    switch (position) {
    case POS_ESC: return 0;
    case POS_N1:  return 1;
    case POS_N2:  return 2;
    case POS_N3:  return 3;
    case POS_N4:  return 4;
    case POS_N5:  return 5;
    default:      return -1;
    }
}

static uint8_t swallowed;  /* slots whose press we ate; eat the release too  */
static int64_t fn_acted_at = -FN_REPLAY_GUARD_MS;

static int position_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (!ev->state) {
        /* Key release: swallow it iff we swallowed its press. */
        int slot = picker_slot(ev->position);
        if (slot >= 0 && (swallowed & BIT(slot))) {
            swallowed &= ~BIT(slot);
            return ZMK_EV_EVENT_HANDLED;
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (v_menu) {
        int slot = picker_slot(ev->position);
        int profile = -1;
        switch (ev->position) {
        case POS_N1: profile = 0; break;
        case POS_N2: profile = 1; break;
        case POS_N3: profile = 2; break;
        case POS_N4: profile = 3; break;
        case POS_N5:
            zmk_endpoint_toggle_preferred_transport(); /* BLE <-> USB */
            if (zmk_endpoint_get_preferred_transport() == ZMK_TRANSPORT_BLE) {
                /* Deliberately switched to BLE: show its connect blink with a
                 * full window, even if we stay on USB as the fallback. */
                int64_t t = k_uptime_get();
                bt_user_action_at = t;
                bt_episode_at = t;
            }
            picker_set(false); /* toggle output + exit */
            swallowed |= BIT(slot);
            return ZMK_EV_EVENT_HANDLED;
        case POS_ESC:
            picker_set(false); /* cancel */
            swallowed |= BIT(slot);
            return ZMK_EV_EVENT_HANDLED;
        default: break;
        }
        if (profile >= 0) {
            int64_t t = k_uptime_get();
            bt_user_action_at = t; /* deliberate BT action */
            /* Re-arm the blink window here: re-selecting the CURRENT profile
             * raises no event, so the handler's edge detection alone would
             * leave an expired episode dark despite the deliberate gesture. */
            bt_episode_at = t;
            zmk_ble_prof_select(profile);
            /* Re-selecting the already-connected profile fires no BLE event,
             * so force the success flash here (the original flashes 3x even
             * when the chosen profile is the current, already-connected one). */
            if (zmk_ble_active_profile_index() == profile &&
                zmk_ble_active_profile_is_connected()) {
                v_success = true;
            }
            picker_set(false); /* select + exit */
            swallowed |= BIT(slot);
            return ZMK_EV_EVENT_HANDLED;
        }
    }

    if (ev->position == FN_POSITION) {
        zmk_mod_flags_t mods = zmk_hid_get_explicit_mods();
        if ((mods & (MOD_LCTL | MOD_RCTL)) && (mods & (MOD_LALT | MOD_RALT))) {
            int64_t now = k_uptime_get();
            if ((now - fn_acted_at) >= FN_REPLAY_GUARD_MS) {
                fn_acted_at = now;
                picker_toggle();
            }
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(ind_position, position_cb);
ZMK_SUBSCRIPTION(ind_position, zmk_position_state_changed);

/* ──────────── Event listeners ──────────────────────────────────────────── */
/* The handler polls everything fresh, so most events only need to run it. */
static int poke_cb(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    poke();
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(ind_poke, poke_cb);
ZMK_SUBSCRIPTION(ind_poke, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(ind_poke, zmk_hid_indicators_changed);
ZMK_SUBSCRIPTION(ind_poke, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(ind_poke, zmk_battery_state_changed);

static int activity_cb(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev) {
        bool sleeping = ev->state == ZMK_ACTIVITY_SLEEP;
        v_sleeping = sleeping;
        if (sleeping) {
            /* Entering deep sleep: the activity work item powers the system
             * off right after raising this event, before any queued poke
             * could run - douse the LEDs synchronously or they would stay
             * lit through System OFF. (Unreachable unless CONFIG_ZMK_SLEEP
             * is enabled, but kept correct for when it is.) */
            set_leds(false, false);
        }
        poke();
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(ind_activity, activity_cb);
ZMK_SUBSCRIPTION(ind_activity, zmk_activity_state_changed);

/* ──────────── Init ─────────────────────────────────────────────────────── */
static int indicator_init(void) {
    if (!device_is_ready(led_dev)) {
        return -ENODEV;
    }
    /* prev_* default to "disconnected at boot"; the boot reconnect/pairing
     * blink runs on the episode clock from now. The first handler runs poll
     * everything else fresh, so nothing needs seeding here. */
    bt_episode_at = k_uptime_get();
    poke();
    return 0;
}
SYS_INIT(indicator_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
