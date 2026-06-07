/*
 * Custom behavior: toggle the BT device-picker mode.
 *
 * The combo (Ctrl+Alt+Fn) is bound to this behavior instead of `&mo 2`. When
 * the combo fires, ZMK invokes this behavior synchronously on the very same
 * code path that already resolves the picker layer's keys (profile selection
 * works) -- so unlike polling the layer state or listening for layer events,
 * this is GUARANTEED to run for the gesture. It simply calls into the LED
 * module, which owns the picker flag, the picker layer lock, and the timeout.
 *
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_bt_picker

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Implemented in indicator_leds.c. */
void filco_picker_toggle(void);

static int bp_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    filco_picker_toggle();
    return ZMK_BEHAVIOR_OPAQUE;
}

static int bp_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api bp_api = {
    .binding_pressed = bp_pressed,
    .binding_released = bp_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &bp_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
