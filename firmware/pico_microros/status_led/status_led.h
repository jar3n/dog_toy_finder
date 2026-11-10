// Built-in status LED for the Pico 2 W.
//
// The LED is not a Pico GPIO: it belongs to the CYW43 wireless chip and is
// driven through the cyw43_arch API. The module blinks while the firmware is
// starting up and holds the LED on once status_led_set_ready(true) is called.
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the built-in LED and start blinking.
 *
 * Brings up the CYW43 chip when Wi-Fi has not done so yet (the LED lives on
 * that chip), drives the LED off, and marks the module available. Safe to
 * call once during boot, before wifi_init().
 *
 * @return true when the LED can be driven, false when it is disabled or the
 *         wireless chip could not be initialized.
 */
bool status_led_init(void);

/**
 * Switch between the startup blink and the ready indication.
 *
 * @param ready true = solid on (startup finished), false = blink again
 *              (for example after a later subsystem restart).
 */
void status_led_set_ready(bool ready);

/** @return true when the startup sequence has finished and the LED is solid. */
bool status_led_ready(void);

/**
 * Advance the startup blink pattern.
 *
 * Call regularly (at least every STATUS_LED_BLINK_PERIOD_MS / 2) while the
 * firmware is starting. Does nothing once the LED is ready, so it is cheap
 * to keep calling from the main loop.
 */
void status_led_task(void);

#ifdef __cplusplus
}
#endif
