// Built-in status LED for the Pico 2 W.
//
// The LED is not a Pico GPIO: it belongs to the CYW43 wireless chip and is
// driven through the cyw43_arch API. The module blinks while the firmware is
// starting up, holds the LED on when startup succeeded, and turns it off when
// startup failed.
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Boot state shown by the built-in LED. */
typedef enum {
  /** Firmware is still starting up; the LED blinks. */
  STATUS_LED_STARTING = 0,
  /** Startup finished and every required device is up; the LED is solid. */
  STATUS_LED_READY,
  /** Startup finished but at least one step failed; the LED is off. */
  STATUS_LED_FAILED,
} status_led_state_t;

/**
 * Initialize the built-in LED and start blinking.
 *
 * Brings up the CYW43 chip when Wi-Fi has not done so yet (the LED lives on
 * that chip), drives the LED off, and puts the module in
 * STATUS_LED_STARTING. Safe to call once during boot, before wifi_init().
 *
 * @return true when the LED can be driven, false when it is disabled or the
 *         wireless chip could not be initialized.
 */
bool status_led_init(void);

/**
 * Set the boot state shown by the LED.
 *
 * STATUS_LED_STARTING blinks, STATUS_LED_READY lights it solid, and
 * STATUS_LED_FAILED leaves it off.
 */
void status_led_set_state(status_led_state_t state);

/** @return the current boot state. */
status_led_state_t status_led_state(void);

/**
 * Advance the startup blink pattern.
 *
 * Call regularly (at least every STATUS_LED_BLINK_PERIOD_MS / 2) while the
 * firmware is starting. Does nothing in the ready and failed states, so it is
 * cheap to keep calling from the main loop.
 */
void status_led_task(void);

#ifdef __cplusplus
}
#endif
