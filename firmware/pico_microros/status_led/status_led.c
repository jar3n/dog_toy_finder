// Built-in status LED implementation.
#include "status_led/status_led.h"

#include "config/status_led_config.h"

#if STATUS_LED_ENABLED

#include <stdbool.h>
#include <stdint.h>

#include "pico/cyw43_arch.h"
#include "pico/error.h"
#include "pico/stdlib.h"

// True once the CYW43 chip is available and the LED can be driven.
static bool available = false;

// True after status_led_set_ready(true); the LED then stays solid.
static bool ready = false;

// Logical LED state (true = lit) used by the blink pattern.
static bool lit = false;

// Time of the last blink toggle, in milliseconds since boot.
static uint32_t last_toggle_ms = 0;

/**
 * Drive the on-module LED, applying the configured polarity.
 *
 * The Pico 2 W LED is wired to the CYW43 wireless chip, so it must be set
 * through cyw43_arch_gpio_put() rather than a Pico GPIO.
 */
static void status_led_write(bool on) {
  cyw43_arch_gpio_put(
    CYW43_WL_GPIO_LED_PIN,
    STATUS_LED_ACTIVE_HIGH ? on : !on);
}

/**
 * Initialize the built-in LED and start blinking.
 *
 * Reuses an already initialized CYW43 chip (wifi_init() may have brought it
 * up first) and initializes it otherwise. The LED is left off so the first
 * status_led_task() call starts a clean blink.
 */
bool status_led_init(void) {
  if (!cyw43_is_initialized(&cyw43_state)) {
    if (cyw43_arch_init() != PICO_OK) {
      available = false;
      return false;
    }
  }

  available = true;
  ready = false;
  lit = false;
  status_led_write(false);
  last_toggle_ms = to_ms_since_boot(get_absolute_time());
  return true;
}

/**
 * Switch between the startup blink and the solid ready indication.
 */
void status_led_set_ready(bool is_ready) {
  if (!available) {
    return;
  }

  ready = is_ready;
  lit = is_ready;
  status_led_write(is_ready);
  last_toggle_ms = to_ms_since_boot(get_absolute_time());
}

/**
 * Return whether startup has finished and the LED is solid.
 */
bool status_led_ready(void) {
  return ready;
}

/**
 * Advance the startup blink pattern.
 *
 * Toggles once per half period. Does nothing once ready, so the main loop can
 * call it unconditionally without affecting the solid indication.
 */
void status_led_task(void) {
  if (!available || ready) {
    return;
  }

  // A full period is one on/off cycle, so each half toggles the LED.
  uint32_t half_period_ms = STATUS_LED_BLINK_PERIOD_MS / 2;
  if (half_period_ms == 0) {
    half_period_ms = 1;
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  if ((now - last_toggle_ms) >= half_period_ms) {
    lit = !lit;
    status_led_write(lit);
    last_toggle_ms = now;
  }
}

#else  // !STATUS_LED_ENABLED

// Disabled build: keep the API callable but do nothing.

bool status_led_init(void) {
  return false;
}

void status_led_set_ready(bool is_ready) {
  (void) is_ready;
}

bool status_led_ready(void) {
  return false;
}

void status_led_task(void) {
}

#endif  // STATUS_LED_ENABLED
