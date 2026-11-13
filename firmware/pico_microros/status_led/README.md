# Built-in status LED

`status_led.c` / `status_led.h` drive the LED built into the Pico 2 W as a
boot indicator:

| LED | Meaning |
|---|---|
| Blinking | firmware is starting up (Wi-Fi join, micro-ROS, sensors) |
| Solid on | startup finished and every required device came up |
| Off | startup finished but at least one initialization step failed |

The firmware does not stop on a failure. It logs the failing step (USB serial,
UDP, and `/pico/log`) and keeps running in its degraded state, so the LED is
the quick indication and the log says what went wrong. A failure leaves the
LED off until the Pico is reset.

## Wiring

None. On the Pico 2 W the LED is not connected to a Pico GPIO: it is on the
CYW43 wireless chip (`CYW43_WL_GPIO_LED_PIN`, wireless GPIO 0). The driver
uses `cyw43_arch_gpio_put()`, so there is no entry in `../config/pinout.h`.
The same chip provides Wi-Fi, which is why `status_led_init()` can bring it up
before `wifi_init()` and `wifi_init()` reuses it.

## Configuration

Timing lives in `../config/status_led_config.h`.

| Setting | Default | Meaning |
|---|---|---|
| `STATUS_LED_ENABLED` | 1 | 1 = use the LED, 0 = compile the driver to no-ops |
| `STATUS_LED_ACTIVE_HIGH` | 1 | 1 = LED on when driven high (Pico 2 W) |
| `STATUS_LED_BLINK_PERIOD_MS` | 500 | startup blink period; one full on/off cycle |

## Usage

`dtf_pico_fw_main.c` calls, in order:

1. `status_led_init()` before Wi-Fi, so the LED blinks during the join.
2. `status_led_task()` from the main loop, and from `wifi_init()`'s connect
   wait via `wifi_set_poll_hook()` so the blink keeps moving during the
   (potentially tens of seconds) Wi-Fi association.
3. `status_led_set_state(STATUS_LED_READY)` when every initialization step
   succeeded, or `status_led_set_state(STATUS_LED_FAILED)` when any failed,
   just before the control loop starts.

`status_led_set_state(STATUS_LED_STARTING)` resumes blinking if a later
subsystem restart should look busy again.

## Notes

- `status_led_task()` is cheap and safe to call every loop; it does nothing
  once startup has finished.
- Do not toggle the LED from an interrupt. The CYW43 driver uses locks shared
  with the background Wi-Fi processing, so LED writes belong in the main
  thread.
