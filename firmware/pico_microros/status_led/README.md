# Built-in status LED

`status_led.c` / `status_led.h` drive the LED built into the Pico 2 W as a
boot indicator:

| LED | Meaning |
|---|---|
| Blinking | firmware is starting up (Wi-Fi join, micro-ROS, sensors) |
| Solid on | startup finished; the main control loop is running |

A sensor or the micro-ROS agent failing to initialize does **not** keep the
LED blinking. Those failures are logged (USB serial, UDP, and `/pico/log`) and
the firmware continues in its degraded state, so solid always means "the boot
sequence is done", not "every optional device is healthy".

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
3. `status_led_set_ready(true)` once every initialization step has been
   attempted, just before the control loop starts.

`status_led_set_ready(false)` returns to blinking if a later subsystem is
restarted and the board should look busy again.

## Notes

- `status_led_task()` is cheap and safe to call every loop; it does nothing
  once the LED is ready.
- Do not toggle the LED from an interrupt. The CYW43 driver uses locks shared
  with the background Wi-Fi processing, so LED writes belong in the main
  thread.
