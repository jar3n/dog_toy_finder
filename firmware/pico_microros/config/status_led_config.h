#pragma once

// Built-in status LED settings. Edit this file in place.
//
// The Pico 2 W does not connect its LED to a Pico GPIO: it is wired to the
// CYW43 wireless chip and driven with
// cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ...). There is therefore no pin
// to set here; see ../config/pinout.h for the board pin map.
//
// Behaviour: while the firmware is still bringing up Wi-Fi, micro-ROS, and
// the sensors the LED blinks. Once startup finishes it stays on solid. A
// sensor that fails to initialize does not keep it blinking; the failure is
// logged instead and the robot keeps running in its degraded state.

// 1 = use the built-in LED, 0 = disable it entirely.
#define STATUS_LED_ENABLED 1

// 1 = LED on when the output is driven high (Pico 2 W default).
// 0 = active low, for boards wired the other way around.
#define STATUS_LED_ACTIVE_HIGH 1

// Blink period while starting up, in milliseconds. One full on/off cycle
// takes this long, so 500 means 250 ms on and 250 ms off.
#define STATUS_LED_BLINK_PERIOD_MS 500
