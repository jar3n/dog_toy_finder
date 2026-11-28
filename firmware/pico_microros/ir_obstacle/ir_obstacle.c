// Low-level driver implementation.
#include "ir_obstacle.h"

#include <stdbool.h>

#include "pico/stdlib.h"

#include "config/ir_obstacle_config.h"
#include "config/pinout.h"

static bool last_raw = false;
static bool stable = false;
static uint32_t last_change_ms = 0;
static bool initialized = false;

bool ir_obstacle_init(void) {
    gpio_init(IR_OBSTACLE_SIGNAL_PIN);
    gpio_set_dir(IR_OBSTACLE_SIGNAL_PIN, GPIO_IN);
    gpio_pull_down(IR_OBSTACLE_SIGNAL_PIN);

#ifdef IR_OBSTACLE_ENABLE_PIN
    gpio_init(IR_OBSTACLE_ENABLE_PIN);
    gpio_set_dir(IR_OBSTACLE_ENABLE_PIN, GPIO_OUT);
    gpio_put(IR_OBSTACLE_ENABLE_PIN, 1);  // enable the sensor
#endif

    last_raw = gpio_get(IR_OBSTACLE_SIGNAL_PIN) == IR_OBSTACLE_ACTIVE_LEVEL;
    stable = last_raw;
    last_change_ms = to_ms_since_boot(get_absolute_time());
    initialized = true;
    return true;
}

void ir_obstacle_update(void) {
    if (!initialized) {
        return;
    }
    bool raw = gpio_get(IR_OBSTACLE_SIGNAL_PIN) == IR_OBSTACLE_ACTIVE_LEVEL;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (raw != last_raw) {
        last_raw = raw;
        last_change_ms = now;
    } else if (raw != stable &&
               (now - last_change_ms) >= IR_OBSTACLE_DEBOUNCE_MS) {
        stable = raw;
    }
}

bool ir_obstacle_detected(void) {
    if (!initialized) {
        return false;
    }
    return stable;
}
