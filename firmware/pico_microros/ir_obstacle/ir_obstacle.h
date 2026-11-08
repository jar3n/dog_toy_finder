// Low-level driver for the Keyestudio KS0051 infrared obstacle avoidance sensor.
//
// The sensor has a digital output (TTL) that goes active when an obstacle is
// detected in front of it (2-40 cm, adjustable with the on-board trimmer). It
// also has an enable (EN) line; driving it high enables the sensor.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Initialize the signal and enable GPIOs. Returns true on success.
bool ir_obstacle_init(void);

// Read the raw (debounced) obstacle state. true = obstacle detected.
bool ir_obstacle_detected(void);

// Call periodically to update the debounce filter. Safe to call every loop.
void ir_obstacle_update(void);