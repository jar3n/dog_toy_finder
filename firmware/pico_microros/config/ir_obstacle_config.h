#pragma once

// IR obstacle sensor settings. Edit this file in place.

// Topic carrying the obstacle detection result.
#define IR_OBSTACLE_TOPIC "/ir_obstacle/detected"

// Debounce window in milliseconds. The sensor output can chatter near the
// threshold, so changes shorter than this are ignored.
#define IR_OBSTACLE_DEBOUNCE_MS 50

// Active level: 1 = obstacle detected when the signal line is HIGH (KS0051
// default, see the module documentation), 0 = active low.
#define IR_OBSTACLE_ACTIVE_LEVEL 1

// Publish period in milliseconds.
#define IR_OBSTACLE_PUBLISH_PERIOD_MS 100
