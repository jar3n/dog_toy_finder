#pragma once

// Shared main-loop tick for the Pico firmware, in milliseconds.
// 20 ms = 50 Hz. The cmd_vel service, encoder publishing and IMU publishing
// all derive their periods from this value so they stay phase-compatible.
#define CONTROL_PERIOD_MS 20
