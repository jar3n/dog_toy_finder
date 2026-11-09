// Camera driver: initialises the OV2640/OV3660 sensor and grabs fresh frames.
#pragma once

#include "esp_camera.h"

// Start the camera in JPEG mode at VGA resolution. Falls back to QVGA in DRAM
// when no PSRAM is present. Returns true on success.
bool start_camera();

// Discard the buffered frame so the returned frame is taken after this call.
camera_fb_t *take_fresh_photo();
