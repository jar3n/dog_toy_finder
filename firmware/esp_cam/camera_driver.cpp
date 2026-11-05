// Camera driver implementation.
#include "camera_driver.h"

#include <Arduino.h>

#include "camera_pins.h"
#include "config.h"

// Start the camera in JPEG mode at VGA resolution. Falls back to QVGA in DRAM
// when no PSRAM is present (some boards ship without it).
bool start_camera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    // VGA keeps each frame small (20-40 KB) while giving enough resolution for
    // stereo disparity; raise to SVGA/XGA if the host needs more detail.
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;              // 0-63, lower = better quality, bigger
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (!psramFound()) {
        Serial.println("No PSRAM found: using QVGA in DRAM.");
        config.frame_size = FRAMESIZE_QVGA;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x. Check the ribbon cable.\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
        Serial.println("Camera ready: OV3660.");
    } else {
        Serial.printf("Camera ready: sensor ID 0x%x.\n", s->id.PID);
    }
    s->set_vflip(s, CAMERA_VFLIP);
    s->set_hmirror(s, CAMERA_HMIRROR);
    return true;
}

// Discard the buffered frame so the returned frame is taken after this call.
camera_fb_t *take_fresh_photo() {
    camera_fb_t *old = esp_camera_fb_get();
    if (old) {
        esp_camera_fb_return(old);
    }
    return esp_camera_fb_get();
}