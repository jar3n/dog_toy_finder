// Dog Toy Finder stereo-vision camera firmware (entry point).
//
// All the work lives in the modules next to this file:
//   camera_pins.h / config.h       pin map and per-device settings
//   camera_driver.h/.cpp           camera init and frame grab
//   wifi_manager.h/.cpp            Wi-Fi connection and mDNS
//   http_server.h/.cpp             HTTP endpoints (capture, stream, status, ...)
//
// This file only wires the modules together in setup() and reports status in
// loop(). Build with PlatformIO (esp32cam_left / esp32cam_right) or with the
// Arduino IDE board "AI Thinker ESP32-CAM".

#include <Arduino.h>

#include "wifi_config.h"   // WIFI_SSID / WIFI_PASSWORD (copy wifi_config.example.h)

#include "camera_pins.h"
#include "config.h"
#include "camera_driver.h"
#include "http_server.h"
#include "wifi_manager.h"

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.printf("ESP32-CAM stereo firmware, side: %s\n", CAMERA_SIDE);

    pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_LED_GPIO_NUM, LOW);

    if (!start_camera()) {
        return;
    }
    if (!connect_wifi()) {
        return;
    }
    start_mdns();
    start_http_server();
    Serial.printf("Ready: http://%s/capture  (mDNS: esp32cam-%s.local)\n",
                  WiFi.localIP().toString().c_str(), CAMERA_SIDE);
}

void loop() {
    static uint32_t last_report = 0;
    if (millis() - last_report > 10000) {
        last_report = millis();
        Serial.printf("%s eye: http://%s/capture | Wi-Fi %d dBm | %lu photos\n",
                      CAMERA_SIDE, WiFi.localIP().toString().c_str(),
                      WiFi.RSSI(), (unsigned long) photos_served);
    }
    delay(100);
}