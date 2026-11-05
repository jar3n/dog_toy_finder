// Wi-Fi connection and mDNS advertisement implementation.
#include "wifi_manager.h"

#include <Arduino.h>
#include <ESPmDNS.h>

#include "config.h"
#include "wifi_config.h"

// Join the configured Wi-Fi network. Returns true on success.
bool connect_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   // answer photo requests quickly
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.printf("Connected. %s eye: http://%s/capture\n",
                  CAMERA_SIDE, WiFi.localIP().toString().c_str());
    return true;
}

// Advertise this camera as esp32cam-<side>.local over mDNS.
void start_mdns() {
    if (MDNS.begin(String("esp32cam-") + CAMERA_SIDE)) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("mDNS: http://esp32cam-%s.local/\n", CAMERA_SIDE);
    } else {
        Serial.println("mDNS failed to start.");
    }
}