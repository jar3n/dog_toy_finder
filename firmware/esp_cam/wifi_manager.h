// Wi-Fi connection and mDNS advertisement helpers.
#pragma once

#include <WiFi.h>

// Join the configured Wi-Fi network. Returns true on success.
bool connect_wifi();

// Advertise this camera as esp32cam-<side>.local over mDNS.
void start_mdns();