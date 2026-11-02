#pragma once

// Copy this file to config/wifi_config.h and fill in your network details.
// wifi_config.h is ignored by git; never commit real credentials.
//
// The build creates wifi_config.h from this file automatically when it is
// missing or empty.

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Authentication mode. Use CYW43_AUTH_WPA2_AES_PSK for WPA2, or
// CYW43_AUTH_WPA_WPA2_PSK for a mixed WPA/WPA2 network.
#define WIFI_AUTH CYW43_AUTH_WPA2_AES_PSK

// Connection tuning. Timeouts are per attempt; retries are 1-based.
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define WIFI_CONNECT_RETRIES 3
#define WIFI_CONNECT_RETRY_DELAY_MS 1000

// Log mirror over UDP. The default broadcast address requires no knowledge of
// the computer's IP as long as it is on the same network. Set this to the
// computer's IPv4 address for unicast (more reliable through firewalls).
#define WIFI_LOG_TARGET_IP "255.255.255.255"
#define WIFI_LOG_TARGET_PORT 5005
