# Wi-Fi and network logging

`wifi.c` / `wifi.h` wrap the Pico SDK CYW43 station setup:

- initialize the wireless chip
- join the configured SSID
- report connection state and the assigned IPv4 address

`wifi_log.c` / `wifi_log.h` mirror log output to a UDP socket so the same
messages appear on USB serial and over Wi-Fi. This is the low-level sink; see
`../ros/README.md` for publishing the same lines to ROS 2 with
`ros_log_printf()`.

`main` only needs:

```c
#include "wifi/wifi.h"
#include "wifi/wifi_log.h"

wifi_init();
wifi_log_printf("Hello from the Pico\n");
```

`wifi_init()` blocks while connecting (bounded by the configured timeout and
retry count), and starts the UDP log stream once connected. The rest of the
firmware keeps running if the connection fails; serial logging still works.

`wifi_log_write()` does the same as `wifi_log_printf()` but takes an
already-formatted buffer, so higher-level sinks can format once and forward the
same bytes (the ROS logger uses this).

## Configuration

Credentials and the log destination live in `config/wifi_config.h`, which is
ignored by git. When that file is missing or empty, CMake copies
`config/wifi_config.example.h` into place during configure. Edit the generated
file:

```c
#define WIFI_SSID "MyNetwork"
#define WIFI_PASSWORD "my-password"

#define WIFI_AUTH CYW43_AUTH_WPA2_AES_PSK

#define WIFI_CONNECT_TIMEOUT_MS 15000
#define WIFI_CONNECT_RETRIES 3
#define WIFI_CONNECT_RETRY_DELAY_MS 1000

#define WIFI_LOG_TARGET_IP "255.255.255.255"
#define WIFI_LOG_TARGET_PORT 5005
```

Do not commit real credentials. If you change the example after
`wifi_config.h` has already been created, update the local file manually.

The default `255.255.255.255` broadcasts logs on the local network, so no
computer IP is needed. For more reliable delivery through firewalls, set
`WIFI_LOG_TARGET_IP` to the computer's IPv4 address instead.

## Receiving the logs

Run the listener on the computer that is on the same network:

```bash
firmware/scripts/wifi_log_listen.py
```

Example output:

```text
Listening for Pico logs on udp://0.0.0.0:5005
192.168.1.42: wifi: logs mirrored to udp://255.255.255.255:5005
192.168.1.42: wifi: connected, ip=192.168.1.42
192.168.1.42: Hello World from the PICO
```

If nothing arrives, check that the host firewall allows UDP port 5005, or set
`WIFI_LOG_TARGET_IP` to the computer's unicast address.

Messages are truncated at 512 bytes by default; override
`WIFI_LOG_MAX_MESSAGE_LENGTH` in `wifi_config.h` if needed.

## Notes

- The Pico 2 W CYW43 radio is **2.4 GHz only**.
- WPA2-PSK is the default; use `CYW43_AUTH_WPA_WPA2_PSK` for a mixed
  WPA/WPA2 network. WPA3-only networks are not supported by this configuration.
- The UDP transport in `udp_transport/` sits on top of this Wi-Fi connection
  once micro-ROS is initialized.
