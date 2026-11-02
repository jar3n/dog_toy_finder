#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start mirroring log output to a UDP datagram socket.
 *
 * The destination is configured with WIFI_LOG_TARGET_IP and
 * WIFI_LOG_TARGET_PORT in config/wifi_config.h. A broadcast address such as
 * 255.255.255.255 works without knowing the computer's IP.
 *
 * Call after the Wi-Fi link is up. Logging still goes to USB serial even when
 * this function fails or is never called.
 *
 * @return true when the UDP log socket was created.
 */
bool wifi_log_init(void);

/**
 * Write a formatted log message to USB serial and, when initialized, UDP.
 *
 * Messages longer than WIFI_LOG_MAX_MESSAGE_LENGTH bytes are truncated.
 */
void wifi_log_printf(const char *format, ...)
  __attribute__((format(printf, 1, 2)));

/**
 * Write an already-formatted message to USB serial and, when initialized, UDP.
 *
 * Useful for log sinks that format once and forward the same bytes elsewhere
 * (for example, to a ROS publisher).
 */
void wifi_log_write(const char *message, size_t length);

#ifdef __cplusplus
}
#endif
