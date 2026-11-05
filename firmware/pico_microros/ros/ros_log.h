#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize micro-ROS and start publishing logs to ROS 2.
 *
 * Calls ros_node_init() and creates a best-effort publisher for
 * rcl_interfaces/msg/Log on ROS_LOG_TOPIC. When the agent cannot be reached
 * this returns false and logging falls back to USB serial and UDP.
 *
 * @return true when ROS log publishing is available.
 */
bool ros_log_init(void);

/** @return true when the ROS log publisher is ready. */
bool ros_log_ready(void);

/**
 * Write a formatted log message to USB serial, UDP, and ROS 2.
 *
 * Every message is mirrored to USB serial and, when Wi-Fi logging is active,
 * to UDP. The same text is published to ROS when ros_log_init() succeeded.
 * Messages longer than ROS_LOG_MAX_MESSAGE_LENGTH bytes are truncated.
 */
void ros_log_printf(const char *format, ...)
  __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif
