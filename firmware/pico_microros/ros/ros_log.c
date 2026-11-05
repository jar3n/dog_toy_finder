#include "ros/ros_log.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw/qos_profiles.h>

#include "config/ros_config.h"
#include "ros/ros_node.h"
#include "wifi/wifi_log.h"

#ifndef ROS_LOG_QUEUE_DEPTH
#define ROS_LOG_QUEUE_DEPTH 10
#endif

#ifndef ROS_LOG_MAX_MESSAGE_LENGTH
#define ROS_LOG_MAX_MESSAGE_LENGTH 512
#endif

static rcl_publisher_t publisher;
static bool publisher_ready = false;
static bool publish_error_reported = false;

static char node_name[] = ROS_NODE_NAME;
static char empty_string[] = "";

/**
 * Return whether the ROS log publisher has been created.
 */
bool ros_log_ready(void) {
  return publisher_ready;
}

/**
 * Create the best-effort rcl_interfaces/msg/Log publisher.
 *
 * Ensures the shared micro-ROS node exists (through ros_node_init()) and
 * publishes a confirmation line once the publisher is ready.
 *
 * @return true when log messages can be published to ROS.
 */
bool ros_log_init(void) {
  if (publisher_ready) {
    return true;
  }

  if (!ros_node_init()) {
    return false;
  }

  publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t options = rcl_publisher_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  options.qos.depth = ROS_LOG_QUEUE_DEPTH;

  rcl_ret_t status = rcl_publisher_init(
    &publisher,
    ros_node_handle(),
    ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
    ROS_LOG_TOPIC,
    &options);
  if (status != RCL_RET_OK) {
    wifi_log_printf("ros: log publisher init failed (%d)\n", (int) status);
    return false;
  }

  publisher_ready = true;
  ros_log_printf("ros: logs mirrored to %s\n", ROS_LOG_TOPIC);
  return true;
}

/**
 * Publish one pre-formatted message as an rcl_interfaces/msg/Log.
 *
 * Fills the timestamp, node name, INFO severity, and message fields, then
 * calls rcl_publish(). The first publish failure is reported through the
 * serial/UDP log so a broken agent does not spam every message.
 */
static void publish_log(char *message, size_t length) {
  if (!publisher_ready || length == 0) {
    return;
  }

  int64_t stamp_ns = ros_node_time_ns();

  rcl_interfaces__msg__Log log_message;
  memset(&log_message, 0, sizeof log_message);

  log_message.stamp.sec = (int32_t) (stamp_ns / 1000000000LL);
  log_message.stamp.nanosec = (uint32_t) (stamp_ns % 1000000000LL);
  log_message.level = rcl_interfaces__msg__Log__INFO;

  log_message.name.data = node_name;
  log_message.name.size = strlen(node_name);
  log_message.name.capacity = sizeof node_name;

  log_message.msg.data = message;
  log_message.msg.size = length;
  log_message.msg.capacity = length + 1;

  log_message.file.data = empty_string;
  log_message.file.size = 0;
  log_message.file.capacity = sizeof empty_string;

  log_message.function.data = empty_string;
  log_message.function.size = 0;
  log_message.function.capacity = sizeof empty_string;

  rcl_ret_t status = rcl_publish(&publisher, &log_message, NULL);
  if (status != RCL_RET_OK && !publish_error_reported) {
    publish_error_reported = true;
    wifi_log_printf(
      "ros: failed to publish a log message (%d)\n",
      (int) status);
  }
}

/**
 * Format a log line and send it to every enabled sink.
 *
 * Writes the same bytes to USB serial, UDP when Wi-Fi logging is active, and
 * ROS when ros_log_init() succeeded. Output is truncated to
 * ROS_LOG_MAX_MESSAGE_LENGTH bytes including the terminator.
 */
void ros_log_printf(const char *format, ...) {
  char message[ROS_LOG_MAX_MESSAGE_LENGTH];

  va_list args;
  va_start(args, format);
  int formatted = vsnprintf(message, sizeof message, format, args);
  va_end(args);

  if (formatted <= 0) {
    return;
  }

  size_t length = (size_t) formatted;
  if (length >= sizeof message) {
    length = sizeof message - 1;
  }

  wifi_log_write(message, length);
  publish_log(message, length);
}
