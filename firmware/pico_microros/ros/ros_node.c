#include "ros/ros_node.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/custom_transport.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "config/ros_config.h"
#include "udp_transport/picow_udp_transport.h"
#include "wifi/wifi.h"
#include "wifi/wifi_log.h"

#ifndef ROS_INIT_ATTEMPTS
#define ROS_INIT_ATTEMPTS 3
#endif

#ifndef ROS_INIT_RETRY_DELAY_MS
#define ROS_INIT_RETRY_DELAY_MS 1000
#endif

#ifndef ROS_AGENT_PING_TIMEOUT_MS
#define ROS_AGENT_PING_TIMEOUT_MS 100
#endif

#ifndef ROS_TIME_SYNC_TIMEOUT_MS
#define ROS_TIME_SYNC_TIMEOUT_MS 1000
#endif

static bool node_ready = false;
static rclc_support_t support;
static rcl_node_t node;
static picow_udp_transport_context_t transport_context = {
  .agent_ip = ROS_AGENT_IP,
  .agent_port = ROS_AGENT_PORT,
};

/**
 * Return whether the shared micro-ROS node has been created.
 */
bool ros_node_ready(void) {
  return node_ready;
}

/**
 * Return the shared micro-ROS node, or NULL before ros_node_init() succeeds.
 */
rcl_node_t *ros_node_handle(void) {
  return node_ready ? &node : NULL;
}

/**
 * Return the current time in nanoseconds for message headers.
 *
 * Uses the agent-synchronized epoch when available and falls back to
 * microseconds since boot when no time sync has happened.
 */
int64_t ros_node_time_ns(void) {
  int64_t stamp_ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : 0;
  if (stamp_ns <= 0) {
    stamp_ns = (int64_t) time_us_64() * 1000;
  }
  return stamp_ns;
}

/**
 * Connect the shared micro-ROS session and create the node.
 *
 * Checks Wi-Fi, installs the custom UDP transport, pings the agent up to
 * ROS_INIT_ATTEMPTS times, initializes rcl, and creates a single node with
 * rosout disabled. On success it also synchronizes the session clock.
 * Repeated calls return true immediately once connected.
 *
 * @return true when the node is ready to create publishers.
 */
bool ros_node_init(void) {
  if (node_ready) {
    return true;
  }

  if (!wifi_is_connected()) {
    wifi_log_printf("ros: Wi-Fi is not connected; micro-ROS not started\n");
    return false;
  }

  rmw_ret_t transport_status = rmw_uros_set_custom_transport(
    true,
    &transport_context,
    picow_udp_transport_open,
    picow_udp_transport_close,
    picow_udp_transport_write,
    picow_udp_transport_read);
  if (transport_status != RMW_RET_OK) {
    wifi_log_printf("ros: could not configure the udp transport\n");
    return false;
  }

  bool agent_found = false;
  for (unsigned attempt = 1; attempt <= ROS_INIT_ATTEMPTS; ++attempt) {
    wifi_log_printf(
      "ros: pinging agent udp://%s:%u (attempt %u/%u)\n",
      ROS_AGENT_IP,
      (unsigned) ROS_AGENT_PORT,
      attempt,
      (unsigned) ROS_INIT_ATTEMPTS);

    if (rmw_uros_ping_agent(ROS_AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
      agent_found = true;
      break;
    }

    if (attempt < ROS_INIT_ATTEMPTS) {
      sleep_ms(ROS_INIT_RETRY_DELAY_MS);
    }
  }

  if (!agent_found) {
    wifi_log_printf(
      "ros: no agent at udp://%s:%u; micro-ROS disabled\n",
      ROS_AGENT_IP,
      (unsigned) ROS_AGENT_PORT);
    return false;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t status = rclc_support_init(&support, 0, NULL, &allocator);
  if (status != RCL_RET_OK) {
    wifi_log_printf("ros: rcl init failed (%d)\n", (int) status);
    return false;
  }

  node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_options = rcl_node_get_default_options();
  node_options.enable_rosout = false;

  status = rclc_node_init_with_options(
    &node,
    ROS_NODE_NAME,
    "",
    &support,
    &node_options);
  if (status != RCL_RET_OK) {
    wifi_log_printf("ros: node init failed (%d)\n", (int) status);
    rclc_support_fini(&support);
    return false;
  }

  node_ready = true;

  if (rmw_uros_sync_session(ROS_TIME_SYNC_TIMEOUT_MS) != RMW_RET_OK) {
    wifi_log_printf("ros: time sync failed; message stamps use boot time\n");
  }

  wifi_log_printf("ros: node \"%s\" connected to the agent\n", ROS_NODE_NAME);
  return true;
}
