#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "rcl/node.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the micro-ROS session over the Wi-Fi UDP transport.
 *
 * Probes the configured agent a few times, then creates the rcl support
 * structures and a single node. Blocks for at most a few seconds at boot;
 * returns false when the agent is unreachable so the rest of the firmware can
 * keep running.
 *
 * @return true when the node is connected to the agent, false otherwise.
 */
bool ros_node_init(void);

/** @return true when the node is connected to the agent. */
bool ros_node_ready(void);

/**
 * @return the active node, or NULL when ros_node_init() has not succeeded.
 */
rcl_node_t *ros_node_handle(void);

/**
 * Current time in nanoseconds.
 *
 * Uses the agent-synchronized epoch when rmw_uros_sync_session() succeeded and
 * falls back to time since boot otherwise. Useful for message stamps.
 */
int64_t ros_node_time_ns(void);

#ifdef __cplusplus
}
#endif
