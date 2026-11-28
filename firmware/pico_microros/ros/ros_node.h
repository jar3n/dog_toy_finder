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
 * Waits for the agent, creates the rcl support structures and a single node,
 * then retries agent time sync until the epoch clock is valid. There is no
 * attempt limit: the LED keeps blinking while either the agent or a plausible
 * host-epoch clock is pending.
 *
 * @return true when the node is connected and time-synchronized, false when
 * setup fails before an agent session can be created.
 */
bool ros_node_init(void);

/** @return true when the node is connected to the agent. */
bool ros_node_ready(void);

/**
 * @return the active node, or NULL when ros_node_init() has not succeeded.
 */
rcl_node_t *ros_node_handle(void);

/**
 * Current host-epoch time in nanoseconds for message headers.
 *
 * Returns zero when the session is not synchronized with a plausible host
 * epoch; boot-time values are never used as message stamps.
 */
int64_t ros_node_time_ns(void);

#ifdef __cplusplus
}
#endif
