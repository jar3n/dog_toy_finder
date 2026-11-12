#pragma once

// micro-ROS agent address and node settings. Edit this file in place.

// IPv4 address of the computer running the micro-ROS agent. The Pico must be
// able to reach it over Wi-Fi.
#define ROS_AGENT_IP "192.168.4.31"
#define ROS_AGENT_PORT 8888

// micro-ROS node name that shows up in the ROS 2 graph.
#define ROS_NODE_NAME "dtf_pico"

// Logs are published as rcl_interfaces/msg/Log on this topic.
// Point it at /rosout if you want rqt_console to pick the messages up.
#define ROS_LOG_TOPIC "/pico/log"
#define ROS_LOG_QUEUE_DEPTH 10
#define ROS_LOG_MAX_MESSAGE_LENGTH 512

// How many times to probe for the agent before giving up, and how long to wait
// between probes. Each probe blocks for at most ROS_AGENT_PING_TIMEOUT_MS.
#define ROS_INIT_ATTEMPTS 3
#define ROS_INIT_RETRY_DELAY_MS 1000
#define ROS_AGENT_PING_TIMEOUT_MS 100

// Time sync with the agent. When it fails, log stamps fall back to time since
// boot. Set to 0 to skip the sync attempt.
#define ROS_TIME_SYNC_TIMEOUT_MS 1000
