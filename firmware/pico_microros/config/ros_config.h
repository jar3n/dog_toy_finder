#pragma once

// micro-ROS agent address and node settings. Edit this file in place.

// IPv4 address of the computer running the micro-ROS agent. The Pico must be
// able to reach it over Wi-Fi.
#define ROS_AGENT_IP "192.168.4.32"
#define ROS_AGENT_PORT 8888

// micro-ROS node name that shows up in the ROS 2 graph.
#define ROS_NODE_NAME "dtf_pico"

// Stable Micro XRCE-DDS client key. The agent keeps a proxy client (and its
// DDS entities) after the Pico reboots. Reusing the same key makes the agent
// reset that session and replace its entities instead of creating a second
// session with a second /dtf_pico node. Do not change this value while old
// sessions may still be present: restart the micro-ROS agent once when changing
// it so stale proxy clients are discarded.
#define ROS_CLIENT_KEY 0x44544650u  // "DTFP"

// Logs are published as rcl_interfaces/msg/Log on this topic.
// Point it at /rosout if you want rqt_console to pick the messages up.
#define ROS_LOG_TOPIC "/pico/log"
#define ROS_LOG_QUEUE_DEPTH 10
#define ROS_LOG_MAX_MESSAGE_LENGTH 512

// The firmware now probes the agent indefinitely (no attempt limit and no
// automatic reset); ROS_INIT_ATTEMPTS is retained for compatibility only.
// ROS_INIT_RETRY_DELAY_MS is the delay between probes.
#define ROS_INIT_ATTEMPTS 0
#define ROS_INIT_RETRY_DELAY_MS 1000
#define ROS_AGENT_PING_TIMEOUT_MS 500

// Time sync with the agent. When it fails, log stamps fall back to time since
// boot. Set to 0 to skip the sync attempt.
#define ROS_TIME_SYNC_TIMEOUT_MS 1000
