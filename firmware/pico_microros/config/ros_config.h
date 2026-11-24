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

// The firmware probes the agent indefinitely (no attempt limit and no
// automatic reset). ROS_INIT_RETRY_DELAY_MS is the delay between probes.
#define ROS_INIT_RETRY_DELAY_MS 1000
#define ROS_AGENT_PING_TIMEOUT_MS 500

// Time sync with the agent is required before the node is considered ready.
// Wi-Fi stalls can outlast a single NTP exchange, so the firmware retries
// indefinitely instead of publishing boot-time stamps. ROS_TIME_SYNC_TIMEOUT_MS
// is the budget for each sync attempt; ROS_TIME_SYNC_RETRY_DELAY_MS is the pause
// between attempts. Both must be positive.
#define ROS_TIME_SYNC_TIMEOUT_MS 5000
#define ROS_TIME_SYNC_RETRY_DELAY_MS 1000

// Ignore any synchronized epoch at or below this value. Boot time on the Pico
// is only a few seconds (~1e12 ns); current host epoch time is ~1.8e18 ns.
#define ROS_MIN_VALID_EPOCH_NS 1000000000000000LL
