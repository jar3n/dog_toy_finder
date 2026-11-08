# ROS logging

`ros_node.c` / `ros_node.h` bring up a micro-ROS session over the Wi-Fi UDP
transport. `ros_log.c` / `ros_log.h` turn log lines into ROS 2 messages.

`main` only needs:

```c
#include "ros/ros_log.h"

wifi_init();
ros_log_init();
ros_log_printf("Hello from the Pico\n");
```

`ros_log_printf()` writes the message to USB serial, mirrors it to UDP (when
Wi-Fi logging is active), and publishes it to ROS 2 as an
`rcl_interfaces/msg/Log` message. Before the agent is reachable, the ROS part is
skipped and the serial/UDP output still works.

## Configuration

Settings live in `config/ros_config.h`, which is ignored by git. When that file
is missing or empty, CMake copies `config/ros_config.example.h` into place
during configure. Edit the generated file:

```c
#define ROS_AGENT_IP "192.168.1.100"
#define ROS_AGENT_PORT 8888

#define ROS_NODE_NAME "dtf_pico"
#define ROS_LOG_TOPIC "/pico/log"
#define ROS_LOG_QUEUE_DEPTH 10
#define ROS_LOG_MAX_MESSAGE_LENGTH 512

#define ROS_INIT_ATTEMPTS 3
#define ROS_INIT_RETRY_DELAY_MS 1000
#define ROS_AGENT_PING_TIMEOUT_MS 100
#define ROS_TIME_SYNC_TIMEOUT_MS 1000
```

- `ROS_AGENT_IP` must be the address of the computer running the agent, reachable
  from the Pico over Wi-Fi.
- `ROS_LOG_TOPIC` defaults to a dedicated topic. Set it to `/rosout` if you want
  `rqt_console` to show the messages without extra configuration.
- `ROS_INIT_ATTEMPTS` probes are made at boot. If the agent is not running yet,
  ROS publishing stays off for that boot; serial and UDP logging still work.

## Running the agent

On the computer, with the agent listening on UDP port 8888 (the compose file's
existing command):

```bash
docker run --rm -it \
  --name pico-micro-ros-agent \
  -p 8888:8888/udp \
  microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6
```

Or natively:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

The agent must bind to an address the Pico can reach; the default (`0.0.0.0`)
works for most setups.

## Receiving the logs

```bash
ros2 node list            # /dtf_pico
ros2 topic echo /pico/log
```

Example message:

```yaml
stamp:
  sec: 1762041234
  nanosec: 567000000
level: 20
name: dtf_pico
msg: Hello World from the PICO
file: ''
function: ''
```

`level` follows the usual severity values (`10` debug, `20` info, `30` warn,
`40` error, `50` fatal). Timestamps come from `rmw_uros_sync_session()`; when the
sync fails, stamps fall back to time since boot.

## Notes

- The publisher uses best-effort QoS: log delivery is not retried, so a message
  can be dropped under load or while the agent is disconnected.
- The firmware raises `PICO_STACK_SIZE` to 4 KiB (`CMakeLists.txt`) because
  micro-ROS and the log fan-out need more than the 2 KiB SDK default. That is
  the largest value that fits in the SCRATCH_Y region used for core 0.
- `wifi_init()` must run first; `ros_node_init()` refuses to start without a
  Wi-Fi link.
- Log lines emitted before `ros_log_init()` succeeds (for example the Wi-Fi
  connection messages) are serial/UDP only.
- The micro-ROS library is built with `RCL_LOGGING_ENABLED=OFF`, so ROS 2's
  built-in `rosout` machinery is unavailable; this module publishes its own
  `Log` messages instead.
- `ros_node_time_ns()` provides the synchronized timestamp used by both the log
  publisher and the IMU publisher (`../imu/imu_ros.c`).
- The node is shared: `RMW_UXRCE_MAX_NODES=1` and
  `RMW_UXRCE_MAX_PUBLISHERS=3` allow the log publisher plus the IMU publisher
  (and one spare).
- The Pico radio is 2.4 GHz only.
