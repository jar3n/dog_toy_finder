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

Settings live in `config/ros_config.h`, which is committed. Edit it in place:

```c
#define ROS_AGENT_IP "192.168.1.100"
#define ROS_AGENT_PORT 8888

#define ROS_NODE_NAME "dtf_pico"
#define ROS_CLIENT_KEY 0x44544650u
#define ROS_LOG_TOPIC "/pico/log"
#define ROS_LOG_QUEUE_DEPTH 10
#define ROS_LOG_MAX_MESSAGE_LENGTH 512

#define ROS_INIT_RETRY_DELAY_MS 1000
#define ROS_AGENT_PING_TIMEOUT_MS 500
#define ROS_TIME_SYNC_TIMEOUT_MS 1000
```

- `ROS_AGENT_IP` must be the address of the computer running the agent, reachable
  from the Pico over Wi-Fi.
- `ROS_CLIENT_KEY` identifies the micro-ROS session. Keep it stable across Pico
  reboots and firmware rebuilds: the agent then resets the existing session and
  replaces its entities instead of leaving another `/dtf_pico` in the graph.
  Changing the key requires one agent restart to discard the old session.
- `ROS_LOG_TOPIC` defaults to a dedicated topic. Set it to `/rosout` if you want
  `rqt_console` to show the messages without extra configuration.
- `ROS_INIT_RETRY_DELAY_MS` is the pause between agent probes.
  `ROS_AGENT_PING_TIMEOUT_MS` is the time allowed for one probe's reply.
- The firmware waits for the agent indefinitely: there is no attempt limit, no
  auto-reset, and the LED keeps blinking until the agent answers. Start or
  restart the agent at any time; the Pico joins on the next probe. Only a
  power cycle reboots the Pico.
- `ROS_INIT_ATTEMPTS` is retained in the config for compatibility but is no
  longer used.

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
works for most setups. The startup log prints one
`ros: attempting to connect to the micro-ROS bridge ...` line and one
`ros: node "dtf_pico" connected to the agent` line; it does not repeat either
message while retrying.

## Connection troubleshooting

**The agent logs 23-byte requests but no node appears.** The client is probably
sending stream-framed messages. The agent's `udp4` transport does not implement
framing, so the reply never reaches the client's framing reader. Confirm that
`ros_node_init()` calls `rmw_uros_set_custom_transport(false, ...)` and that the
flashed binary contains the zero argument; `../udp_transport/README.md` has the
exact symptom and the `objdump` check.

**Pings reach the agent but it never creates a session.** Check
`ROS_AGENT_IP` from the Pico's network, confirm the host firewall allows UDP
8888, and watch the agent:

```bash
docker compose logs -f micro_ros_agent
ros2 node list        # /dtf_pico after the session forms
```

**A Pico reboot adds another `/dtf_pico`.** The agent keeps proxy clients after
a client disconnects. With a random client key, every reboot looks like a new
client and leaves another node, `/cmd_vel` subscription, and `/imu/data`
publisher in the graph. The firmware now sends `ROS_CLIENT_KEY`; confirm the
boot log prints `ros: using stable client key 0x44544650` and the agent log
says `session re-established` after a reboot (not a fresh `create` line).
Sessions created by older firmware are not removed automatically. After
flashing this firmware, restart the agent once to clear them:

```bash
(cd docker && docker compose restart micro_ros_agent)
# wait for the Pico to reconnect, then verify exactly one of each:
firmware/scripts/check_pico_graph.sh
```

**No serial console is available.** The firmware mirrors every log line to
UDP 5005 as well:

```bash
python3 firmware/scripts/wifi_log_listen.py
```

**The LED never stops blinking.** The boot sequence is still waiting for the
agent or Wi-Fi. Solid means every required device and publisher came up; off
means at least one boot check failed (the log says which).

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
- The firmware brings Wi-Fi up before starting the micro-ROS node;
  `ros_node_init()` refuses to start without a Wi-Fi link. Boot order checks the
  sensors before the Wi-Fi and micro-ROS connections are attempted.
- Log lines emitted before `ros_log_init()` succeeds (for example the Wi-Fi
  connection messages) are serial/UDP only.
- The micro-ROS library is built with `RCL_LOGGING_ENABLED=OFF`, so ROS 2's
  built-in `rosout` machinery is unavailable; this module publishes its own
  `Log` messages instead.
- `ros_node_time_ns()` provides the synchronized timestamp used by both the log
  publisher and the IMU publisher (`../imu/imu_ros.c`).
- The node is shared: `RMW_UXRCE_MAX_NODES=1` and
  `RMW_UXRCE_MAX_PUBLISHERS=4` cover the log, IMU, joint-state, and IR
  publishers. `/cmd_vel` uses one of the `RMW_UXRCE_MAX_SUBSCRIPTIONS=2` slots.
- The Pico radio is 2.4 GHz only.
