# Docker

| Folder | Container | Notes |
|---|---|---|
| `ros2/` | ROS 2 Humble + ROSA + Ollama/LangChain/MCP Python deps | host networking |
| `gazebo/` | ROS 2 Humble + Gazebo Fortress + noVNC browser desktop + ROSA | open http://localhost:6080/vnc.html |
| `microros_builder/` | ROS 2 Humble + Pico SDK + arm-none-eabi toolchain + micro_ros_setup | builds `libmicroros` for the Pico |
| `micro_ros_agent` | `microros/micro-ros-agent:humble` | host networking, UDP port 8888; connects the Pico to the ROS 2 graph |

`docker-compose.yml` defines all four (run from this folder). The repo root is mounted at `/workspace`.

Do not run both the ros2 and the gazebo docker containers. ros2 is for running with the real robot and gazebo is for running a simulation.

## micro-ROS agent

Run this to enable the micro-ROS firmware on the Pico to talk with the ROS 2
graph on the computer. It is a separate container because the `ros2` image does
not include the `micro_ros_agent` package.

```bash
docker compose up -d micro_ros_agent
docker compose logs -f micro_ros_agent     # watch connections
docker compose stop micro_ros_agent
```

The compose service uses host networking, so the agent listens on UDP 8888 on
the computer directly. The equivalent standalone command is:

```bash
docker run --rm -it \
  --name pico-micro-ros-agent \
  --network host \
  microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6
```
