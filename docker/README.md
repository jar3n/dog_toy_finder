# Docker

| Folder | Container | Notes |
|---|---|---|
| `ros2/` | ROS 2 Humble + ROSA + Ollama/LangChain/MCP Python deps | host networking |
| `gazebo/` | ROS 2 Humble + Gazebo Fortress + noVNC browser desktop + ROSA | open http://localhost:6080/vnc.html |
| `microros_builder/` | ROS 2 Humble + Pico SDK + arm-none-eabi toolchain + micro_ros_setup | builds `libmicroros` for the Pico |

`docker-compose.yml` defines all three (run from this folder). The repo root is mounted at `/workspace`.

Do not run both the ros2 and the gazebo docker containers. ros2 is for running with the real robot and gazebo is for running a simulation.

## micro-ROS agent (from the old notes)

Run this to enable the microROS firmware on the pico to talk with the ROS 2 graph on the computer.

```
docker run --rm -it \
  --name pico-micro-ros-agent \
  -p 8888:8888/udp \
  microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6
```
