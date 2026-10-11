# Docker

| Folder | Container | Notes |
|---|---|---|
| `ros2/` | ROS 2 Humble + ROSA + Ollama/LangChain/MCP Python deps | host networking |
| `gazebo/` | ROS 2 Humble + Gazebo Fortress + noVNC browser desktop + ROSA | open http://localhost:6080/vnc.html |
| `microros_builder/` | ROS 2 Humble + Pico SDK + arm-none-eabi toolchain + micro_ros_setup | builds `libmicroros` for the Pico |

`docker-compose.yml` defines all three (run from this folder). The repo root is mounted at `/workspace`.

The micro-ROS builder can also be opened as a VS Code dev container via
`.devcontainer/microros-builder/devcontainer.json` at the repo root.

## micro-ROS agent (from the old notes)

```
docker run --rm -it \
  --name pico-micro-ros-agent \
  -p 8888:8888/udp \
  microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6
```
