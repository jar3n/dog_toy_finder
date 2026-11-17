# Docker

| Folder / service | Container | Notes |
|---|---|---|
| `ros2/` | ROS 2 Humble + vision/mapping packages + ROSA Python deps | host networking; the main development and run container |
| `microros_builder/` | ROS 2 Humble + Pico SDK + `arm-none-eabi` toolchain + `micro_ros_setup` | builds `libmicroros` for the Pico |
| `micro_ros_agent` | `microros/micro-ros-agent:humble` | host networking, UDP port 8888; connects the Pico to the ROS 2 graph |

`docker-compose.yml` defines all three (run from this folder). The repository
root is mounted at `/workspace` in the build containers, so the same tree is
visible on the host and inside them.

Build the main image once:

```bash
cd docker
docker compose build ros2
```

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

`scripts/launch_all.sh` starts and stops this container automatically when you
run it from the host.

## micro-ROS library builder

Only rebuild this image when the micro-ROS transport configuration changes
(for example `microros_builder/microros/colcon.meta` or `toolchain.cmake`):

```bash
docker compose build microros_builder
docker compose run --rm microros_builder
```

The builder writes the static library used by the Pico firmware. See
`microros_builder/readme.md` for details.
