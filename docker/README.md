# Docker

| Folder / service | Container | Notes |
|---|---|---|
| `ros2/` | ROS 2 Humble + vision/mapping packages + ROSA Python deps + Pillow | host networking; the main development and run container |
| `microros_builder/` | ROS 2 Humble + Pico SDK + `arm-none-eabi` toolchain + `micro_ros_setup` | builds `libmicroros` for the Pico |
| `micro_ros_agent` | `microros/micro-ros-agent:humble` | host networking, UDP port 8888; connects the Pico to the ROS 2 graph |

`docker-compose.yml` defines all three (run from this folder). The repository
root is mounted at `/workspace` in the build containers, so the same tree is
visible on the host and inside them.

The `ros2` container starts D-Bus/Avahi as root, then runs your command as
`HOST_UID:HOST_GID` (default `1000:1000`). Set those variables in the shell or
a local `.env` if your account uses different IDs. Captures, `colcon build`
output, and test files written to `/workspace` are then owned by you rather
than root. `microros_builder` still needs root for `rosdep`, but it chowns its
generated artifacts back to the same IDs when it finishes.

Build the main image once:

```bash
cd docker
docker compose build ros2
```

The `ros2` image installs Pillow so vision nodes can convert ROS images to JPEG.
Open-vocabulary inference for the `open_vocab` pipeline runs on the computer via
`./scripts/start_yolo_service.sh` (host venv with CPU torch, ultralytics,
transformers, and weights in `software/host_vision/models/`). One detector runs
at a time, selected with `DETECTOR_MODEL` (`yolo_world`, `yoloe_26`, or
`grounding_dino`). The container only talks to it over HTTP at
`YOLO_SERVICE_URL` (default `http://localhost:5006`). The `stereo-net` pipeline also
uses `./scripts/start_stereo_service.sh` for dense-stereo depth on the host
(`STEREO_SERVICE_URL`, default `http://localhost:5007`; one model via
`DEPTH_MODEL`). Vision does not need an Ollama server; Ollama is used separately
for ROSA planning.

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
