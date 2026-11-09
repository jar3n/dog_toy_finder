# Install and bring-up guide

End-to-end setup for the Dog Toy Finder: toolchains, Docker image, firmware
builds, flashing, and first run. For the full test/evaluation flow see
[full_system_test.md](full_system_test.md).

## 1. Host prerequisites

- Git
- Docker + Docker Compose
- PlatformIO (`pio`) for the ESP32-CAM (optional but recommended)
- Pico SDK toolchain for the Pico firmware (handled by the build script)

Clone the repo:

```bash
git clone git@github.com:jar3n/dog_toy_finder.git
cd dog_toy_finder
```

## 2. Build the ROS 2 Docker image

```bash
cd docker
docker compose build
```

This builds `docker-ros2:latest` with ROS 2 Humble, the vision/mapping
dependencies, and the ROSA Python packages.

## 3. Build the micro-ROS library (Pico)

Only needed if you change `docker/microros_builder/microros/colcon.meta`
(e.g. to raise `RMW_UXRCE_MAX_PUBLISHERS`):

```bash
cd docker
docker compose build microros_builder
docker compose run --rm microros_builder bash /workspace/docker/microros_builder/microros/build-library.sh
# copy docker/microros_builder/microros/artifacts/* into firmware/pico_microros/lib/libmicroros/
```

The repo already ships a prebuilt `lib/libmicroros/libmicroros.a`, so this step
is optional for a first build.

## 4. Build the Pico firmware

```bash
cd firmware/pico_microros
# Create the (git-ignored) config files from their templates:
../../scripts/setup_firmware_config.sh
# edit the created files: SSID, password, agent IP/port, pin/geometry
cd ../..
firmware/scripts/pico_build/build.sh
```

The UF2 is staged at
`firmware/pico_microros/build/artifacts/dtf_pico_fw.uf2`.

## 5. Flash the Pico

1. Hold BOOTSEL on the Pico 2 W and plug it in (or reset into bootloader).
2. Copy `dtf_pico_fw.uf2` to the mounted `RPI-RP2` drive.
3. The Pico reboots and connects to Wi-Fi + the micro-ROS agent.

## 6. Build and flash the ESP32-CAM (stereo eyes)

```bash
cd firmware/esp_cam
cp wifi_config.example.h wifi_config.h   # fill in 2.4 GHz SSID/password
pio run -e esp32cam_left  -t upload
pio run -e esp32cam_right -t upload
```

Note the printed IPs / mDNS names (`esp32cam-left.local`, `esp32cam-right.local`).

## 7. Start the ROS 2 stack

On the host:

```bash
cd docker
docker compose up -d micro_ros_agent
docker compose run --rm ros2
```

Inside the container:

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# one command for everything:
/workspace/scripts/launch_all.sh
# stop everything:
/workspace/scripts/stop_all.sh
```

Or run components individually (see [full_system_test.md](full_system_test.md)).

## 8. First-run checks

```bash
ros2 topic hz /joint_states
ros2 topic hz /imu/data
ros2 topic hz /stereo/disparity
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
```

## 9. Common issues

- **No `/joint_states`**: check the micro-ROS agent is running on UDP 8888 and
  the Pico's `ros_config.h` points at the computer's IP.
- **EKF warnings about QoS**: the firmware publishes best-effort; the
  computer side uses `SensorDataQoS` (best effort). Do not change one side to
  reliable without the other.
- **Encoder counts backwards**: flip `ENCODER_LEFT_INVERT` / `ENCODER_RIGHT_INVERT`
  in `encoder_config.h` (or the matching firmware invert flag).
- **Motor spins the wrong way**: set `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`.
- **IR sensor conflicts with an encoder**: move one device to free GPIOs
  (0-3, 10-13, 26-28) and update `config/pinout.h`.

## 10. Geometry defaults

Module 8 course robot values (refine on your chassis):

| Parameter | Default |
|---|---|
| Wheel diameter | 0.065 m (radius 0.0325) |
| Wheel separation | 0.133 m |
| Counts per revolution | 4320 |
| `/cmd_vel` timeout | 500 ms |