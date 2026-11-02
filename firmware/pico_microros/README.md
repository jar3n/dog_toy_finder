# Pico micro-ROS firmware

- `src/`, `include/` - main application
- `transport/` - WiFi UDP transport for micro-ROS
- `drivers/` - motors, encoders, IMU, arm servos, other sensors
- `wifi/` - Wi-Fi station setup and serial/UDP log mirror
- `ros/` - micro-ROS session and ROS 2 log publisher
- `imu/` - BNO055 9-DoF IMU driver over I2C and `sensor_msgs/msg/Imu` publisher
- `config/` - pin maps and tunables (WiFi secrets stay out of git)
- `microros/` - micro-ROS colcon config and library builder
- `lib/` - generated libmicroros (ignored)
- `tests/`, `tools/`, `build/`
