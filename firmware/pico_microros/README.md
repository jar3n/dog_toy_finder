# Pico micro-ROS firmware

- `dtf_pico_fw_main.c` - firmware entry point and main publish/log/control loop
- `udp_transport/` - custom UDP transport for micro-ROS over Wi-Fi
- `wifi/` - Wi-Fi station setup and serial/UDP log mirror
- `ros/` - micro-ROS session and ROS 2 log publisher
- `imu/` - BNO055 9-DoF IMU driver over I2C and `sensor_msgs/msg/Imu` publisher
- `encoder/` - hall quadrature decoders, wheel odometry, and `JointState` publisher
- `motor/` - TB6612FNG dual motor controller and `/cmd_vel` subscriber
- `ir_obstacle/` - Keyestudio KS0051 infrared obstacle avoidance sensor and `/ir_obstacle/detected` publisher
- `microros_time/` - agent time synchronization helper
- `config/` - committed pin maps and tunables; only the Wi-Fi credentials file stays local
- `microros/` - micro-ROS colcon config and library builder
- `lib/` - generated libmicroros (ignored)
- `build/` - CMake/Ninja output and staged artifacts (ignored)
