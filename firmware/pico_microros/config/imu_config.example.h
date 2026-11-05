#pragma once

// Copy this file to config/imu_config.h and adjust if needed.
// imu_config.h is ignored by git.
//
// The build creates imu_config.h from this file automatically when it is
// missing or empty.

// Topic and frame for sensor_msgs/msg/Imu messages.
#define IMU_ROS_TOPIC "/imu/data"
#define IMU_ROS_FRAME_ID "imu_link"
#define IMU_ROS_QUEUE_DEPTH 10

// Covariance diagonal entries. The sensor_msgs/Imu convention treats an all
// zero covariance as "unknown", so fill in rough variances instead.
// Units are rad^2, (rad/s)^2, and (m/s^2)^2 respectively.
#define IMU_ROS_ORIENTATION_VARIANCE 0.01
#define IMU_ROS_ANGULAR_VELOCITY_VARIANCE 0.01
#define IMU_ROS_LINEAR_ACCELERATION_VARIANCE 0.04
