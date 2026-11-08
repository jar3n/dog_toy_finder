#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create the sensor_msgs/msg/Imu publisher on the shared micro-ROS node.
 *
 * The node and Wi-Fi UDP session are owned by ros/ros_node.c; this function
 * attaches an IMU publisher to it. Call ros_node_init() (or ros_log_init())
 * first, or let this function start the node when the agent is reachable.
 *
 * @return true when the publisher was created.
 */
bool imu_ros_init(void);

/** @return true when the IMU publisher exists. */
bool imu_ros_ready(void);

/**
 * Read the BNO055 and publish one sensor_msgs/msg/Imu message.
 *
 * Publishes orientation (BNO055 fused quaternion), angular velocity in rad/s,
 * and linear acceleration in m/s^2 with gravity removed. The sensor must be in
 * a fusion mode, which bno055_init() selects.
 *
 * @return true when every read succeeded and the message was published.
 */
bool imu_ros_publish(void);

#ifdef __cplusplus
}
#endif
