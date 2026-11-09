// micro-ROS publisher for the IR obstacle avoidance sensor.
#pragma once

#include <stdbool.h>

// Initialize the publisher. Returns true when it was created.
bool ir_obstacle_ros_init(void);

// Return whether the publisher is ready.
bool ir_obstacle_ros_ready(void);

// Publish the current obstacle state. Call periodically.
void ir_obstacle_ros_publish(void);
