#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize both wheel encoders and the joint-state publisher.
 *
 * The GPIO decoders and the position/velocity filters are always initialized,
 * even when the micro-ROS agent is unreachable, so the main loop can still log
 * encoder data over serial/UDP. The ROS publisher is created on the shared
 * micro-ROS node when the agent is available.
 *
 * @return true when both encoders were configured, false on a GPIO setup error.
 */
bool encoder_ros_init(void);

/**
 * @return true when the sensor_msgs/msg/JointState publisher exists.
 */
bool encoder_ros_ready(void);

/**
 * Sample both encoders once using the current time.
 *
 * Call this when encoder data is needed but the ROS publisher is unavailable,
 * or let encoder_ros_publish() do it.
 */
void encoder_ros_update(void);

/**
 * Sample both encoders and publish one sensor_msgs/msg/JointState message.
 *
 * The message contains the left and right joint names in a fixed order, with
 * output-shaft position in radians and filtered velocity in radians per
 * second. Effort is left empty.
 *
 * @return true when the message was published, false when ROS is unavailable
 *         or rcl_publish() failed.
 */
bool encoder_ros_publish(void);

/** @return latest left output-shaft position in radians. */
double encoder_ros_left_position_rad(void);

/** @return latest right output-shaft position in radians. */
double encoder_ros_right_position_rad(void);

/** @return latest left output-shaft velocity in radians per second. */
double encoder_ros_left_velocity_rad_s(void);

/** @return latest right output-shaft velocity in radians per second. */
double encoder_ros_right_velocity_rad_s(void);

/** @return latest left wheel linear velocity in metres per second. */
double encoder_ros_left_linear_velocity_m_s(void);

/** @return latest right wheel linear velocity in metres per second. */
double encoder_ros_right_linear_velocity_m_s(void);

/** @return signed 4x quadrature count for the left encoder. */
int32_t encoder_ros_left_count(void);

/** @return signed 4x quadrature count for the right encoder. */
int32_t encoder_ros_right_count(void);

#ifdef __cplusplus
}
#endif
