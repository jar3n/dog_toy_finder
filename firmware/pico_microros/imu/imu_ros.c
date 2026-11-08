#include "imu/imu_ros.h"

#include <stdint.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/imu.h>

#include "config/imu_config.h"
#include "imu/bno055.h"
#include "ros/ros_node.h"
#include "wifi/wifi_log.h"

#ifndef IMU_ROS_TOPIC
#define IMU_ROS_TOPIC "/imu/data"
#endif

#ifndef IMU_ROS_FRAME_ID
#define IMU_ROS_FRAME_ID "imu_link"
#endif

#ifndef IMU_ROS_QUEUE_DEPTH
#define IMU_ROS_QUEUE_DEPTH 10
#endif

#ifndef IMU_ROS_ORIENTATION_VARIANCE
#define IMU_ROS_ORIENTATION_VARIANCE 0.01
#endif

#ifndef IMU_ROS_ANGULAR_VELOCITY_VARIANCE
#define IMU_ROS_ANGULAR_VELOCITY_VARIANCE 0.01
#endif

#ifndef IMU_ROS_LINEAR_ACCELERATION_VARIANCE
#define IMU_ROS_LINEAR_ACCELERATION_VARIANCE 0.04
#endif

#define DEGREES_TO_RADIANS (0.017453292519943295f)
#define NANOSECONDS_PER_SECOND 1000000000LL

static rcl_publisher_t publisher;
static bool publisher_ready = false;
static bool publish_error_reported = false;

static char frame_id[] = IMU_ROS_FRAME_ID;

/**
 * Return whether the IMU publisher has been created.
 */
bool imu_ros_ready(void) {
  return publisher_ready;
}

/**
 * Attach the sensor_msgs/msg/Imu publisher to the shared micro-ROS node.
 *
 * Ensures the node exists, creates a best-effort publisher using the configured
 * topic and queue depth, and logs the result through the serial/UDP sink.
 *
 * @return true when the publisher was created.
 */
bool imu_ros_init(void) {
  if (publisher_ready) {
    return true;
  }

  if (!ros_node_init()) {
    return false;
  }

  publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t options = rcl_publisher_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  options.qos.depth = IMU_ROS_QUEUE_DEPTH;

  rcl_ret_t status = rcl_publisher_init(
    &publisher,
    ros_node_handle(),
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    IMU_ROS_TOPIC,
    &options);
  if (status != RCL_RET_OK) {
    wifi_log_printf("imu: ROS publisher init failed (%d)\n", (int) status);
    return false;
  }

  publisher_ready = true;
  wifi_log_printf(
    "imu: publishing sensor_msgs/msg/Imu on %s (frame \"%s\")\n",
    IMU_ROS_TOPIC,
    IMU_ROS_FRAME_ID);
  return true;
}

/**
 * Fill a 3x3 covariance matrix with zeros and one variance on its diagonal.
 */
static void set_covariance(double covariance[9], double variance) {
  memset(covariance, 0, 9 * sizeof covariance[0]);
  covariance[0] = variance;
  covariance[4] = variance;
  covariance[8] = variance;
}

/**
 * Read the BNO055 and publish one sensor_msgs/msg/Imu message.
 *
 * Reads the fused quaternion, angular velocity, and linear acceleration,
 * converts the gyro from deg/s to rad/s, stamps the message, fills the
 * covariance diagonals, and publishes it. Returns false when a sensor read or
 * the publish fails.
 */
bool imu_ros_publish(void) {
  if (!publisher_ready) {
    return false;
  }

  bno055_quaternion_t orientation;
  bno055_vector_t angular_velocity;
  bno055_vector_t linear_acceleration;

  if (!bno055_read_quaternion(&orientation) ||
      !bno055_read_gyroscope(&angular_velocity) ||
      !bno055_read_linear_acceleration(&linear_acceleration)) {
    return false;
  }

  sensor_msgs__msg__Imu message;
  memset(&message, 0, sizeof message);

  int64_t stamp_ns = ros_node_time_ns();
  message.header.stamp.sec =
    (int32_t) (stamp_ns / NANOSECONDS_PER_SECOND);
  message.header.stamp.nanosec =
    (uint32_t) (stamp_ns % NANOSECONDS_PER_SECOND);

  message.header.frame_id.data = frame_id;
  message.header.frame_id.size = strlen(frame_id);
  message.header.frame_id.capacity = sizeof frame_id;

  message.orientation.w = orientation.w;
  message.orientation.x = orientation.x;
  message.orientation.y = orientation.y;
  message.orientation.z = orientation.z;
  set_covariance(
    message.orientation_covariance,
    IMU_ROS_ORIENTATION_VARIANCE);

  message.angular_velocity.x = angular_velocity.x * DEGREES_TO_RADIANS;
  message.angular_velocity.y = angular_velocity.y * DEGREES_TO_RADIANS;
  message.angular_velocity.z = angular_velocity.z * DEGREES_TO_RADIANS;
  set_covariance(
    message.angular_velocity_covariance,
    IMU_ROS_ANGULAR_VELOCITY_VARIANCE);

  message.linear_acceleration.x = linear_acceleration.x;
  message.linear_acceleration.y = linear_acceleration.y;
  message.linear_acceleration.z = linear_acceleration.z;
  set_covariance(
    message.linear_acceleration_covariance,
    IMU_ROS_LINEAR_ACCELERATION_VARIANCE);

  rcl_ret_t status = rcl_publish(&publisher, &message, NULL);
  if (status != RCL_RET_OK) {
    if (!publish_error_reported) {
      publish_error_reported = true;
      wifi_log_printf(
        "imu: failed to publish an Imu message (%d)\n",
        (int) status);
    }
    return false;
  }

  return true;
}
