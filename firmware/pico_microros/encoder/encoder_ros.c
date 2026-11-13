#include "encoder/encoder_ros.h"

#include <stddef.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/joint_state.h>

#include "hardware/gpio.h"
#include "pico/time.h"

#include "config/encoder_config.h"
#include "config/pinout.h"
#include "encoder/encoder.h"
#include "ros/ros_node.h"
#include "wifi/wifi_log.h"

// Total 4x counts for one output-shaft revolution.
#define ENCODER_COUNTS_PER_REVOLUTION \
  ((double) ENCODER_PPR * \
   (double) ENCODER_QUADRATURE_MULTIPLIER * \
   (double) ENCODER_GEAR_RATIO)

static rcl_publisher_t publisher;
static bool publisher_ready = false;
static bool hardware_ready = false;
static bool publish_error_reported = false;

static encoder_t left_encoder;
static encoder_t right_encoder;

// Joint names are stored once; the message points at these buffers.
static char left_joint_name[] = ENCODER_LEFT_JOINT_NAME;
static char right_joint_name[] = ENCODER_RIGHT_JOINT_NAME;
static char empty_frame[] = ENCODER_ROS_FRAME_ID;

static rosidl_runtime_c__String joint_names[2];
static double joint_positions[2];
static double joint_velocities[2];

/**
 * Fill one fixed-capacity string view without allocating memory.
 */
static void set_string_view(
  rosidl_runtime_c__String *view,
  char *data,
  size_t capacity) {
  view->data = data;
  view->size = strlen(data);
  view->capacity = capacity;
}

/**
 * Convert an output-shaft angular velocity to wheel linear velocity.
 */
static double linear_velocity_m_s(const encoder_t *encoder) {
  return encoder_velocity_rad_s(encoder) * ENCODER_WHEEL_RADIUS_M;
}

/**
 * Attach the sensor_msgs/msg/JointState publisher to the shared node.
 *
 * Called only after both decoders are configured. Returns without creating a
 * publisher when the agent is unreachable; the encoder data keep working.
 */
static bool init_publisher(void) {
  if (publisher_ready) {
    return true;
  }

  if (!ros_node_init()) {
    return false;
  }

  publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t options = rcl_publisher_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  options.qos.depth = ENCODER_ROS_QUEUE_DEPTH;

  rcl_ret_t status = rcl_publisher_init(
    &publisher,
    ros_node_handle(),
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    ENCODER_ROS_TOPIC,
    &options);
  if (status != RCL_RET_OK) {
    wifi_log_printf(
      "encoder: ROS publisher init failed (%d)\n",
      (int) status);
    return false;
  }

  publisher_ready = true;
  wifi_log_printf(
    "encoder: publishing JointState on %s (%u counts/rev)\n",
    ENCODER_ROS_TOPIC,
    (unsigned) ENCODER_COUNTS_PER_REVOLUTION);
  return true;
}

/**
 * Configure both quadrature decoders and remember the joint-name views.
 */
static bool init_hardware(void) {
  if (hardware_ready) {
    return true;
  }

  // Presence/short check: the hall outputs are open-drain and pulled up, so
  // with a healthy (or absent) encoder both channels read high. A pin reading
  // low at boot means it is shorted to ground (or to a low-driven line), which
  // is never a valid encoder state.
  if (ENCODER_LEFT_PIN_A == ENCODER_LEFT_PIN_B ||
      ENCODER_RIGHT_PIN_A == ENCODER_RIGHT_PIN_B ||
      ENCODER_LEFT_PIN_A == ENCODER_RIGHT_PIN_A ||
      ENCODER_LEFT_PIN_A == ENCODER_RIGHT_PIN_B ||
      ENCODER_LEFT_PIN_B == ENCODER_RIGHT_PIN_A ||
      ENCODER_LEFT_PIN_B == ENCODER_RIGHT_PIN_B) {
    wifi_log_printf("encoder: encoder pins are not distinct\n");
    return false;
  }

  if (!encoder_init(
        &left_encoder,
        ENCODER_LEFT_PIN_A,
        ENCODER_LEFT_PIN_B,
        ENCODER_LEFT_INVERT != 0,
        ENCODER_COUNTS_PER_REVOLUTION,
        ENCODER_VELOCITY_FILTER_ALPHA)) {
    return false;
  }

  if (!encoder_init(
        &right_encoder,
        ENCODER_RIGHT_PIN_A,
        ENCODER_RIGHT_PIN_B,
        ENCODER_RIGHT_INVERT != 0,
        ENCODER_COUNTS_PER_REVOLUTION,
        ENCODER_VELOCITY_FILTER_ALPHA)) {
    return false;
  }

  // Verify both channels read high (pull-ups) rather than being shorted to
  // ground. Open-drain hall outputs at rest are high, so a low reading means
  // a short or a stuck-low line, not a healthy encoder.
  if (!gpio_get(ENCODER_LEFT_PIN_A) || !gpio_get(ENCODER_LEFT_PIN_B) ||
      !gpio_get(ENCODER_RIGHT_PIN_A) || !gpio_get(ENCODER_RIGHT_PIN_B)) {
    wifi_log_printf(
      "encoder: a channel pin reads low at boot (short to ground?)\n");
    return false;
  }

  set_string_view(
    &joint_names[0],
    left_joint_name,
    sizeof left_joint_name);
  set_string_view(
    &joint_names[1],
    right_joint_name,
    sizeof right_joint_name);

  hardware_ready = true;
  return true;
}

/**
 * Initialize both encoders and, when possible, their ROS publisher.
 */
bool encoder_ros_init(void) {
  if (!init_hardware()) {
    wifi_log_printf(
      "encoder: hardware check failed; check wiring in config/pinout.h\n");
    return false;
  }

  if (!init_publisher()) {
    wifi_log_printf(
      "encoder: ROS publishing unavailable; data still logged\n");
  }
  return true;
}

/**
 * Return whether the JointState publisher was created.
 */
bool encoder_ros_ready(void) {
  return publisher_ready;
}

/**
 * Sample both encoders with a single timestamp.
 */
void encoder_ros_update(void) {
  if (!hardware_ready) {
    return;
  }

  uint64_t now_us = time_us_64();
  encoder_update(&left_encoder, now_us);
  encoder_update(&right_encoder, now_us);
}

/**
 * Sample the encoders and publish their positions and velocities.
 */
bool encoder_ros_publish(void) {
  if (!hardware_ready || !publisher_ready) {
    return false;
  }

  encoder_ros_update();

  sensor_msgs__msg__JointState message;
  memset(&message, 0, sizeof message);

  int64_t stamp_ns = ros_node_time_ns();
  message.header.stamp.sec =
    (int32_t) (stamp_ns / 1000000000LL);
  message.header.stamp.nanosec =
    (uint32_t) (stamp_ns % 1000000000LL);
  set_string_view(
    &message.header.frame_id,
    empty_frame,
    sizeof empty_frame);

  message.name.data = joint_names;
  message.name.size = 2;
  message.name.capacity = 2;

  joint_positions[0] = encoder_position_rad(&left_encoder);
  joint_positions[1] = encoder_position_rad(&right_encoder);
  joint_velocities[0] = encoder_velocity_rad_s(&left_encoder);
  joint_velocities[1] = encoder_velocity_rad_s(&right_encoder);

  message.position.data = joint_positions;
  message.position.size = 2;
  message.position.capacity = 2;

  message.velocity.data = joint_velocities;
  message.velocity.size = 2;
  message.velocity.capacity = 2;

  // No torque sensor is present, so leave effort empty.
  message.effort.data = NULL;
  message.effort.size = 0;
  message.effort.capacity = 0;

  rcl_ret_t status = rcl_publish(&publisher, &message, NULL);
  if (status != RCL_RET_OK) {
    if (!publish_error_reported) {
      publish_error_reported = true;
      wifi_log_printf(
        "encoder: failed to publish JointState (%d)\n",
        (int) status);
    }
    return false;
  }

  return true;
}

/**
 * Return the latest left output-shaft position in radians.
 */
double encoder_ros_left_position_rad(void) {
  return encoder_position_rad(&left_encoder);
}

/**
 * Return the latest right output-shaft position in radians.
 */
double encoder_ros_right_position_rad(void) {
  return encoder_position_rad(&right_encoder);
}

/**
 * Return the latest left output-shaft velocity in radians per second.
 */
double encoder_ros_left_velocity_rad_s(void) {
  return encoder_velocity_rad_s(&left_encoder);
}

/**
 * Return the latest right output-shaft velocity in radians per second.
 */
double encoder_ros_right_velocity_rad_s(void) {
  return encoder_velocity_rad_s(&right_encoder);
}

/**
 * Return the left wheel linear velocity in metres per second.
 */
double encoder_ros_left_linear_velocity_m_s(void) {
  return linear_velocity_m_s(&left_encoder);
}

/**
 * Return the right wheel linear velocity in metres per second.
 */
double encoder_ros_right_linear_velocity_m_s(void) {
  return linear_velocity_m_s(&right_encoder);
}

/**
 * Return the left signed 4x quadrature count.
 */
int32_t encoder_ros_left_count(void) {
  return encoder_count(&left_encoder);
}

/**
 * Return the right signed 4x quadrature count.
 */
int32_t encoder_ros_right_count(void) {
  return encoder_count(&right_encoder);
}
