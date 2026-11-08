#include "motor/cmd_vel.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include <geometry_msgs/msg/twist.h>
#include <rcl/allocator.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw/qos_profiles.h>

#include "pico/time.h"

#include "config/cmd_vel_config.h"
#include "ros/ros_node.h"
#include "wifi/wifi_log.h"

#if CMD_VEL_QUEUE_DEPTH < 1
#error "CMD_VEL_QUEUE_DEPTH must be at least 1"
#endif

#if CMD_VEL_TIMEOUT_MS < 0
#error "CMD_VEL_TIMEOUT_MS must be zero or positive"
#endif

#if CMD_VEL_UPDATE_PERIOD_MS < 1
#error "CMD_VEL_UPDATE_PERIOD_MS must be at least 1"
#endif

static tb6612fng_t *motor_driver = NULL;
static rcl_subscription_t subscription;
static rclc_executor_t executor;
static geometry_msgs__msg__Twist incoming_command;

static bool subscriber_ready = false;
static bool command_received = false;
static bool motors_active = false;
static bool first_command_reported = false;
static bool invalid_command_reported = false;
static bool drive_error_reported = false;
static bool spin_error_reported = false;

static uint64_t last_command_us = 0;
static float target_linear_m_s = 0.0f;
static float target_angular_rad_s = 0.0f;

/**
 * Clamp a signed duty request to the range accepted by the motor driver.
 */
static float clamp_unit(float value) {
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

/**
 * Copy a validated Twist into the latest-command state.
 *
 * Runs inside the executor callback. It only records the command and its
 * arrival time; motor outputs are changed later from cmd_vel_process().
 */
static void twist_callback(const void *message) {
  const geometry_msgs__msg__Twist *twist =
    (const geometry_msgs__msg__Twist *) message;
  if (twist == NULL) {
    return;
  }

  if (!isfinite(twist->linear.x) || !isfinite(twist->angular.z)) {
    if (!invalid_command_reported) {
      invalid_command_reported = true;
      wifi_log_printf("cmd_vel: ignoring non-finite Twist command\n");
    }
    return;
  }

  target_linear_m_s = (float) twist->linear.x;
  target_angular_rad_s = (float) twist->angular.z;
  last_command_us = time_us_64();
  command_received = true;

  if (!first_command_reported) {
    first_command_reported = true;
    wifi_log_printf(
      "cmd_vel: first command received (linear=%.3f m/s angular=%.3f rad/s)\n",
      (double) target_linear_m_s,
      (double) target_angular_rad_s);
  }
}

/**
 * Convert the latest linear and angular command into left/right motor duty.
 *
 * Uses the standard differential-drive mix:
 *
 *   v_left  = linear.x - angular.z * wheel_separation / 2
 *   v_right = linear.x + angular.z * wheel_separation / 2
 *
 * If either wheel exceeds CMD_VEL_MAX_WHEEL_SPEED_M_S, both wheel speeds are
 * scaled by the same factor so the requested turn radius is preserved.
 */
static void apply_command(float linear_m_s, float angular_rad_s) {
  float half_separation = CMD_VEL_WHEEL_SEPARATION_M * 0.5f;
  float left_m_s = linear_m_s - angular_rad_s * half_separation;
  float right_m_s = linear_m_s + angular_rad_s * half_separation;

  float peak_m_s = fmaxf(fabsf(left_m_s), fabsf(right_m_s));
  if (peak_m_s > CMD_VEL_MAX_WHEEL_SPEED_M_S) {
    float scale = CMD_VEL_MAX_WHEEL_SPEED_M_S / peak_m_s;
    left_m_s *= scale;
    right_m_s *= scale;
  }

  float left_duty = clamp_unit(left_m_s / CMD_VEL_MAX_WHEEL_SPEED_M_S);
  float right_duty = clamp_unit(right_m_s / CMD_VEL_MAX_WHEEL_SPEED_M_S);

  bool left_ok = tb6612fng_set_speed(
    motor_driver,
    TB6612FNG_MOTOR_LEFT,
    left_duty);
  bool right_ok = tb6612fng_set_speed(
    motor_driver,
    TB6612FNG_MOTOR_RIGHT,
    right_duty);

  if (!left_ok || !right_ok) {
    if (!drive_error_reported) {
      drive_error_reported = true;
      wifi_log_printf("cmd_vel: motor driver rejected a speed command\n");
    }
  }

  motors_active =
    fabsf(left_duty) > 0.001f || fabsf(right_duty) > 0.001f;
}

/**
 * Stop both motors after a command timeout and clear the latched command.
 */
static void stop_after_timeout(void) {
  if (motors_active) {
    tb6612fng_stop_all(motor_driver, CMD_VEL_STOP_BRAKE != 0);
    motors_active = false;
    wifi_log_printf(
      "cmd_vel: no command for %u ms; motors %s\n",
      (unsigned) CMD_VEL_TIMEOUT_MS,
      CMD_VEL_STOP_BRAKE ? "braked" : "coasting");
  }

  command_received = false;
}

/**
 * Create the /cmd_vel subscription and its single-handle executor.
 */
static bool init_subscription(void) {
  subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t options =
    rcl_subscription_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  options.qos.depth = CMD_VEL_QUEUE_DEPTH;

  rcl_ret_t status = rcl_subscription_init(
    &subscription,
    ros_node_handle(),
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    CMD_VEL_TOPIC,
    &options);
  if (status != RCL_RET_OK) {
    wifi_log_printf(
      "cmd_vel: subscription init failed (%d)\n",
      (int) status);
    return false;
  }

  rcl_allocator_t allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();
  status = rclc_executor_init(
    &executor,
    ros_node_handle()->context,
    1,
    &allocator);
  if (status != RCL_RET_OK) {
    wifi_log_printf("cmd_vel: executor init failed (%d)\n", (int) status);
    rcl_ret_t fini_status =
      rcl_subscription_fini(&subscription, ros_node_handle());
    if (fini_status != RCL_RET_OK) {
      wifi_log_printf(
        "cmd_vel: subscription cleanup failed (%d)\n",
        (int) fini_status);
    }
    return false;
  }

  status = rclc_executor_add_subscription(
    &executor,
    &subscription,
    &incoming_command,
    twist_callback,
    ON_NEW_DATA);
  if (status != RCL_RET_OK) {
    wifi_log_printf(
      "cmd_vel: could not add subscription to executor (%d)\n",
      (int) status);
    rcl_ret_t executor_status = rclc_executor_fini(&executor);
    if (executor_status != RCL_RET_OK) {
      wifi_log_printf(
        "cmd_vel: executor cleanup failed (%d)\n",
        (int) executor_status);
    }
    rcl_ret_t fini_status =
      rcl_subscription_fini(&subscription, ros_node_handle());
    if (fini_status != RCL_RET_OK) {
      wifi_log_printf(
        "cmd_vel: subscription cleanup failed (%d)\n",
        (int) fini_status);
    }
    return false;
  }

  return true;
}

/**
 * Initialize the command subscriber and executor.
 */
bool cmd_vel_init(tb6612fng_t *driver) {
  if (driver == NULL || !driver->initialized) {
    return false;
  }

  motor_driver = driver;

  if (CMD_VEL_WHEEL_SEPARATION_M <= 0.0f ||
      CMD_VEL_MAX_WHEEL_SPEED_M_S <= 0.0f) {
    wifi_log_printf(
      "cmd_vel: wheel separation and max wheel speed must be positive\n");
    return false;
  }

  if (subscriber_ready) {
    return true;
  }

  if (!ros_node_init()) {
    return false;
  }

  if (!init_subscription()) {
    return false;
  }

  subscriber_ready = true;
  wifi_log_printf(
    "cmd_vel: listening on %s (wheel separation=%.3f m, max=%.3f m/s, "
    "timeout=%u ms)\n",
    CMD_VEL_TOPIC,
    (double) CMD_VEL_WHEEL_SEPARATION_M,
    (double) CMD_VEL_MAX_WHEEL_SPEED_M_S,
    (unsigned) CMD_VEL_TIMEOUT_MS);
  return true;
}

/**
 * Return whether the subscription was created.
 */
bool cmd_vel_ready(void) {
  return subscriber_ready;
}

/**
 * Service the subscription and apply the current command or timeout stop.
 */
void cmd_vel_process(void) {
  if (!subscriber_ready || motor_driver == NULL) {
    return;
  }

  // A zero timeout still polls the transport once, so this does not block.
  rcl_ret_t status = rclc_executor_spin_some(&executor, 0);
  if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    if (!spin_error_reported) {
      spin_error_reported = true;
      wifi_log_printf(
        "cmd_vel: executor spin failed (%d)\n",
        (int) status);
    }
  }

  if (!command_received) {
    return;
  }

  if (CMD_VEL_TIMEOUT_MS > 0) {
    uint64_t timeout_us = (uint64_t) CMD_VEL_TIMEOUT_MS * 1000u;
    if (time_us_64() - last_command_us > timeout_us) {
      stop_after_timeout();
      return;
    }
  }

  apply_command(target_linear_m_s, target_angular_rad_s);
}
