#pragma once

#include <stdbool.h>

#include "motor/tb6612fng.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Subscribe to geometry_msgs/msg/Twist on the configured /cmd_vel topic.
 *
 * The driver must already be initialized with tb6612fng_init(). On success
 * the module creates a best-effort subscription and an rclc executor. The
 * motors remain coasting until the first command arrives.
 *
 * @param driver initialized TB6612FNG driver to command.
 * @return true when the subscription and executor are ready.
 */
bool cmd_vel_init(tb6612fng_t *driver);

/**
 * @return true when the /cmd_vel subscription was created successfully.
 */
bool cmd_vel_ready(void);

/**
 * Service the subscription, apply the newest command, and enforce the timeout.
 *
 * Call this periodically at CMD_VEL_UPDATE_PERIOD_MS. It polls the subscription
 * without blocking, converts linear.x and angular.z into left/right wheel duty,
 * and stops both motors when commands stop arriving.
 */
void cmd_vel_process(void);

#ifdef __cplusplus
}
#endif
