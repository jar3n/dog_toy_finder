// micro-ROS publisher implementation.
#include "ir_obstacle_ros.h"

#include <stdbool.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw/qos_profiles.h>
#include <std_msgs/msg/bool.h>

#include "config/ir_obstacle_config.h"
#include "config/pinout.h"
#include "ir_obstacle.h"
#include "ros/ros_node.h"
#include "wifi/wifi_log.h"

static rcl_publisher_t publisher;
static bool publisher_ready = false;
static bool hardware_ready = false;
static bool publish_error_reported = false;

static bool init_publisher(void) {
    if (publisher_ready) {
        return true;
    }
    if (!ros_node_ready()) {
        return false;
    }

    publisher = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t options = rcl_publisher_get_default_options();
    options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    options.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    options.qos.depth = 5;

    rcl_ret_t status = rcl_publisher_init(
        &publisher,
        ros_node_handle(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        IR_OBSTACLE_TOPIC,
        &options);

    if (status != RCL_RET_OK) {
        wifi_log_printf("ir_obstacle: ROS publisher init failed (%d)\n",
                        (int) status);
        return false;
    }

    publisher_ready = true;
    wifi_log_printf("ir_obstacle: ROS publisher ready on %s\n",
                    IR_OBSTACLE_TOPIC);
    return true;
}

bool ir_obstacle_ros_init(void) {
    hardware_ready = ir_obstacle_init();
    if (!hardware_ready) {
        wifi_log_printf("ir_obstacle: sensor init failed\n");
        return false;
    }
    wifi_log_printf("ir_obstacle: sensor ready on GPIO%d\n",
                    IR_OBSTACLE_SIGNAL_PIN);
    return init_publisher();
}

bool ir_obstacle_ros_ready(void) {
    if (!hardware_ready) {
        return false;
    }
    if (!publisher_ready) {
        return init_publisher();
    }
    return true;
}

void ir_obstacle_ros_publish(void) {
    if (!ir_obstacle_ros_ready()) {
        return;
    }

    ir_obstacle_update();

    std_msgs__msg__Bool msg;
    msg.data = ir_obstacle_detected();

    rcl_ret_t status = rcl_publish(&publisher, &msg, NULL);
    if (status != RCL_RET_OK && !publish_error_reported) {
        publish_error_reported = true;
        wifi_log_printf("ir_obstacle: publish failed (%d)\n", (int) status);
    }
}