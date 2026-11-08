/**

Dog Toy Finder Pico 2 W firmware Main file

*/


#include <pico/time.h>
#include <stdio.h>
#include "pico/stdio.h"

#include "config/cmd_vel_config.h"
#include "config/encoder_config.h"
#include "config/ir_obstacle_config.h"
#include "encoder/encoder_ros.h"
#include "imu/bno055.h"
#include "imu/imu_ros.h"
#include "ir_obstacle/ir_obstacle.h"
#include "ir_obstacle/ir_obstacle_ros.h"
#include "motor/cmd_vel.h"
#include "motor/tb6612fng.h"
#include "ros/ros_log.h"
#include "wifi/wifi.h"

#define IMU_PUBLISH_PERIOD_MS 20
#define IMU_LOG_PERIOD_MS 500


/**
 * Firmware entry point.
 *
 * Starts USB serial, connects Wi-Fi, brings up micro-ROS logging, and
 * initializes the BNO055, both wheel encoders, and the TB6612FNG motor
 * controller. It then subscribes to /cmd_vel and drives the motors from Twist
 * messages, stopping them when commands time out. The main loop publishes Imu
 * and JointState messages at 50 Hz, services /cmd_vel at 50 Hz, and logs IMU
 * and encoder status at 2 Hz and 1 Hz respectively.
 */
int main(void){
    stdio_init_all();
    sleep_ms(1500);
    wifi_init();
    ros_log_init();

    bool imu_ready = bno055_init();
    if (imu_ready) {
        ros_log_printf("imu: BNO055 ready in IMU mode\n");
    } else {
        ros_log_printf(
            "imu: BNO055 not detected; check the I2C wiring in config/pinout.h\n");
    }

    bool imu_publishing = imu_ready && imu_ros_init();
    if (imu_ready && !imu_publishing) {
        ros_log_printf("imu: ROS publishing unavailable\n");
    }

    bool encoder_ready = encoder_ros_init();
    bool encoder_publishing = encoder_ready && encoder_ros_ready();

    tb6612fng_t motor_driver;
    bool motor_ready = tb6612fng_init(&motor_driver);
    if (motor_ready) {
        ros_log_printf(
            "motor: TB6612FNG ready; both channels coasting\n");
    } else {
        ros_log_printf(
            "motor: TB6612FNG init failed; check the wiring in "
            "config/pinout.h\n");
    }

    bool cmd_vel_subscribed = motor_ready && cmd_vel_init(&motor_driver);
    if (motor_ready && !cmd_vel_subscribed) {
        ros_log_printf(
            "cmd_vel: subscriber unavailable; motors stay coasting\n");
    }

    bool ir_obstacle_ready = ir_obstacle_ros_init();
    if (ir_obstacle_ready) {
        ros_log_printf("ir_obstacle: sensor ready\n");
    } else {
        ros_log_printf(
            "ir_obstacle: unavailable; check the wiring in config/pinout.h\n");
    }

    absolute_time_t next_publish = get_absolute_time();
    absolute_time_t next_log = get_absolute_time();
    absolute_time_t next_encoder_publish = get_absolute_time();
    absolute_time_t next_encoder_log = get_absolute_time();
    absolute_time_t next_cmd_vel = get_absolute_time();
    absolute_time_t next_ir_obstacle_publish = get_absolute_time();

    while (true) {
        absolute_time_t now = get_absolute_time();

        if (cmd_vel_subscribed &&
            absolute_time_diff_us(now, next_cmd_vel) <= 0) {
            cmd_vel_process();
            next_cmd_vel = delayed_by_ms(now, CMD_VEL_UPDATE_PERIOD_MS);
        }

        if (imu_publishing &&
            absolute_time_diff_us(now, next_publish) <= 0) {
            imu_ros_publish();
            next_publish = delayed_by_ms(now, IMU_PUBLISH_PERIOD_MS);
        }

        if (ir_obstacle_ready &&
            absolute_time_diff_us(now, next_ir_obstacle_publish) <= 0) {
            ir_obstacle_ros_publish();
            next_ir_obstacle_publish =
                delayed_by_ms(now, IR_OBSTACLE_PUBLISH_PERIOD_MS);
        }

        if (imu_ready && absolute_time_diff_us(now, next_log) <= 0) {
            bno055_euler_t euler;
            bno055_calibration_t calibration;

            if (bno055_read_euler(&euler) &&
                bno055_read_calibration(&calibration)) {
                ros_log_printf(
                    "imu: heading=%.1f roll=%.1f pitch=%.1f cal=%u%u%u\n",
                    euler.heading,
                    euler.roll,
                    euler.pitch,
                    (unsigned) calibration.system,
                    (unsigned) calibration.gyro,
                    (unsigned) calibration.accel);
            }

            next_log = delayed_by_ms(now, IMU_LOG_PERIOD_MS);
        }

        if (encoder_publishing &&
            absolute_time_diff_us(now, next_encoder_publish) <= 0) {
            encoder_ros_publish();
            next_encoder_publish =
                delayed_by_ms(now, ENCODER_PUBLISH_PERIOD_MS);
        }

        if (encoder_ready &&
            absolute_time_diff_us(now, next_encoder_log) <= 0) {
            // Sample here only when publish() is not already doing it.
            if (!encoder_publishing) {
                encoder_ros_update();
            }

            ros_log_printf(
                "enc: counts L=%ld R=%ld | L=%.2f rad/s (%.3f m/s) "
                "R=%.2f rad/s (%.3f m/s)\n",
                (long) encoder_ros_left_count(),
                (long) encoder_ros_right_count(),
                encoder_ros_left_velocity_rad_s(),
                encoder_ros_left_linear_velocity_m_s(),
                encoder_ros_right_velocity_rad_s(),
                encoder_ros_right_linear_velocity_m_s());

            next_encoder_log = delayed_by_ms(now, ENCODER_LOG_PERIOD_MS);
        }

        sleep_ms(1);
    }
    
}
