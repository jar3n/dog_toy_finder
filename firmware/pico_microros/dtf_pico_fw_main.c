/**

Dog Toy Finder Pico 2 W firmware Main file

*/


#include <pico/time.h>
#include <stdio.h>
#include "pico/stdio.h"

#include "imu/bno055.h"
#include "imu/imu_ros.h"
#include "ros/ros_log.h"
#include "wifi/wifi.h"

#define IMU_PUBLISH_PERIOD_MS 20
#define IMU_LOG_PERIOD_MS 500


/**
 * Firmware entry point.
 *
 * Starts USB serial, connects Wi-Fi, brings up micro-ROS logging, and
 * initializes the BNO055. The main loop then publishes Imu messages at 50 Hz
 * and logs Euler angles plus calibration status at 2 Hz.
 */
int main(void){
    stdio_init_all();
    sleep_ms(1500);
    wifi_init();
    ros_log_init();

    bool imu_ready = bno055_init();
    if (imu_ready) {
        ros_log_printf("imu: BNO055 ready in NDOF mode\n");
    } else {
        ros_log_printf(
            "imu: BNO055 not detected; check the I2C wiring in config/pinout.h\n");
    }

    bool imu_publishing = imu_ready && imu_ros_init();
    if (imu_ready && !imu_publishing) {
        ros_log_printf("imu: ROS publishing unavailable\n");
    }

    absolute_time_t next_publish = get_absolute_time();
    absolute_time_t next_log = get_absolute_time();

    while (true) {
        absolute_time_t now = get_absolute_time();

        if (imu_publishing &&
            absolute_time_diff_us(now, next_publish) <= 0) {
            imu_ros_publish();
            next_publish = delayed_by_ms(now, IMU_PUBLISH_PERIOD_MS);
        }

        if (imu_ready && absolute_time_diff_us(now, next_log) <= 0) {
            bno055_euler_t euler;
            bno055_calibration_t calibration;

            if (bno055_read_euler(&euler) &&
                bno055_read_calibration(&calibration)) {
                ros_log_printf(
                    "imu: heading=%.1f roll=%.1f pitch=%.1f cal=%u%u%u%u\n",
                    euler.heading,
                    euler.roll,
                    euler.pitch,
                    (unsigned) calibration.system,
                    (unsigned) calibration.gyro,
                    (unsigned) calibration.accel,
                    (unsigned) calibration.mag);
            }

            next_log = delayed_by_ms(now, IMU_LOG_PERIOD_MS);
        }

        sleep_ms(1);
    }
    
}
