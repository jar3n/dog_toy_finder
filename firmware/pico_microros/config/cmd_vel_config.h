#pragma once

// Copy this file to config/cmd_vel_config.h and adjust to the robot.
// cmd_vel_config.h is ignored by git; the build creates it from this file
// automatically when it is missing or empty.
//
// These settings describe how ROS 2 Twist commands map to the two wheel
// motors. Electrical motor settings live in config/motor_config.h.

// Topic carrying geometry_msgs/msg/Twist commands.
#define CMD_VEL_TOPIC "/cmd_vel"

// Subscription queue depth. One is enough for teleop because only the newest
// command matters, but a larger depth tolerates short scheduling delays.
#define CMD_VEL_QUEUE_DEPTH 1

// Distance between the centres of the two wheel contact patches, in metres.
// Module 8 course robot: 0.133 m (5 in centre to centre). Calibrated value
// from the course data is 0.1291 m; refine on your own chassis.
#define CMD_VEL_WHEEL_SEPARATION_M 0.133f

// Wheel linear speed produced by a full-duty command, in metres per second.
// This converts requested m/s into PWM duty. It is a tuning value, not a
// hardware limit; the motor driver still clamps duty to MOTOR_MAX_DUTY.
// Module 8 course robot (TT 1:90 motors): about 0.45 m/s with no load.
#define CMD_VEL_MAX_WHEEL_SPEED_M_S 0.45f

// Stop the motors when no /cmd_vel message has arrived for this long.
// Set to 0 to disable the watchdog. Disabling it is not recommended.
#define CMD_VEL_TIMEOUT_MS 500

// How often the subscriber is serviced and motor duty is refreshed. 20 ms
// gives a 50 Hz control rate, matching the IMU and encoder publish rate.
#define CMD_VEL_UPDATE_PERIOD_MS 20

// Timeout stop behaviour: 1 = active short brake, 0 = coast.
#define CMD_VEL_STOP_BRAKE 1
