#pragma once

// Copy this file to config/encoder_config.h and adjust to the robot.
// encoder_config.h is ignored by git; the build creates it from this file
// automatically when it is missing or empty.
//
// Wiring (GPIO numbers) lives in config/pinout.h. This file holds the encoder
// geometry, direction, filtering, and ROS settings.

// ---------------------------------------------------------------------------
// TT encoded gearmotor geometry
// ---------------------------------------------------------------------------
//
// The reference motor (nulllaborg/tt-encoder-motor) has a 12 PPR hall disk
// and a 1:90 gearbox. With 4x quadrature decoding that is
//   12 * 90 * 4 = 4320 counts per revolution of the output shaft.
// Measure a wheel, then rotate it one full turn by hand and confirm that the
// reported position changes by 2*pi before trusting odometry.
#define ENCODER_PPR 12
#define ENCODER_QUADRATURE_MULTIPLIER 4
#define ENCODER_GEAR_RATIO 90.0

// Radius of the drive wheel in metres. 0.0325 is a 65 mm TT wheel, matching
// the Module 8 course robot (wheel diameter 0.065 m; calibrated 0.0629 m).
// Replace it with the measured value for your chassis.
#define ENCODER_WHEEL_RADIUS_M 0.0325

// Set to 1 when a side counts backwards for the robot's forward direction.
// Swapping the A/B wires is equivalent.
#define ENCODER_LEFT_INVERT 0
#define ENCODER_RIGHT_INVERT 0

// ---------------------------------------------------------------------------
// Velocity estimate
// ---------------------------------------------------------------------------
//
// First-order low-pass weight for each new velocity sample: 0.3 trusts the new
// measurement 30% and the previous estimate 70%. Use 1.0 to disable smoothing.
#define ENCODER_VELOCITY_FILTER_ALPHA 0.3f

// ---------------------------------------------------------------------------
// ROS publishing
// ---------------------------------------------------------------------------
//
// Both wheels are published together as one sensor_msgs/msg/JointState.
// Joint names must match the robot_description/URDF used by the computer.
#define ENCODER_ROS_TOPIC "/joint_states"
#define ENCODER_ROS_FRAME_ID ""
#define ENCODER_ROS_QUEUE_DEPTH 10
#define ENCODER_LEFT_JOINT_NAME "left_wheel_joint"
#define ENCODER_RIGHT_JOINT_NAME "right_wheel_joint"

// Publish and log periods in milliseconds. 20 ms = 50 Hz.
#define ENCODER_PUBLISH_PERIOD_MS 20
#define ENCODER_LOG_PERIOD_MS 1000
