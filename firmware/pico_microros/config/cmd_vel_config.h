#pragma once

// These settings describe how ROS 2 Twist commands map to the two wheel
// motors. Edit this file in place. Electrical motor settings live in
// config/motor_config.h.

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

// Reflex stop: 1 = brake locally when the IR obstacle sensor sees something in
// front, 0 = ignore the sensor. This runs on the Pico, so it still protects the
// robot if Wi-Fi or the ROS 2 computer is slow or disconnected.
#define CMD_VEL_IR_REFLEX 1

// While the IR reflex is active, allow reverse motion only in short steps so
// the robot can back away and turn around. Set to 0 to block every command
// until the path is clear (no reverse at all).
#define CMD_VEL_IR_REFLEX_REVERSE_STEPS 1

// Maximum reverse speed during a step. The normal reverse command may request
// more, but the reflex caps it to this value.
#define CMD_VEL_IR_REFLEX_REVERSE_SPEED_M_S 0.10f

// One reverse step lasts this long, then the reflex brakes and waits for the
// pause before allowing another step. With the defaults above one step moves
// about 4 cm at 0.10 m/s.
#define CMD_VEL_IR_REFLEX_REVERSE_STEP_MS 400

// Pause between reverse steps. This prevents a continuous, unrestricted
// reverse even if the ROS side keeps publishing a negative linear.x.
#define CMD_VEL_IR_REFLEX_REVERSE_PAUSE_MS 300
