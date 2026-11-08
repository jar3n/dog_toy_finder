#pragma once

// Copy this file to config/motor_config.h and adjust to the robot.
// motor_config.h is ignored by git; the build creates it from this file
// automatically when it is missing or empty.
//
// Pin wiring lives in config/pinout.h. This file holds electrical and
// direction settings for the TB6612FNG driver.

// PWM frequency for both channels. 20 kHz is above the audible range and is
// well below the TB6612FNG limit. Lower it (for example 1000) only if a motor
// whines or the driver runs warm. The RP2350's PWM divider limits the minimum
// to roughly 9 Hz with the default 150 MHz system clock.
#define MOTOR_PWM_FREQUENCY_HZ 20000

// Maximum duty cycle applied to a full-speed command. Keep below 1.0 to limit
// torque or top speed. The driver clamps requested speeds to this value.
#define MOTOR_MAX_DUTY 1.0f

// Set to 1 when a side spins backwards for a positive command. This depends on
// how the motor leads are connected and how the gearbox is mounted. Swapping
// the two motor leads is equivalent.
#define MOTOR_LEFT_INVERT 0
#define MOTOR_RIGHT_INVERT 0
