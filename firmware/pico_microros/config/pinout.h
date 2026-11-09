#pragma once

// Board pin assignments for the Dog Toy Finder Pico 2 W.
//
// Keep every physical wiring choice in this file so the driver code stays
// board-independent. If a peripheral moves, change it here only.

#include "hardware/i2c.h"

// ---------------------------------------------------------------------------
// BNO055 9-DoF IMU (I2C)
// ---------------------------------------------------------------------------
//
// Wiring on the Pico 2 W:
//
//   BNO055     Pico 2 W
//   ------     --------
//   VIN        3V3 (3.3 V only; the BNO055 is not 5 V tolerant)
//   GND        GND
//   SDA        GPIO14 (physical pin 19)
//   SCL        GPIO15 (physical pin 20)
//   ADDR       GND    -> I2C address 0x28
//
// I2C1 is used on GPIO14/15, one of its valid pin pairs.
//
// To use different pins, pick another valid I2C1 pair (for example GPIO2/3 or
// GPIO26/27) or switch BNO055_I2C to i2c0 and use that instance's pins. Both
// lines need pull-ups; most BNO055 breakouts already have them.
#define BNO055_I2C i2c1
#define BNO055_I2C_SDA_PIN 14
#define BNO055_I2C_SCL_PIN 15

// BNO055 supports 100 kHz and 400 kHz. 400 kHz keeps the IMU readout fast.
#define BNO055_I2C_FREQUENCY_HZ 400000

// 0x28 with ADDR tied low (breakout default), 0x29 with ADDR tied high.
#define BNO055_I2C_ADDRESS 0x28

// ---------------------------------------------------------------------------
// Wheel encoders (hall-effect quadrature, TT encoded gearmotor)
// ---------------------------------------------------------------------------
//
// Each motor has a 6-pin PH2.0 lead carrying motor +/-, encoder VCC/GND, and
// the two hall channels A/B. Power the encoder from 3V3 (the sensor is rated
// 3.3-5 V) and connect its ground to the Pico ground. The hall outputs are
// open-drain, so the driver enables the Pico's internal pull-ups on every
// channel; no external resistors are required.
//
// Wiring on the Pico 2 W:
//
//   Left encoder A    GPIO7   (physical pin 10)
//   Left encoder B    GPIO6   (physical pin 9)
//   Right encoder A   GPIO9   (physical pin 12)
//   Right encoder B   GPIO8   (physical pin 11)
//
// A/B are swapped relative to the original assignment; this is equivalent to
// swapping the two hall wires or setting the matching *_INVERT flag. These
// four pins support edge interrupts. Encoder geometry and joint names live in
// config/encoder_config.h.
#define ENCODER_LEFT_PIN_A 7
#define ENCODER_LEFT_PIN_B 6
#define ENCODER_RIGHT_PIN_A 9
#define ENCODER_RIGHT_PIN_B 8

// ---------------------------------------------------------------------------
// TB6612FNG dual motor controller
// ---------------------------------------------------------------------------
//
// Wiring on the Pico 2 W:
//
//   Pico 2 W       TB6612FNG       Notes
//   --------       ---------       -----
//   3V3            VCC             logic supply (2.7-5.5 V)
//   GND            GND             common ground with the motor battery
//   battery +      VM              motor supply (3-15 V), not the Pico 3V3
//   GPIO16         PWMA            left motor PWM  (PWM slice 0, channel A)
//   GPIO18         AIN1            left motor direction 1
//   GPIO17         AIN2            left motor direction 2
//   GPIO22         PWMB            right motor PWM (PWM slice 3, channel A)
//   GPIO20         BIN1            right motor direction 1
//   GPIO21         BIN2            right motor direction 2
//   GPIO19         STBY            high = bridge enabled
//   AO1/AO2        left motor      motor winding, never the Pico
//   BO1/BO2        right motor     motor winding, never the Pico
//
// Add a 0.1 uF decoupling capacitor across VCC/GND and a bulk capacitor
// (10-100 uF) across VM/GND near the driver. The TB6612 includes flyback
// diodes, but short motor leads and a common ground still matter.
//
// The two PWM pins intentionally use different slices so each channel can be
// configured independently.
#define MOTOR_PWMA_PIN 16
#define MOTOR_AIN1_PIN 18
#define MOTOR_AIN2_PIN 17
#define MOTOR_PWMB_PIN 22
#define MOTOR_BIN1_PIN 20
#define MOTOR_BIN2_PIN 21
#define MOTOR_STBY_PIN 19

// ---------------------------------------------------------------------------
// Keyestudio KS0051 infrared obstacle avoidance sensor
// ---------------------------------------------------------------------------
//
// The sensor has a digital (TTL) output that goes active when an obstacle is
// detected 2-40 cm in front of it (distance set by the on-board trimmer), plus
// an enable (EN) line. Working voltage 3.3-5 V; power the sensor from 3V3.
//
// Wiring on the Pico 2 W:
//
//   Sensor     Pico 2 W
//   ------     --------
//   VCC        3V3
//   GND        GND
//   S          GPIO4   (signal, active high by default)
//   EN         GPIO5   (high enables the sensor)
//
#define IR_OBSTACLE_SIGNAL_PIN 4
#define IR_OBSTACLE_ENABLE_PIN 5
