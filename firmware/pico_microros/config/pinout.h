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
//   SDA        GPIO4  (physical pin 6)
//   SCL        GPIO5  (physical pin 7)
//   ADDR       GND    -> I2C address 0x28
//
// I2C0 is used on its default pin pair, so the Pico SDK can route it directly.
// GPIO4/5 are free on the Pico 2 W (the wireless chip uses other pins).
//
// To use different pins, pick another valid I2C0 pair (for example GPIO0/1 or
// GPIO16/17) or switch BNO055_I2C to i2c1 and use that instance's pins. Both
// lines need pull-ups; most BNO055 breakouts already have them.
#define BNO055_I2C i2c0
#define BNO055_I2C_SDA_PIN 4
#define BNO055_I2C_SCL_PIN 5

// BNO055 supports 100 kHz and 400 kHz. 400 kHz keeps the IMU readout fast.
#define BNO055_I2C_FREQUENCY_HZ 400000

// 0x28 with ADDR tied low (breakout default), 0x29 with ADDR tied high.
#define BNO055_I2C_ADDRESS 0x28
