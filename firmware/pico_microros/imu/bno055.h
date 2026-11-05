#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * BNO055 operation modes (register 0x3D).
 *
 * IMU mode fuses the accelerometer and gyroscope into a relative orientation
 * without using the magnetometer, and is the mode bno055_init() selects.
 */
typedef enum {
  BNO055_MODE_CONFIG = 0x00,
  BNO055_MODE_ACCEL_ONLY = 0x01,
  BNO055_MODE_MAG_ONLY = 0x02,
  BNO055_MODE_GYRO_ONLY = 0x03,
  BNO055_MODE_ACCEL_MAG = 0x04,
  BNO055_MODE_ACCEL_GYRO = 0x05,
  BNO055_MODE_MAG_GYRO = 0x06,
  BNO055_MODE_AMG = 0x07,
  BNO055_MODE_IMU = 0x08,
  BNO055_MODE_COMPASS = 0x09,
  BNO055_MODE_M4G = 0x0A,
  BNO055_MODE_NDOF_FMC_OFF = 0x0B,
  BNO055_MODE_NDOF = 0x0C,
} bno055_operation_mode_t;

/** Orientation in degrees (heading 0..360, roll/pitch -180..180). */
typedef struct {
  float heading;
  float roll;
  float pitch;
} bno055_euler_t;

/** Unit quaternion (w, x, y, z). */
typedef struct {
  float w;
  float x;
  float y;
  float z;
} bno055_quaternion_t;

/** Three-axis sample. Units depend on the read function. */
typedef struct {
  float x;
  float y;
  float z;
} bno055_vector_t;

/** Per-sensor calibration level, each 0 (uncalibrated) to 3 (fully). */
typedef struct {
  uint8_t system;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
} bno055_calibration_t;

/**
 * Initialize the I2C bus, reset the sensor, and put it in IMU mode.
 *
 * IMU mode fuses the accelerometer and gyroscope only. Heading is relative to
 * the orientation at power-on and slowly drifts, but it is not disturbed by
 * motor currents the way magnetometer-based modes are.
 *
 * Blocks for roughly a second while the sensor boots. Safe to call more than
 * once; each call performs a fresh reset.
 *
 * @return true when the chip ID matched and IMU mode was entered.
 */
bool bno055_init(void);

/**
 * Reset the sensor through SYS_TRIGGER and wait for it to boot.
 *
 * Leaves the sensor in CONFIG mode; call bno055_init() for the full setup.
 *
 * @return true when the sensor answered with the expected chip ID.
 */
bool bno055_reset(void);

/**
 * Probe the bus and check the chip ID without changing any configuration.
 *
 * @return true when a BNO055 answers at BNO055_I2C_ADDRESS.
 */
bool bno055_is_connected(void);

/**
 * Select an operation mode and wait for the switch to settle.
 *
 * Most reads below require a fusion mode (NDOF, IMU, COMPASS, ...). Reading
 * Euler angles or quaternions in CONFIG mode returns stale/zero data.
 *
 * @return true when the mode register was written.
 */
bool bno055_set_operation_mode(bno055_operation_mode_t mode);

/** Read fused Euler angles in degrees. */
bool bno055_read_euler(bno055_euler_t *euler);

/** Read the fused orientation quaternion. */
bool bno055_read_quaternion(bno055_quaternion_t *quaternion);

/** Read acceleration in m/s^2 (includes gravity). */
bool bno055_read_acceleration(bno055_vector_t *acceleration);

/** Read linear acceleration in m/s^2 (gravity removed). */
bool bno055_read_linear_acceleration(bno055_vector_t *acceleration);

/** Read the gravity vector in m/s^2. */
bool bno055_read_gravity(bno055_vector_t *gravity);

/** Read angular velocity in degrees/s. */
bool bno055_read_gyroscope(bno055_vector_t *angular_velocity);

/** Read the magnetic field in microtesla. */
bool bno055_read_magnetometer(bno055_vector_t *magnetic_field);

/** Read the die temperature in degrees Celsius. */
bool bno055_read_temperature(float *celsius);

/** Read the per-sensor calibration levels. */
bool bno055_read_calibration(bno055_calibration_t *calibration);

/**
 * Poll until the sensors used by IMU mode report full calibration.
 *
 * The robot has to be moved through a few orientations for the gyroscope and
 * accelerometer to calibrate. The magnetometer level is ignored because IMU
 * mode does not use it. Calibration values are not persisted across power
 * cycles by the sensor.
 *
 * @param timeout_ms maximum time to wait; 0 performs a single check.
 * @return true when system, gyro, and accel all reached level 3.
 */
bool bno055_wait_for_calibration(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
