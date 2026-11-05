#include "imu/bno055.h"

#include <stddef.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#include "config/pinout.h"

// Register map (datasheet section 4.3).
#define BNO055_REG_CHIP_ID 0x00
#define BNO055_REG_PAGE_ID 0x07
#define BNO055_REG_ACCEL_DATA 0x08
#define BNO055_REG_MAG_DATA 0x0E
#define BNO055_REG_GYRO_DATA 0x14
#define BNO055_REG_EULER_DATA 0x1A
#define BNO055_REG_QUATERNION_DATA 0x20
#define BNO055_REG_LINEAR_ACCEL_DATA 0x28
#define BNO055_REG_GRAVITY_DATA 0x2E
#define BNO055_REG_TEMPERATURE 0x34
#define BNO055_REG_CALIBRATION_STATUS 0x35
#define BNO055_REG_UNIT_SELECT 0x3B
#define BNO055_REG_OPERATION_MODE 0x3D
#define BNO055_REG_POWER_MODE 0x3E
#define BNO055_REG_SYSTEM_TRIGGER 0x3F

#define BNO055_CHIP_ID 0xA0

#define BNO055_POWER_MODE_NORMAL 0x00
#define BNO055_SYSTEM_TRIGGER_RESET 0x20

#define BNO055_UNIT_SELECT_METRIC_DEGREES 0x00

#define BNO055_RESET_DELAY_MS 650
#define BNO055_BOOT_TIMEOUT_MS 1000
#define BNO055_MODE_SWITCH_DELAY_MS 30
#define BNO055_CALIBRATION_POLL_MS 50

// Scale factors for the default unit selection above (datasheet table 3-6).
#define BNO055_EULER_LSB_TO_DEGREES (1.0f / 16.0f)
#define BNO055_QUATERNION_LSB (1.0f / 16384.0f)
#define BNO055_ACCEL_LSB_TO_MPS2 (1.0f / 100.0f)
#define BNO055_GYRO_LSB_TO_DPS (1.0f / 16.0f)
#define BNO055_MAG_LSB_TO_UT (1.0f / 16.0f)

static bool bus_initialized = false;

/**
 * Configure the I2C peripheral and GPIOs for the BNO055 once.
 *
 * Enables the selected I2C instance, routes SDA/SCL to the peripheral, turns on
 * the internal pull-ups, and waits for the sensor's power-on reset.
 */
static void init_bus(void) {
  if (bus_initialized) {
    return;
  }

  i2c_init(BNO055_I2C, BNO055_I2C_FREQUENCY_HZ);
  gpio_set_function(BNO055_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(BNO055_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(BNO055_I2C_SDA_PIN);
  gpio_pull_up(BNO055_I2C_SCL_PIN);

  // Let the bus and the sensor's power-on reset settle.
  sleep_ms(10);
  bus_initialized = true;
}

/**
 * Read one or more consecutive BNO055 registers.
 *
 * Writes the register address with a repeated start, then reads length bytes.
 * Returns false when either I2C transfer is short.
 */
static bool read_registers(uint8_t reg, uint8_t *buffer, size_t length) {
  if (i2c_write_blocking(
        BNO055_I2C,
        BNO055_I2C_ADDRESS,
        &reg,
        1,
        true) != 1) {
    return false;
  }

  return i2c_read_blocking(
           BNO055_I2C,
           BNO055_I2C_ADDRESS,
           buffer,
           length,
           false) == (int) length;
}

/**
 * Write one BNO055 register over I2C.
 *
 * Sends the register address followed by the value in a single transaction.
 */
static bool write_register(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  return i2c_write_blocking(
           BNO055_I2C,
           BNO055_I2C_ADDRESS,
           buffer,
           sizeof buffer,
           false) == (int) sizeof buffer;
}

/**
 * Decode a little-endian signed 16-bit value from two bytes.
 */
static int16_t decode_int16(const uint8_t *bytes) {
  return (int16_t) ((uint16_t) bytes[0] | ((uint16_t) bytes[1] << 8));
}

/**
 * Read a three-axis sample and convert it to physical units.
 *
 * Reads six bytes starting at reg, decodes three little-endian int16 values,
 * and multiplies each by scale.
 */
static bool read_vector(
  uint8_t reg,
  float scale,
  bno055_vector_t *vector) {
  if (vector == NULL) {
    return false;
  }

  uint8_t raw[6];
  if (!read_registers(reg, raw, sizeof raw)) {
    return false;
  }

  vector->x = (float) decode_int16(&raw[0]) * scale;
  vector->y = (float) decode_int16(&raw[2]) * scale;
  vector->z = (float) decode_int16(&raw[4]) * scale;
  return true;
}

/**
 * Probe the sensor without changing its configuration.
 *
 * Reads the chip ID register and compares it with the BNO055 value 0xA0.
 */
bool bno055_is_connected(void) {
  init_bus();

  uint8_t chip_id = 0;
  return read_registers(BNO055_REG_CHIP_ID, &chip_id, 1) &&
         chip_id == BNO055_CHIP_ID;
}

/**
 * Perform a software reset through SYS_TRIGGER.
 *
 * Waits BNO055_RESET_DELAY_MS for the reset to finish, then polls the chip ID
 * until the sensor answers or BNO055_BOOT_TIMEOUT_MS expires. Leaves the
 * sensor in CONFIG mode.
 */
bool bno055_reset(void) {
  init_bus();

  if (!write_register(
        BNO055_REG_SYSTEM_TRIGGER,
        BNO055_SYSTEM_TRIGGER_RESET)) {
    return false;
  }

  sleep_ms(BNO055_RESET_DELAY_MS);

  absolute_time_t deadline = make_timeout_time_ms(BNO055_BOOT_TIMEOUT_MS);
  do {
    uint8_t chip_id = 0;
    if (read_registers(BNO055_REG_CHIP_ID, &chip_id, 1) &&
        chip_id == BNO055_CHIP_ID) {
      return true;
    }
    sleep_ms(10);
  } while (!time_reached(deadline));

  return false;
}

/**
 * Write a new operation mode and wait for the sensor to apply it.
 */
bool bno055_set_operation_mode(bno055_operation_mode_t mode) {
  init_bus();

  if (!write_register(BNO055_REG_OPERATION_MODE, (uint8_t) mode)) {
    return false;
  }

  sleep_ms(BNO055_MODE_SWITCH_DELAY_MS);
  return true;
}

/**
 * Full BNO055 bring-up.
 *
 * Resets the sensor, selects page 0, normal power, and metric units, switches
 * through CONFIG into NDOF, and verifies the mode register. Returns false at
 * the first failed write or read.
 */
bool bno055_init(void) {
  if (!bno055_reset()) {
    return false;
  }

  if (!write_register(BNO055_REG_PAGE_ID, 0x00)) {
    return false;
  }

  if (!write_register(BNO055_REG_POWER_MODE, BNO055_POWER_MODE_NORMAL)) {
    return false;
  }

  if (!write_register(
        BNO055_REG_UNIT_SELECT,
        BNO055_UNIT_SELECT_METRIC_DEGREES)) {
    return false;
  }

  // NDOF must be entered from CONFIG mode.
  if (!bno055_set_operation_mode(BNO055_MODE_CONFIG)) {
    return false;
  }

  if (!bno055_set_operation_mode(BNO055_MODE_NDOF)) {
    return false;
  }

  uint8_t mode = 0;
  return read_registers(BNO055_REG_OPERATION_MODE, &mode, 1) &&
         mode == (uint8_t) BNO055_MODE_NDOF;
}

/**
 * Read the fused heading, roll, and pitch angles in degrees.
 */
bool bno055_read_euler(bno055_euler_t *euler) {
  if (euler == NULL) {
    return false;
  }

  uint8_t raw[6];
  if (!read_registers(BNO055_REG_EULER_DATA, raw, sizeof raw)) {
    return false;
  }

  euler->heading = (float) decode_int16(&raw[0]) * BNO055_EULER_LSB_TO_DEGREES;
  euler->roll = (float) decode_int16(&raw[2]) * BNO055_EULER_LSB_TO_DEGREES;
  euler->pitch = (float) decode_int16(&raw[4]) * BNO055_EULER_LSB_TO_DEGREES;
  return true;
}

/**
 * Read the fused orientation as a w, x, y, z unit quaternion.
 */
bool bno055_read_quaternion(bno055_quaternion_t *quaternion) {
  if (quaternion == NULL) {
    return false;
  }

  uint8_t raw[8];
  if (!read_registers(BNO055_REG_QUATERNION_DATA, raw, sizeof raw)) {
    return false;
  }

  quaternion->w = (float) decode_int16(&raw[0]) * BNO055_QUATERNION_LSB;
  quaternion->x = (float) decode_int16(&raw[2]) * BNO055_QUATERNION_LSB;
  quaternion->y = (float) decode_int16(&raw[4]) * BNO055_QUATERNION_LSB;
  quaternion->z = (float) decode_int16(&raw[6]) * BNO055_QUATERNION_LSB;
  return true;
}

/**
 * Read the accelerometer vector in m/s^2, including gravity.
 */
bool bno055_read_acceleration(bno055_vector_t *acceleration) {
  return read_vector(
    BNO055_REG_ACCEL_DATA,
    BNO055_ACCEL_LSB_TO_MPS2,
    acceleration);
}

/**
 * Read the gravity-compensated linear acceleration in m/s^2.
 */
bool bno055_read_linear_acceleration(bno055_vector_t *acceleration) {
  return read_vector(
    BNO055_REG_LINEAR_ACCEL_DATA,
    BNO055_ACCEL_LSB_TO_MPS2,
    acceleration);
}

/**
 * Read the gravity vector in m/s^2.
 */
bool bno055_read_gravity(bno055_vector_t *gravity) {
  return read_vector(
    BNO055_REG_GRAVITY_DATA,
    BNO055_ACCEL_LSB_TO_MPS2,
    gravity);
}

/**
 * Read angular velocity in degrees per second.
 */
bool bno055_read_gyroscope(bno055_vector_t *angular_velocity) {
  return read_vector(
    BNO055_REG_GYRO_DATA,
    BNO055_GYRO_LSB_TO_DPS,
    angular_velocity);
}

/**
 * Read the magnetic field in microtesla.
 */
bool bno055_read_magnetometer(bno055_vector_t *magnetic_field) {
  return read_vector(
    BNO055_REG_MAG_DATA,
    BNO055_MAG_LSB_TO_UT,
    magnetic_field);
}

/**
 * Read the die temperature in degrees Celsius.
 */
bool bno055_read_temperature(float *celsius) {
  if (celsius == NULL) {
    return false;
  }

  uint8_t raw = 0;
  if (!read_registers(BNO055_REG_TEMPERATURE, &raw, 1)) {
    return false;
  }

  *celsius = (float) (int8_t) raw;
  return true;
}

/**
 * Read the four calibration levels from CALIB_STAT.
 *
 * Each field is 0 (uncalibrated) through 3 (fully calibrated).
 */
bool bno055_read_calibration(bno055_calibration_t *calibration) {
  if (calibration == NULL) {
    return false;
  }

  uint8_t raw = 0;
  if (!read_registers(BNO055_REG_CALIBRATION_STATUS, &raw, 1)) {
    return false;
  }

  calibration->system = (raw >> 6) & 0x03;
  calibration->gyro = (raw >> 4) & 0x03;
  calibration->accel = (raw >> 2) & 0x03;
  calibration->mag = raw & 0x03;
  return true;
}

/**
 * Poll calibration status until every sensor reaches level 3.
 *
 * Checks immediately, then every BNO055_CALIBRATION_POLL_MS until timeout_ms
 * expires. A timeout of 0 performs a single check.
 *
 * @return true when system, gyro, accel, and mag are all fully calibrated.
 */
bool bno055_wait_for_calibration(uint32_t timeout_ms) {
  absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

  do {
    bno055_calibration_t calibration;
    if (bno055_read_calibration(&calibration)) {
      bool fully_calibrated =
        calibration.system == 3 &&
        calibration.gyro == 3 &&
        calibration.accel == 3 &&
        calibration.mag == 3;
      if (fully_calibrated) {
        return true;
      }
    }

    if (time_reached(deadline)) {
      break;
    }
    sleep_ms(BNO055_CALIBRATION_POLL_MS);
  } while (true);

  return false;
}
