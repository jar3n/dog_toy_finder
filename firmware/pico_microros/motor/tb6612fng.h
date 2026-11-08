#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pico/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Motor channel identifiers for the TB6612FNG's two H-bridge outputs.
 */
typedef enum {
  TB6612FNG_MOTOR_LEFT = 0,
  TB6612FNG_MOTOR_RIGHT = 1,
} tb6612fng_motor_id_t;

/**
 * Runtime state for one TB6612FNG channel.
 *
 * Initialize through tb6612fng_init(); the pin and PWM fields are filled in
 * from config/pinout.h. Do not modify them while the driver is running.
 */
typedef struct {
  /** GPIO connected to PWMA or PWMB. */
  uint pwm_pin;
  /** GPIO connected to AIN1 or BIN1. */
  uint in1_pin;
  /** GPIO connected to AIN2 or BIN2. */
  uint in2_pin;
  /** PWM slice that owns pwm_pin. */
  uint pwm_slice;
  /** PWM channel (A or B) that owns pwm_pin. */
  uint pwm_channel;
  /** Wrap value configured for the slice's PWM period. */
  uint16_t pwm_wrap;
  /** When true, a positive command spins the motor in the reverse direction. */
  bool invert;
  /** Last logical speed requested, in the range -MOTOR_MAX_DUTY..MOTOR_MAX_DUTY. */
  float speed;
  /** True once the channel's GPIOs and PWM slice are configured. */
  bool initialized;
} tb6612fng_channel_t;

/**
 * State for one TB6612FNG dual motor controller.
 *
 * Initialize with tb6612fng_init() before calling any other function.
 */
typedef struct {
  /** Left motor channel (PWMA/AIN1/AIN2). */
  tb6612fng_channel_t left;
  /** Right motor channel (PWMB/BIN1/BIN2). */
  tb6612fng_channel_t right;
  /** GPIO connected to STBY. */
  uint standby_pin;
  /** True when the bridge is enabled (STBY high). */
  bool enabled;
  /** True once tb6612fng_init() completed successfully. */
  bool initialized;
} tb6612fng_t;

/**
 * Configure both H-bridges, the PWM slices, and the standby pin.
 *
 * Uses the MOTOR_* pin assignments from config/pinout.h and the PWM/direction
 * settings from config/motor_config.h. The bridge is left enabled (STBY high)
 * with both channels coasting, so the motors do not move until a speed is set.
 *
 * @return true on success, false for a NULL/invalid driver or pin conflict.
 */
bool tb6612fng_init(tb6612fng_t *driver);

/**
 * Set one motor's signed speed.
 *
 * @param speed -MOTOR_MAX_DUTY (full reverse) through 0 (coast) to
 *              +MOTOR_MAX_DUTY (full forward). Values outside the range are
 *              clamped.
 *
 * @return true when the command was applied, false when the driver is not
 *         initialized or is disabled.
 */
bool tb6612fng_set_speed(
  tb6612fng_t *driver,
  tb6612fng_motor_id_t motor,
  float speed);

/**
 * Enable or disable the H-bridge outputs through STBY.
 *
 * Disabling first coasts both channels and then pulls STBY low. Enabling
 * raises STBY and leaves both channels coasting until the next set_speed().
 */
void tb6612fng_set_enabled(tb6612fng_t *driver, bool enabled);

/**
 * Stop one motor.
 *
 * @param brake true for an active short brake (both direction pins high),
 *              false to coast (both direction pins low, PWM off).
 */
void tb6612fng_stop(
  tb6612fng_t *driver,
  tb6612fng_motor_id_t motor,
  bool brake);

/** Stop both motors with the requested braking mode. */
void tb6612fng_stop_all(tb6612fng_t *driver, bool brake);

/**
 * @return the last logical speed requested for a motor, or 0 when the driver
 *         or motor ID is invalid.
 */
float tb6612fng_get_speed(
  const tb6612fng_t *driver,
  tb6612fng_motor_id_t motor);

#ifdef __cplusplus
}
#endif
