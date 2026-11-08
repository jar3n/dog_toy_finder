#include "motor/tb6612fng.h"

#include <stddef.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "config/motor_config.h"
#include "config/pinout.h"

#if MOTOR_PWM_FREQUENCY_HZ < 1
#error "MOTOR_PWM_FREQUENCY_HZ must be at least 1"
#endif

/**
 * Return the channel state for a motor ID, or NULL for an invalid ID.
 */
static tb6612fng_channel_t *channel_for(
  tb6612fng_t *driver,
  tb6612fng_motor_id_t motor) {
  if (driver == NULL) {
    return NULL;
  }
  if (motor == TB6612FNG_MOTOR_LEFT) {
    return &driver->left;
  }
  if (motor == TB6612FNG_MOTOR_RIGHT) {
    return &driver->right;
  }
  return NULL;
}

/**
 * Return true when every pin in the array is different from the others.
 *
 * A duplicate would make two functions drive the same GPIO, so the driver
 * refuses to initialize instead of producing unpredictable motor behavior.
 */
static bool pins_are_unique(const uint *pins, size_t count) {
  for (size_t first = 0; first < count; ++first) {
    for (size_t second = first + 1; second < count; ++second) {
      if (pins[first] == pins[second]) {
        return false;
      }
    }
  }
  return true;
}

/**
 * Return true when a motor pin overlaps a pin owned by another module.
 *
 * The encoder and IMU pins are reserved here because configuring one of them
 * as a motor output would silently break that peripheral. The wireless pins
 * are fixed by the board and are not part of the configurable pin map.
 */
static bool motor_pins_overlap_reserved(const uint *motor_pins, size_t count) {
  const uint reserved[] = {
    ENCODER_LEFT_PIN_A,
    ENCODER_LEFT_PIN_B,
    ENCODER_RIGHT_PIN_A,
    ENCODER_RIGHT_PIN_B,
    BNO055_I2C_SDA_PIN,
    BNO055_I2C_SCL_PIN,
  };
  for (size_t motor_index = 0; motor_index < count; ++motor_index) {
    for (size_t reserved_index = 0;
         reserved_index < sizeof reserved / sizeof reserved[0];
         ++reserved_index) {
      if (motor_pins[motor_index] == reserved[reserved_index]) {
        return true;
      }
    }
  }
  return false;
}

/**
 * Configure the direction pins and PWM slice for one motor channel.
 *
 * The direction pins start low (coast) and the PWM output starts at zero so
 * the motor cannot move until tb6612fng_set_speed() is called.
 */
static bool init_channel(
  tb6612fng_channel_t *channel,
  uint pwm_pin,
  uint in1_pin,
  uint in2_pin,
  bool invert) {
  if (channel == NULL) {
    return false;
  }

  channel->pwm_pin = pwm_pin;
  channel->in1_pin = in1_pin;
  channel->in2_pin = in2_pin;
  channel->invert = invert;
  channel->speed = 0.0f;
  channel->initialized = false;

  gpio_init(in1_pin);
  gpio_set_dir(in1_pin, GPIO_OUT);
  gpio_put(in1_pin, 0);
  gpio_init(in2_pin);
  gpio_set_dir(in2_pin, GPIO_OUT);
  gpio_put(in2_pin, 0);

  gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

  uint slice = pwm_gpio_to_slice_num(pwm_pin);
  uint pwm_channel = pwm_gpio_to_channel(pwm_pin);

  // Keep the wrap inside the 16-bit counter. For low frequencies the wrap at
  // divider 1 would overflow, so raise the PWM clock divider first and then
  // derive the wrap from the divided clock. The TB6612FNG itself has no
  // practical lower PWM limit; the RP2350 divider caps the minimum at about
  // 9 Hz with the default 150 MHz system clock.
  uint32_t sys_hz = clock_get_hz(clk_sys);
  float clkdiv = 1.0f;
  uint32_t wrap = sys_hz / MOTOR_PWM_FREQUENCY_HZ;
  if (wrap > 65536u) {
    clkdiv = (float) sys_hz /
      ((float) MOTOR_PWM_FREQUENCY_HZ * 65536.0f);
    if (clkdiv > 255.9375f) {
      clkdiv = 255.9375f;
    }
    wrap = (uint32_t) ((float) sys_hz /
      ((float) MOTOR_PWM_FREQUENCY_HZ * clkdiv));
  }
  if (wrap < 2u) {
    wrap = 2u;
  } else if (wrap > 65536u) {
    wrap = 65536u;
  }

  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, clkdiv);
  pwm_config_set_wrap(&config, (uint16_t) (wrap - 1u));
  pwm_init(slice, &config, true);
  pwm_set_chan_level(slice, pwm_channel, 0);

  channel->pwm_slice = slice;
  channel->pwm_channel = pwm_channel;
  channel->pwm_wrap = (uint16_t) (wrap - 1u);
  channel->initialized = true;
  return true;
}

/**
 * Turn both direction pins low and remove the PWM signal (coast).
 */
static void coast_channel(tb6612fng_channel_t *channel) {
  pwm_set_chan_level(channel->pwm_slice, channel->pwm_channel, 0);
  gpio_put(channel->in1_pin, 0);
  gpio_put(channel->in2_pin, 0);
  channel->speed = 0.0f;
}

/**
 * Turn both direction pins high with PWM off for an active short brake.
 */
static void brake_channel(tb6612fng_channel_t *channel) {
  pwm_set_chan_level(channel->pwm_slice, channel->pwm_channel, 0);
  gpio_put(channel->in1_pin, 1);
  gpio_put(channel->in2_pin, 1);
  channel->speed = 0.0f;
}

/**
 * Initialize both TB6612FNG H-bridges and leave them coasting.
 *
 * Standby is held low while the pins and PWM slices are configured, then
 * raised at the end so a partially configured bridge cannot drive a motor.
 */
bool tb6612fng_init(tb6612fng_t *driver) {
  if (driver == NULL) {
    return false;
  }

  // Start from a known state so a failed init is still safe to inspect.
  *driver = (tb6612fng_t){0};

  const uint pins[] = {
    MOTOR_PWMA_PIN,
    MOTOR_AIN1_PIN,
    MOTOR_AIN2_PIN,
    MOTOR_PWMB_PIN,
    MOTOR_BIN1_PIN,
    MOTOR_BIN2_PIN,
    MOTOR_STBY_PIN,
  };
  if (!pins_are_unique(pins, sizeof pins / sizeof pins[0]) ||
      motor_pins_overlap_reserved(pins, sizeof pins / sizeof pins[0])) {
    return false;
  }

  driver->standby_pin = MOTOR_STBY_PIN;
  driver->enabled = false;
  driver->initialized = false;

  // Keep the bridge disabled until every output is configured.
  gpio_init(MOTOR_STBY_PIN);
  gpio_set_dir(MOTOR_STBY_PIN, GPIO_OUT);
  gpio_put(MOTOR_STBY_PIN, 0);

  if (!init_channel(
        &driver->left,
        MOTOR_PWMA_PIN,
        MOTOR_AIN1_PIN,
        MOTOR_AIN2_PIN,
        MOTOR_LEFT_INVERT != 0)) {
    return false;
  }

  if (!init_channel(
        &driver->right,
        MOTOR_PWMB_PIN,
        MOTOR_BIN1_PIN,
        MOTOR_BIN2_PIN,
        MOTOR_RIGHT_INVERT != 0)) {
    return false;
  }

  driver->initialized = true;
  driver->enabled = true;
  gpio_put(MOTOR_STBY_PIN, 1);
  tb6612fng_stop_all(driver, false);
  return true;
}

/**
 * Apply a signed speed to one motor channel.
 *
 * Positive values drive forward, negative values reverse, and values near zero
 * coast. The invert flag swaps direction without changing the reported logical
 * speed.
 */
bool tb6612fng_set_speed(
  tb6612fng_t *driver,
  tb6612fng_motor_id_t motor,
  float speed) {
  if (driver == NULL || !driver->initialized || !driver->enabled) {
    return false;
  }

  tb6612fng_channel_t *channel = channel_for(driver, motor);
  if (channel == NULL || !channel->initialized) {
    return false;
  }

  float limit = MOTOR_MAX_DUTY;
  if (limit < 0.0f) {
    limit = 0.0f;
  } else if (limit > 1.0f) {
    limit = 1.0f;
  }
  if (speed > limit) {
    speed = limit;
  } else if (speed < -limit) {
    speed = -limit;
  }

  float logical_speed = speed;
  if (channel->invert) {
    speed = -speed;
  }

  float magnitude = speed < 0.0f ? -speed : speed;
  if (magnitude < 0.001f) {
    coast_channel(channel);
    return true;
  }

  if (speed > 0.0f) {
    gpio_put(channel->in1_pin, 1);
    gpio_put(channel->in2_pin, 0);
  } else {
    gpio_put(channel->in1_pin, 0);
    gpio_put(channel->in2_pin, 1);
  }

  uint32_t level = (uint32_t) (magnitude * (float) channel->pwm_wrap);
  if (level > channel->pwm_wrap) {
    level = channel->pwm_wrap;
  }
  pwm_set_chan_level(
    channel->pwm_slice,
    channel->pwm_channel,
    (uint16_t) level);

  channel->speed = logical_speed;
  return true;
}

/**
 * Enable or disable the bridge and force both channels to coast.
 */
void tb6612fng_set_enabled(tb6612fng_t *driver, bool enabled) {
  if (driver == NULL || !driver->initialized) {
    return;
  }

  tb6612fng_stop_all(driver, false);
  gpio_put(driver->standby_pin, enabled ? 1 : 0);
  driver->enabled = enabled;
}

/**
 * Stop one motor by coasting or active braking.
 */
void tb6612fng_stop(
  tb6612fng_t *driver,
  tb6612fng_motor_id_t motor,
  bool brake) {
  if (driver == NULL || !driver->initialized) {
    return;
  }

  tb6612fng_channel_t *channel = channel_for(driver, motor);
  if (channel == NULL || !channel->initialized) {
    return;
  }

  if (brake) {
    brake_channel(channel);
  } else {
    coast_channel(channel);
  }
}

/**
 * Stop both motors with the requested braking mode.
 */
void tb6612fng_stop_all(tb6612fng_t *driver, bool brake) {
  if (driver == NULL || !driver->initialized) {
    return;
  }

  tb6612fng_stop(driver, TB6612FNG_MOTOR_LEFT, brake);
  tb6612fng_stop(driver, TB6612FNG_MOTOR_RIGHT, brake);
}

/**
 * Return the last logical speed requested for a motor.
 */
float tb6612fng_get_speed(
  const tb6612fng_t *driver,
  tb6612fng_motor_id_t motor) {
  if (driver == NULL) {
    return 0.0f;
  }
  if (motor == TB6612FNG_MOTOR_LEFT) {
    return driver->left.speed;
  }
  if (motor == TB6612FNG_MOTOR_RIGHT) {
    return driver->right.speed;
  }
  return 0.0f;
}
