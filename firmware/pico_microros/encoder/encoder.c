#include "encoder/encoder.h"

#include <stddef.h>

#define MICROSECONDS_PER_SECOND 1000000.0
#define RADIANS_PER_REVOLUTION 6.28318530717958647692
#define MIN_VELOCITY_INTERVAL_US 1000

/**
 * Return the signed step between two snapshots of the 32-bit raw counter.
 *
 * Unsigned subtraction makes the wrap from INT32_MAX to INT32_MIN well
 * defined; the cast reinterprets the modular result as a signed delta.
 */
static int32_t count_difference(int32_t current, int32_t previous) {
  return (int32_t) ((uint32_t) current - (uint32_t) previous);
}

/**
 * Initialize the quadrature decoder and kinematics for one wheel.
 */
bool encoder_init(
  encoder_t *encoder,
  uint pin_a,
  uint pin_b,
  bool invert,
  double counts_per_revolution,
  float velocity_filter_alpha) {
  if (encoder == NULL || counts_per_revolution <= 0.0) {
    return false;
  }

  if (!quadrature_encoder_init(
        &encoder->quadrature,
        pin_a,
        pin_b,
        invert)) {
    return false;
  }

  if (velocity_filter_alpha < 0.0f) {
    velocity_filter_alpha = 0.0f;
  } else if (velocity_filter_alpha > 1.0f) {
    velocity_filter_alpha = 1.0f;
  }

  encoder->radians_per_count =
    RADIANS_PER_REVOLUTION / counts_per_revolution;
  encoder->position_rad = 0.0;
  encoder->velocity_rad_s = 0.0;
  encoder->total_count = 0;
  encoder->position_count = 0;
  encoder->last_count = 0;
  encoder->last_update_us = 0;
  encoder->has_last_update = false;
  encoder->has_velocity = false;
  encoder->velocity_filter_alpha = velocity_filter_alpha;
  return true;
}

/**
 * Sample the count and update position and filtered velocity.
 */
void encoder_update(encoder_t *encoder, uint64_t now_us) {
  if (encoder == NULL) {
    return;
  }

  int32_t count = quadrature_encoder_count(&encoder->quadrature);

  // Accumulate 32-bit deltas so position stays continuous if the raw counter
  // wraps after a very long run.
  int32_t position_delta =
    count_difference(count, encoder->position_count);
  encoder->total_count += position_delta;
  encoder->position_count = count;
  encoder->position_rad =
    (double) encoder->total_count * encoder->radians_per_count;

  if (!encoder->has_last_update) {
    encoder->last_count = count;
    encoder->last_update_us = now_us;
    encoder->has_last_update = true;
    return;
  }

  uint64_t elapsed_us = now_us - encoder->last_update_us;
  int32_t delta_count = count_difference(count, encoder->last_count);

  // Only estimate velocity when enough time has passed for a useful quotient.
  // Counts from skipped intervals are still reflected in the next sample
  // because last_count only advances with a valid interval.
  if (elapsed_us >= MIN_VELOCITY_INTERVAL_US) {
    double raw_velocity =
      (double) delta_count * encoder->radians_per_count *
      MICROSECONDS_PER_SECOND / (double) elapsed_us;

    if (!encoder->has_velocity) {
      encoder->velocity_rad_s = raw_velocity;
      encoder->has_velocity = true;
    } else {
      encoder->velocity_rad_s =
        (double) encoder->velocity_filter_alpha * raw_velocity +
        (1.0 - (double) encoder->velocity_filter_alpha) *
        encoder->velocity_rad_s;
    }

    encoder->last_count = count;
    encoder->last_update_us = now_us;
  }
}

/**
 * Zero the decoder plus all derived position and velocity state.
 */
void encoder_reset(encoder_t *encoder, uint64_t now_us) {
  if (encoder == NULL) {
    return;
  }

  quadrature_encoder_reset(&encoder->quadrature);
  encoder->position_rad = 0.0;
  encoder->velocity_rad_s = 0.0;
  encoder->total_count = 0;
  encoder->position_count = 0;
  encoder->last_count = 0;
  encoder->last_update_us = now_us;
  encoder->has_last_update = false;
  encoder->has_velocity = false;
}

/**
 * Return the last computed output-shaft position in radians.
 */
double encoder_position_rad(const encoder_t *encoder) {
  return encoder == NULL ? 0.0 : encoder->position_rad;
}

/**
 * Return the last computed output-shaft velocity in radians per second.
 */
double encoder_velocity_rad_s(const encoder_t *encoder) {
  return encoder == NULL ? 0.0 : encoder->velocity_rad_s;
}

/**
 * Return the current signed quadrature count.
 */
int32_t encoder_count(const encoder_t *encoder) {
  return encoder == NULL ? 0 : quadrature_encoder_count(&encoder->quadrature);
}
