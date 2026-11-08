#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "encoder/quadrature_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Position and velocity estimate for one wheel encoder.
 *
 * Wraps a quadrature_encoder_t and converts raw counts to output-shaft
 * radians and radians per second. Initialize with encoder_init() and call
 * encoder_update() at a steady rate.
 */
typedef struct {
  /** Underlying 4x quadrature decoder. */
  quadrature_encoder_t quadrature;
  /** Output-shaft radians represented by one quadrature count. */
  double radians_per_count;
  /** Latest output-shaft position in radians. */
  double position_rad;
  /** Latest filtered output-shaft velocity in radians per second. */
  double velocity_rad_s;
  /** Monotonic count accumulated across wraps of the 32-bit raw counter. */
  int64_t total_count;
  /** Raw count captured at the previous position update. */
  int32_t position_count;
  /** Count captured at the previous velocity interval. */
  int32_t last_count;
  /** Timestamp of the previous update in microseconds. */
  uint64_t last_update_us;
  /** True once a first sample has established the time base. */
  bool has_last_update;
  /** True once a velocity estimate exists and filtering can start. */
  bool has_velocity;
  /** Low-pass weight for new velocity samples: 1.0 disables filtering. */
  float velocity_filter_alpha;
} encoder_t;

/**
 * Initialize the GPIO interrupt decoder and the counts-to-radians conversion.
 *
 * @param encoder               Caller-owned state to initialize.
 * @param pin_a                 GPIO for encoder channel A.
 * @param pin_b                 GPIO for encoder channel B.
 * @param invert                Invert the counting direction for this side.
 * @param counts_per_revolution Total 4x counts for one output-shaft turn
 *                              (encoder PPR * gear ratio * 4).
 * @param velocity_filter_alpha Low-pass weight in [0, 1]; values above 1 are
 *                              clamped and 1 disables smoothing.
 *
 * @return true when the quadrature decoder was configured.
 */
bool encoder_init(
  encoder_t *encoder,
  uint pin_a,
  uint pin_b,
  bool invert,
  double counts_per_revolution,
  float velocity_filter_alpha);

/**
 * Sample the decoder, refresh position, and update the filtered velocity.
 *
 * Position is always derived from the absolute count, so it stays correct even
 * if updates are irregular. Velocity is a finite difference over the interval
 * since the previous sample, smoothed by the configured alpha. Samples closer
 * together than 1 ms keep the previous velocity, which avoids divide-by-zero
 * and noise when callers poll faster than the control rate.
 *
 * @param now_us Current monotonic time in microseconds (time_us_64()).
 */
void encoder_update(encoder_t *encoder, uint64_t now_us);

/**
 * Zero the decoder count, position, and velocity.
 *
 * The next encoder_update() starts a new velocity interval so the reset does
 * not produce a spike.
 */
void encoder_reset(encoder_t *encoder, uint64_t now_us);

/**
 * Return the latest output-shaft position in radians.
 */
double encoder_position_rad(const encoder_t *encoder);

/**
 * Return the latest filtered output-shaft velocity in radians per second.
 */
double encoder_velocity_rad_s(const encoder_t *encoder);

/**
 * Return the current signed quadrature count.
 */
int32_t encoder_count(const encoder_t *encoder);

#ifdef __cplusplus
}
#endif
