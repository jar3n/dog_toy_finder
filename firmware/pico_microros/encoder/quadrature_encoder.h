#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pico/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of encoders the shared GPIO interrupt handler can track.
#ifndef QUADRATURE_ENCODER_MAX_INSTANCES
#define QUADRATURE_ENCODER_MAX_INSTANCES 4
#endif

/**
 * State for one hall-effect quadrature encoder channel pair.
 *
 * Initialize the struct with quadrature_encoder_init() before use. The count,
 * last_state, and wiring fields are updated from the GPIO interrupt handler,
 * so do not modify them directly while the encoder is running.
 */
typedef struct {
  /** GPIO connected to encoder channel A. */
  uint pin_a;
  /** GPIO connected to encoder channel B. */
  uint pin_b;
  /** When true, invert the decoded direction for this encoder. */
  bool invert;
  /** Raw 4x counter, updated on every edge of A and B. Read it through
   *  quadrature_encoder_count() so it is interpreted as a signed value. */
  volatile uint32_t count;
  /** Last sampled two-bit channel state, encoded as (A << 1) | B. */
  volatile uint8_t last_state;
} quadrature_encoder_t;

/**
 * Configure two GPIOs as a quadrature encoder input.
 *
 * Enables the internal pull-ups, records the initial A/B state, and installs
 * the shared bank-0 GPIO interrupt handler on the first call. Both channels
 * generate an interrupt on every rising and falling edge, giving full 4x
 * decoding (12 PPR * 90 gear ratio * 4 = 4320 counts per output revolution
 * for the TT encoded motor).
 *
 * @return true on success, false for NULL/invalid arguments or when the
 *         maximum number of instances has been reached.
 */
bool quadrature_encoder_init(
  quadrature_encoder_t *encoder,
  uint pin_a,
  uint pin_b,
  bool invert);

/**
 * Read the signed count without tearing it.
 *
 * Disables interrupts briefly so the value cannot change while the 32-bit
 * count is copied.
 *
 * @return the accumulated count; positive and negative directions depend on
 *         the wiring and the invert flag.
 */
int32_t quadrature_encoder_count(const quadrature_encoder_t *encoder);

/**
 * Reset the count to zero and resynchronize the decoder state.
 *
 * Reads the current A/B levels under a critical section so the next edge is
 * decoded against the actual pin state rather than a stale one.
 */
void quadrature_encoder_reset(quadrature_encoder_t *encoder);

#ifdef __cplusplus
}
#endif
