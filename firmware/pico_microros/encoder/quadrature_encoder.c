#include "encoder/quadrature_encoder.h"

#include <stddef.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// Decode one transition using (previous_state << 2) | current_state, where a
// state is (A << 1) | B. A valid single-channel step is +1 or -1; a double
// step (both channels changed, usually a missed sample or a glitch) and a
// no-change transition both produce 0. This is the standard full 4x table.
static const int8_t QUADRATURE_TRANSITION[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0,
};

// Encoders attached to the shared GPIO interrupt handler. The RP2350 SDK
// allows only one bank-0 callback, so the handler dispatches by GPIO number.
static quadrature_encoder_t *encoder_instances[QUADRATURE_ENCODER_MAX_INSTANCES];
static size_t encoder_instance_count = 0;
static bool irq_handler_installed = false;

/**
 * Read both channel pins and pack them into an (A << 1) | B state value.
 */
static uint8_t read_channel_state(const quadrature_encoder_t *encoder) {
  uint8_t channel_a = gpio_get(encoder->pin_a) ? 1u : 0u;
  uint8_t channel_b = gpio_get(encoder->pin_b) ? 1u : 0u;
  return (uint8_t) ((channel_a << 1) | channel_b);
}

/**
 * Shared GPIO interrupt handler for every registered quadrature encoder.
 *
 * Finds the encoder whose A or B pin caused the interrupt, samples the new
 * channel state, looks up the transition in QUADRATURE_TRANSITION, and applies
 * it to the signed count. Unknown GPIOs are ignored so other peripherals can
 * share the bank interrupt.
 */
static void quadrature_encoder_irq(uint gpio, uint32_t events) {
  (void) events;

  for (size_t index = 0; index < encoder_instance_count; ++index) {
    quadrature_encoder_t *encoder = encoder_instances[index];
    if (gpio != encoder->pin_a && gpio != encoder->pin_b) {
      continue;
    }

    uint8_t current_state = read_channel_state(encoder);
    uint8_t transition = (uint8_t) ((encoder->last_state << 2) | current_state);
    int8_t delta = QUADRATURE_TRANSITION[transition];
    if (encoder->invert) {
      delta = (int8_t) -delta;
    }

    // Unsigned addition keeps the 32-bit wrap well defined for -1 steps.
    encoder->count += (uint32_t) delta;
    encoder->last_state = current_state;
    return;
  }
}

/**
 * Configure one channel pin as a pulled-up input that interrupts on both edges.
 */
static void configure_channel_pin(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_pull_up(pin);
  gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

/**
 * Attach a pair of GPIOs to the shared quadrature decoder.
 *
 * The first call installs the bank-0 callback and enables the bank interrupt;
 * later calls only add pins. The instance is registered before its interrupts
 * are enabled so an edge cannot arrive before last_state is valid.
 */
bool quadrature_encoder_init(
  quadrature_encoder_t *encoder,
  uint pin_a,
  uint pin_b,
  bool invert) {
  if (encoder == NULL || pin_a == pin_b) {
    return false;
  }
  if (encoder_instance_count >= QUADRATURE_ENCODER_MAX_INSTANCES) {
    return false;
  }

  encoder->pin_a = pin_a;
  encoder->pin_b = pin_b;
  encoder->invert = invert;
  encoder->count = 0;

  gpio_init(pin_a);
  gpio_init(pin_b);
  gpio_set_dir(pin_a, GPIO_IN);
  gpio_set_dir(pin_b, GPIO_IN);
  gpio_pull_up(pin_a);
  gpio_pull_up(pin_b);

  // Seed the decoder with the real pin levels before enabling interrupts.
  encoder->last_state = read_channel_state(encoder);

  encoder_instances[encoder_instance_count++] = encoder;

  if (!irq_handler_installed) {
    gpio_set_irq_callback(quadrature_encoder_irq);
    irq_set_enabled(IO_IRQ_BANK0, true);
    irq_handler_installed = true;
  }

  configure_channel_pin(pin_a);
  configure_channel_pin(pin_b);
  return true;
}

/**
 * Return a consistent snapshot of the accumulated count.
 */
int32_t quadrature_encoder_count(const quadrature_encoder_t *encoder) {
  if (encoder == NULL) {
    return 0;
  }

  uint32_t interrupt_state = save_and_disable_interrupts();
  int32_t count = (int32_t) encoder->count;
  restore_interrupts(interrupt_state);
  return count;
}

/**
 * Zero the count and resynchronize the decoder with the current pin levels.
 */
void quadrature_encoder_reset(quadrature_encoder_t *encoder) {
  if (encoder == NULL) {
    return;
  }

  uint32_t interrupt_state = save_and_disable_interrupts();
  encoder->count = 0;
  encoder->last_state = read_channel_state(encoder);
  restore_interrupts(interrupt_state);
}
