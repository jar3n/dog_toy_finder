#pragma once

#include <stddef.h>
#include <stdint.h>

#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"

#define PICOW_UDP_RECEIVE_QUEUE_DEPTH 4

typedef struct {
  const char *agent_ip;
  uint16_t agent_port;
  struct udp_pcb *pcb;
  ip_addr_t agent_address;
  uint8_t receive_buffers[PICOW_UDP_RECEIVE_QUEUE_DEPTH]
                         [UXR_CONFIG_CUSTOM_TRANSPORT_MTU];
  size_t receive_lengths[PICOW_UDP_RECEIVE_QUEUE_DEPTH];
  size_t receive_head;
  size_t receive_tail;
  size_t receive_count;
} picow_udp_transport_context_t;

/**
 * Open the UDP socket connected to the micro-ROS agent.
 *
 * Resolves the configured agent address, creates the PCB, and starts queueing
 * incoming datagrams for picow_udp_transport_read().
 *
 * @return true when the socket is connected.
 */
bool picow_udp_transport_open(struct uxrCustomTransport *transport);

/**
 * Close the UDP socket and discard any queued datagrams.
 *
 * @return true when the transport context was valid.
 */
bool picow_udp_transport_close(struct uxrCustomTransport *transport);

/**
 * Send one XRCE frame to the micro-ROS agent.
 *
 * @return the number of bytes sent, or 0 on failure. When non-NULL,
 *         error_code is set to 1 on failure and 0 on success.
 */
size_t picow_udp_transport_write(
  struct uxrCustomTransport *transport,
  const uint8_t *buffer,
  size_t length,
  uint8_t *error_code);

/**
 * Read the next queued XRCE frame from the agent.
 *
 * Waits up to timeout milliseconds when the queue is empty.
 *
 * @return the number of bytes copied into buffer, or 0 on timeout/failure.
 *         When non-NULL, error_code is set to 1 when the caller's buffer is
 *         too small for the queued datagram.
 */
size_t picow_udp_transport_read(
  struct uxrCustomTransport *transport,
  uint8_t *buffer,
  size_t length,
  int timeout,
  uint8_t *error_code);
