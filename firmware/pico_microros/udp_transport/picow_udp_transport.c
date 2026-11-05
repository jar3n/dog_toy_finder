#include "picow_udp_transport.h"

#include <string.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

/**
 * lwIP receive callback for the micro-ROS transport.
 *
 * Copies an incoming datagram into the context's ring buffer for
 * picow_udp_transport_read() to consume. Packets are dropped when the queue is
 * full or larger than the transport MTU. The pbuf is always freed here.
 */
static void picow_udp_receive(
  void *argument,
  struct udp_pcb *pcb,
  struct pbuf *packet,
  const ip_addr_t *address,
  u16_t port) {
  (void) pcb;
  (void) address;
  (void) port;

  picow_udp_transport_context_t *context = argument;
  if (packet == NULL) {
    return;
  }

  if (context != NULL &&
    context->receive_count < PICOW_UDP_RECEIVE_QUEUE_DEPTH &&
    packet->tot_len <= UXR_CONFIG_CUSTOM_TRANSPORT_MTU) {
    size_t tail = context->receive_tail;
    context->receive_lengths[tail] = pbuf_copy_partial(
      packet,
      context->receive_buffers[tail],
      packet->tot_len,
      0);
    context->receive_tail = (tail + 1) % PICOW_UDP_RECEIVE_QUEUE_DEPTH;
    context->receive_count++;
  }

  pbuf_free(packet);
}

/**
 * Open the custom transport used by micro-ROS.
 *
 * Parses the configured agent address, creates an IPv4 UDP PCB, connects it to
 * the agent, and registers picow_udp_receive() as the receive callback.
 */
bool picow_udp_transport_open(struct uxrCustomTransport *transport) {
  if (transport == NULL || transport->args == NULL) {
    return false;
  }

  picow_udp_transport_context_t *context = transport->args;
  if (!ipaddr_aton(context->agent_ip, &context->agent_address)) {
    return false;
  }

  context->receive_head = 0;
  context->receive_tail = 0;
  context->receive_count = 0;
  cyw43_arch_lwip_begin();
  context->pcb = udp_new_ip_type(IPADDR_TYPE_V4);
  if (context->pcb == NULL) {
    cyw43_arch_lwip_end();
    return false;
  }

  err_t result = udp_connect(
    context->pcb,
    &context->agent_address,
    context->agent_port);
  if (result == ERR_OK) {
    udp_recv(context->pcb, picow_udp_receive, context);
  } else {
    udp_remove(context->pcb);
    context->pcb = NULL;
  }
  cyw43_arch_lwip_end();

  return result == ERR_OK;
}

/**
 * Close the transport's UDP PCB and clear the receive queue.
 *
 * Safe to call when the transport has not been opened.
 */
bool picow_udp_transport_close(struct uxrCustomTransport *transport) {
  if (transport == NULL || transport->args == NULL) {
    return false;
  }

  picow_udp_transport_context_t *context = transport->args;
  cyw43_arch_lwip_begin();
  if (context->pcb != NULL) {
    udp_recv(context->pcb, NULL, NULL);
    udp_remove(context->pcb);
    context->pcb = NULL;
  }
  context->receive_head = 0;
  context->receive_tail = 0;
  context->receive_count = 0;
  cyw43_arch_lwip_end();

  return true;
}

/**
 * Send one XRCE frame to the agent over UDP.
 *
 * Allocates and fills an lwIP pbuf, sends it on the connected PCB, and returns
 * the accepted byte count. Sets *error_code to 1 on any failure.
 */
size_t picow_udp_transport_write(
  struct uxrCustomTransport *transport,
  const uint8_t *buffer,
  size_t length,
  uint8_t *error_code) {
  if (error_code != NULL) {
    *error_code = 0;
  }
  if (transport == NULL || transport->args == NULL || buffer == NULL) {
    if (error_code != NULL) {
      *error_code = 1;
    }
    return 0;
  }

  picow_udp_transport_context_t *context = transport->args;
  if (context->pcb == NULL || length > UINT16_MAX) {
    if (error_code != NULL) {
      *error_code = 1;
    }
    return 0;
  }

  cyw43_arch_lwip_begin();
  struct pbuf *packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t) length, PBUF_RAM);
  err_t result = packet == NULL ? ERR_MEM : pbuf_take(packet, buffer, length);
  if (result == ERR_OK) {
    result = udp_send(context->pcb, packet);
  }
  if (packet != NULL) {
    pbuf_free(packet);
  }
  cyw43_arch_lwip_end();

  if (result != ERR_OK) {
    if (error_code != NULL) {
      *error_code = 1;
    }
    return 0;
  }
  return length;
}

/**
 * Read the next XRCE frame from the receive queue.
 *
 * Copies the oldest queued datagram into buffer and returns its length. Blocks
 * in 1 ms steps until timeout milliseconds elapse, or returns immediately when
 * timeout is not positive. Sets *error_code and returns 0 when the caller's
 * buffer is too small for the queued datagram.
 */
size_t picow_udp_transport_read(
  struct uxrCustomTransport *transport,
  uint8_t *buffer,
  size_t length,
  int timeout,
  uint8_t *error_code) {
  if (error_code != NULL) {
    *error_code = 0;
  }
  if (transport == NULL || transport->args == NULL || buffer == NULL) {
    if (error_code != NULL) {
      *error_code = 1;
    }
    return 0;
  }

  picow_udp_transport_context_t *context = transport->args;
  absolute_time_t deadline = make_timeout_time_ms(timeout > 0 ? timeout : 0);

  while (true) {
    size_t received = 0;
    bool oversized = false;

    cyw43_arch_lwip_begin();
    if (context->receive_count > 0) {
      size_t head = context->receive_head;
      received = context->receive_lengths[head];
      if (received <= length) {
        memcpy(buffer, context->receive_buffers[head], received);
      } else {
        oversized = true;
        received = 0;
      }
      context->receive_head =
        (head + 1) % PICOW_UDP_RECEIVE_QUEUE_DEPTH;
      context->receive_count--;
    }
    cyw43_arch_lwip_end();

    if (oversized) {
      if (error_code != NULL) {
        *error_code = 1;
      }
      return 0;
    }
    if (received > 0) {
      return received;
    }
    if (timeout <= 0 || time_reached(deadline)) {
      return 0;
    }
    sleep_ms(1);
  }
}
