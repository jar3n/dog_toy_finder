#include "wifi/wifi_log.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/cyw43_arch.h"

#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "config/wifi_config.h"

#ifndef WIFI_LOG_TARGET_IP
#define WIFI_LOG_TARGET_IP "255.255.255.255"
#endif

#ifndef WIFI_LOG_TARGET_PORT
#define WIFI_LOG_TARGET_PORT 5005
#endif

#ifndef WIFI_LOG_MAX_MESSAGE_LENGTH
#define WIFI_LOG_MAX_MESSAGE_LENGTH 512
#endif

static struct udp_pcb *log_pcb = NULL;
static ip_addr_t log_target;
static bool log_initialized = false;

/**
 * Send already-formatted log bytes to the configured UDP destination.
 *
 * Copies the message into a pbuf and sends it from the station interface.
 * No-op until wifi_log_init() has created the socket.
 */
static void send_datagram(const char *message, size_t length) {
  if (log_pcb == NULL || length == 0) {
    return;
  }

  cyw43_arch_lwip_begin();
  struct pbuf *packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t) length, PBUF_RAM);
  err_t result = packet == NULL
    ? ERR_MEM
    : pbuf_take(packet, message, (u16_t) length);
  if (result == ERR_OK) {
    result = udp_sendto_if(
      log_pcb,
      packet,
      &log_target,
      WIFI_LOG_TARGET_PORT,
      &cyw43_state.netif[CYW43_ITF_STA]);
  }
  if (packet != NULL) {
    pbuf_free(packet);
  }
  cyw43_arch_lwip_end();
}

/**
 * Write a message to USB serial and mirror it to UDP when available.
 *
 * The bytes are written verbatim; no newline or formatting is added.
 */
void wifi_log_write(const char *message, size_t length) {
  if (message == NULL || length == 0) {
    return;
  }

  fwrite(message, 1, length, stdout);
  fflush(stdout);
  send_datagram(message, length);
}

/**
 * Format a message into a fixed buffer and forward it to wifi_log_write().
 *
 * Output longer than WIFI_LOG_MAX_MESSAGE_LENGTH (including the terminator) is
 * truncated.
 */
void wifi_log_printf(const char *format, ...) {
  char message[WIFI_LOG_MAX_MESSAGE_LENGTH];

  va_list args;
  va_start(args, format);
  int formatted = vsnprintf(message, sizeof message, format, args);
  va_end(args);

  if (formatted <= 0) {
    return;
  }

  size_t length = (size_t) formatted;
  if (length >= sizeof message) {
    length = sizeof message - 1;
  }

  wifi_log_write(message, length);
}

/**
 * Create the UDP socket used to mirror logs.
 *
 * Resolves WIFI_LOG_TARGET_IP and allocates an IPv4 PCB. Called by wifi_init()
 * after the link is up. Repeated calls return the current socket state without
 * creating a second socket.
 */
bool wifi_log_init(void) {
  if (log_initialized) {
    return log_pcb != NULL;
  }
  log_initialized = true;

  if (!ipaddr_aton(WIFI_LOG_TARGET_IP, &log_target)) {
    wifi_log_printf("wifi: invalid log target IP: %s\n", WIFI_LOG_TARGET_IP);
    return false;
  }

  cyw43_arch_lwip_begin();
  log_pcb = udp_new_ip_type(IPADDR_TYPE_V4);
  cyw43_arch_lwip_end();

  if (log_pcb == NULL) {
    wifi_log_printf("wifi: could not create the UDP log socket\n");
    return false;
  }

  wifi_log_printf(
    "wifi: logs mirrored to udp://%s:%u\n",
    WIFI_LOG_TARGET_IP,
    (unsigned) WIFI_LOG_TARGET_PORT);
  return true;
}
