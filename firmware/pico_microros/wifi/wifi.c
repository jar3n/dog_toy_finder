#include "wifi/wifi.h"
#include "wifi/wifi_log.h"

#include <string.h>

#include "pico/cyw43_arch.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

#include "config/wifi_config.h"

#ifndef WIFI_AUTH
#define WIFI_AUTH CYW43_AUTH_WPA2_AES_PSK
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 15000
#endif

#ifndef WIFI_CONNECT_RETRIES
#define WIFI_CONNECT_RETRIES 3
#endif

#ifndef WIFI_CONNECT_RETRY_DELAY_MS
#define WIFI_CONNECT_RETRY_DELAY_MS 1000
#endif

// How often the connect wait services the poll hook and checks the link.
#ifndef WIFI_CONNECT_POLL_MS
#define WIFI_CONNECT_POLL_MS 50
#endif

static bool wifi_initialized = false;
static wifi_poll_hook_t poll_hook = NULL;

/**
 * Install the callback invoked during Wi-Fi waits; NULL removes it.
 */
void wifi_set_poll_hook(wifi_poll_hook_t hook) {
  poll_hook = hook;
}

/**
 * Invoke the optional poll hook once.
 *
 * The hook may be NULL, which is the normal case outside the boot sequence.
 */
static void wifi_poll(void) {
  if (poll_hook != NULL) {
    poll_hook();
  }
}

/**
 * Sleep for a number of milliseconds while servicing the poll hook.
 *
 * Breaks the wait into WIFI_CONNECT_POLL_MS chunks so a status LED keeps
 * blinking during connection retries and link waits.
 */
static void wifi_wait_ms(uint32_t delay_ms) {
  while (delay_ms > 0) {
    uint32_t step =
      delay_ms > WIFI_CONNECT_POLL_MS ? WIFI_CONNECT_POLL_MS : delay_ms;
    wifi_poll();
    sleep_ms(step);
    delay_ms -= step;
  }
}

/**
 * Start a Wi-Fi join and wait for the link, servicing the poll hook.
 *
 * Mirrors cyw43_arch_wifi_connect_timeout_ms() but replaces its internal
 * wait with wifi_wait_ms(), so the status LED can keep blinking while the
 * join is in progress. Retries when the access point disappears mid-join.
 *
 * @return PICO_OK when the link is up, PICO_ERROR_BADAUTH on bad credentials,
 *         PICO_ERROR_CONNECT_FAILED on a failed join, or PICO_ERROR_TIMEOUT
 *         when WIFI_CONNECT_TIMEOUT_MS elapses.
 */
static int wifi_connect_wait(void) {
  int status_code = cyw43_arch_wifi_connect_async(
    WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH);
  if (status_code != PICO_OK) {
    return status_code;
  }

  absolute_time_t deadline = make_timeout_time_ms(WIFI_CONNECT_TIMEOUT_MS);
  while (!time_reached(deadline)) {
    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

    if (link_status == CYW43_LINK_NONET) {
      // The network vanished mid-join; start the join again instead of
      // abandoning the attempt.
      status_code = cyw43_arch_wifi_connect_async(
        WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH);
      if (status_code != PICO_OK) {
        return status_code;
      }
      link_status = CYW43_LINK_JOIN;
    }

    if (link_status == CYW43_LINK_UP) {
      return PICO_OK;
    }
    if (link_status == CYW43_LINK_BADAUTH) {
      return PICO_ERROR_BADAUTH;
    }
    if (link_status == CYW43_LINK_FAIL) {
      return PICO_ERROR_CONNECT_FAILED;
    }

    wifi_wait_ms(WIFI_CONNECT_POLL_MS);
  }

  return PICO_ERROR_TIMEOUT;
}

/**
 * Convert a Pico Wi-Fi error code into a short human-readable string.
 */
static const char *connect_error_name(int error) {
  switch (error) {
    case PICO_OK:
      return "ok";
    case PICO_ERROR_TIMEOUT:
      return "timeout";
    case PICO_ERROR_BADAUTH:
      return "bad credentials";
    case PICO_ERROR_CONNECT_FAILED:
      return "connection failed";
    default:
      return "unknown error";
  }
}

/**
 * Check whether the station interface currently has an active link.
 *
 * Takes the lwIP lock while querying the CYW43 driver. Returns false before
 * wifi_init() has initialized the wireless chip.
 */
bool wifi_is_connected(void) {
  if (!wifi_initialized) {
    return false;
  }

  cyw43_arch_lwip_begin();
  bool connected =
    cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
  cyw43_arch_lwip_end();

  return connected;
}

/**
 * Return the station's current IPv4 address as a dotted string.
 *
 * Returns NULL before initialization, while the link is down, or when no
 * address has been assigned. The returned string is owned by lwIP and may be
 * overwritten by later address conversions.
 */
const char *wifi_ip_address(void) {
  if (!wifi_initialized) {
    return NULL;
  }

  const char *address_string = NULL;
  cyw43_arch_lwip_begin();
  if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP) {
    const ip4_addr_t *address =
      netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA]);
    if (!ip4_addr_isany(address)) {
      address_string = ip4addr_ntoa(address);
    }
  }
  cyw43_arch_lwip_end();

  return address_string;
}

/**
 * Initialize the CYW43 chip and join the configured access point.
 *
 * Initializes the driver once, enables station mode, and retries the join up
 * to WIFI_CONNECT_RETRIES times. Starts the UDP log mirror after a successful
 * join. Returns false when the credentials are still placeholders or every
 * attempt fails; the rest of the firmware can keep running either way.
 */
bool wifi_init(void) {
  if (!wifi_initialized) {
    // The status LED shares this chip and may have initialized it first.
    if (!cyw43_is_initialized(&cyw43_state)) {
      if (cyw43_arch_init() != PICO_OK) {
        wifi_log_printf("wifi: CYW43 initialization failed\n");
        return false;
      }
    }
    wifi_initialized = true;
    cyw43_arch_enable_sta_mode();
  }

  if (wifi_is_connected()) {
    wifi_log_init();
    return true;
  }

  if (strlen(WIFI_SSID) == 0 ||
    strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
    wifi_log_printf(
      "wifi: set WIFI_SSID and WIFI_PASSWORD in config/wifi_config.h\n");
    return false;
  }

  for (unsigned attempt = 1; attempt <= WIFI_CONNECT_RETRIES; ++attempt) {
    wifi_log_printf(
      "wifi: connecting to \"%s\" (attempt %u/%u)\n",
      WIFI_SSID,
      attempt,
      (unsigned) WIFI_CONNECT_RETRIES);

    int status = wifi_connect_wait();

    if (status == PICO_OK) {
      wifi_log_init();
      const char *ip = wifi_ip_address();
      wifi_log_printf("wifi: connected, ip=%s\n", ip == NULL ? "unknown" : ip);
      return true;
    }

    wifi_log_printf(
      "wifi: connection failed (%s)\n",
      connect_error_name(status));
    if (attempt < WIFI_CONNECT_RETRIES) {
      wifi_wait_ms(WIFI_CONNECT_RETRY_DELAY_MS);
    }
  }

  wifi_log_printf("wifi: could not connect to \"%s\"\n", WIFI_SSID);
  return false;
}
