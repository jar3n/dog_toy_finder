#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback invoked periodically while wifi_init() waits for the link.
 *
 * Use it to keep a status indication alive during the potentially long Wi-Fi
 * association (the built-in LED uses this). Pass NULL to remove the callback.
 */
typedef void (*wifi_poll_hook_t)(void);

/** Install the callback invoked during Wi-Fi waits; NULL removes it. */
void wifi_set_poll_hook(wifi_poll_hook_t hook);

/**
 * Initialize the CYW43 Wi-Fi chip and join the configured network.
 *
 * Blocks while connecting, up to the timeout and retry settings in
 * config/wifi_config.h. Subsequent calls retry the connection if the link is
 * down.
 *
 * @return true when the station has an IP address, false otherwise.
 */
bool wifi_init(void);

/** @return true when the station link is up. */
bool wifi_is_connected(void);

/**
 * @return the station IPv4 address as a string, or NULL when not connected.
 *         The returned string is owned by lwIP and is overwritten by later
 *         address conversions.
 */
const char *wifi_ip_address(void);

#ifdef __cplusplus
}
#endif
