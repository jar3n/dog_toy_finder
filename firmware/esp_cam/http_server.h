// HTTP server exposing the stereo camera endpoints.
#pragma once

#include <stdint.h>

// Shared capture counters, updated by the handlers.
extern uint32_t photos_served;
extern uint32_t frame_seq;
extern uint32_t last_capture_ms;

// Start the HTTP server and register all endpoints.
void start_http_server();
