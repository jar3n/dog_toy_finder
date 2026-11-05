#include <stdint.h>
#include <time.h>

#include "pico/time.h"

/**
 * POSIX clock shim used by micro-ROS and the C library.
 *
 * Reports time since boot; the requested clock ID is ignored because the Pico
 * has no battery-backed wall clock. micro-ROS later adds the agent time offset
 * after rmw_uros_sync_session().
 */
int clock_gettime(clockid_t clock_id, struct timespec *time_value) {
  (void) clock_id;

  uint64_t microseconds = time_us_64();
  time_value->tv_sec = (time_t) (microseconds / 1000000ULL);
  time_value->tv_nsec = (long) ((microseconds % 1000000ULL) * 1000ULL);
  return 0;
}