#ifndef DWT_TIME_H
#define DWT_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Monotonic microsecond timestamp from the DWT cycle counter (wrap-safe).
 *
 * The DWT cycle counter must already be enabled (done once in StartDefaultTask).
 * Shared by the IMU task (sample timestamps) and the ROS task (time sync), so it
 * lives here rather than in the CubeMX-managed freertos.c. Not reentrant: the
 * wrap-tracking state is shared by all callers (unchanged from the prior
 * static-inline version in freertos.c).
 */
uint64_t dwt_micros(void);

#ifdef __cplusplus
}
#endif

#endif /* DWT_TIME_H */
