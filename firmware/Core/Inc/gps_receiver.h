#ifndef GPS_RECEIVER_H
#define GPS_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#define GPS_POSITION_TIMEOUT_MS 3000U

/* Single receiver. All decoder calls run in the application context. */
void GpsReceiver_Reset(void);
void GpsReceiver_Feed(uint8_t byte, uint32_t now_ms);
bool GpsReceiver_Position(uint32_t now_ms, int32_t *lat_e7, int32_t *lon_e7);

#endif
