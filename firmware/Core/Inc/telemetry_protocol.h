#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H
#include "telemetry_data.h"
#define TELEMETRY_FIRMWARE_ID "AXIS_FRAME_RP_UART_V13"
#include <stddef.h>
#include <stdint.h>
void TelemetryProtocol_Init(void);
void TelemetryProtocol_Service(const TelemetryData *data);
uint8_t TelemetryProtocol_Event(const uint8_t *data,size_t length);
#endif
