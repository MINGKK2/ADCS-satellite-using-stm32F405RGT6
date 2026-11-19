#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H
#include "telemetry_data.h"
#define TELEMETRY_FIRMWARE_ID "AXIS_FRAME_RP_UART_CFE10_V14"
/* Wire defaults must match the receiver mission configuration. This project
 * does not link cFE: it serializes a CCSDS v1 telemetry packet explicitly.
 * Header: 6-byte primary + 6-byte time, big endian. Payload: 10 float32.
 * Time is MCU uptime, not synchronized cFE mission epoch time.
 */
#ifndef TELEMETRY_SENSOR_MSG_ID
#define TELEMETRY_SENSOR_MSG_ID 0x0881U
#endif
#ifndef TELEMETRY_PAYLOAD_BIG_ENDIAN
#define TELEMETRY_PAYLOAD_BIG_ENDIAN 0
#endif
#define TELEMETRY_PERIOD_MS 50U
#define TELEMETRY_HEADER_BYTES 12U
#define TELEMETRY_PAYLOAD_BYTES 40U
#define TELEMETRY_PACKET_BYTES (TELEMETRY_HEADER_BYTES + TELEMETRY_PAYLOAD_BYTES)
#include <stddef.h>
#include <stdint.h>
void TelemetryProtocol_Init(void);
void TelemetryProtocol_Service(const TelemetryData *data);
uint8_t TelemetryProtocol_Event(const uint8_t *data,size_t length);
#endif
