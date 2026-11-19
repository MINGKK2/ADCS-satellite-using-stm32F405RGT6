#include "telemetry_protocol.h"
#include "telemetry_uart.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24
               && FLT_MAX_EXP == 128, "IEEE-754 float32 required");
_Static_assert((TELEMETRY_SENSOR_MSG_ID & 0xF800U) == 0x0800U,
               "CCSDS v1 telemetry MID must include a secondary header");
_Static_assert(TELEMETRY_PAYLOAD_BIG_ENDIAN == 0 || TELEMETRY_PAYLOAD_BIG_ENDIAN == 1,
               "Select one payload byte order");

static uint32_t last_tick;
static uint16_t sequence;
static uint8_t packet[TELEMETRY_PACKET_BYTES];

static void put_be16(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)(value >> 8);
  out[1] = (uint8_t)value;
}

static void put_be32(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)(value >> 24);
  out[1] = (uint8_t)(value >> 16);
  out[2] = (uint8_t)(value >> 8);
  out[3] = (uint8_t)value;
}

static void put_float32(uint8_t *out, float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  for (unsigned i = 0; i < 4U; ++i)
  {
    unsigned shift = (TELEMETRY_PAYLOAD_BIG_ENDIAN ? 3U - i : i) * 8U;
    out[i] = (uint8_t)(bits >> shift);
  }
}

void TelemetryProtocol_Init(void)
{
  last_tick = 0U;
  sequence = 0U;
}

void TelemetryProtocol_Service(const TelemetryData *s)
{
  if (!s || (uint32_t)(s->uptime_ms - last_tick) < TELEMETRY_PERIOD_MS) return;
  last_tick = s->uptime_ms;

  const int bmp_valid = (s->sv & 1U) != 0U;
  const int ina_valid = (s->sv & 2U) != 0U;
  const int aht_valid = (s->sv & 4U) != 0U;
  const int gps_valid = s->gps_fix && s->gps_lat_e7 != INT32_MAX
      && s->gps_lon_e7 != INT32_MAX
      && s->gps_lat_e7 >= -900000000 && s->gps_lat_e7 <= 900000000
      && s->gps_lon_e7 >= -1800000000 && s->gps_lon_e7 <= 1800000000;

  /* Convert both temperatures to the same unit before averaging. Both sensors
   * must be fresh/valid. Never silently replace a missing sensor with zero.
   */
  const float mean_temperature = bmp_valid && aht_valid
      ? (float)(((double)s->bmp_centi_c / 100.0
                 + (double)s->aht_milli_c / 1000.0) / 2.0)
      : NAN;

  /* Mission_SensorTlm payload order. The names of slots 5 and 9 come from
   * the receiver: Voltage intentionally carries current; RemainingEnergy
   * intentionally carries bus voltage. Missing/stale measurements are NaN.
   */
  const float values[10] = {
    mean_temperature,                                                /* C */
    bmp_valid ? (float)s->pressure_pa : NAN,                          /* Pa */
    s->attitude_valid ? (float)s->angle[0] / 1000.0f : NAN,            /* deg */
    s->attitude_valid ? (float)s->angle[1] / 1000.0f : NAN,            /* deg */
    s->attitude_valid ? (float)s->angle[2] / 1000.0f : NAN,            /* deg */
    ina_valid ? (float)s->current_ma10 / 10000.0f : NAN,              /* A */
    bmp_valid ? (float)s->altitude_mm / 1000.0f : NAN,                 /* m */
    gps_valid ? (float)((double)s->gps_lat_e7 / 10000000.0) : NAN,     /* deg */
    gps_valid ? (float)((double)s->gps_lon_e7 / 10000000.0) : NAN,     /* deg */
    ina_valid ? (float)s->voltage_mv / 1000.0f : NAN                  /* V */
  };

  put_be16(packet, (uint16_t)TELEMETRY_SENSOR_MSG_ID);
  put_be16(packet + 2U, (uint16_t)(0xC000U | sequence)); /* unsegmented */
  put_be16(packet + 4U, (uint16_t)(TELEMETRY_PACKET_BYTES - 7U));
  put_be32(packet + 6U, s->uptime_ms / 1000U);
  put_be16(packet + 10U,
      (uint16_t)(((s->uptime_ms % 1000U) * 65536U) / 1000U));
  for (unsigned i = 0; i < 10U; ++i)
    put_float32(packet + TELEMETRY_HEADER_BYTES + 4U * i, values[i]);

  /* The UART layer copies all 52 bytes into its own DMA queue storage. */
  if (TelemetryUART_Send(packet, sizeof(packet), 0, 0U))
    sequence = (uint16_t)((sequence + 1U) & 0x3FFFU);
}

uint8_t TelemetryProtocol_Event(const uint8_t *data, size_t length)
{
  /* RX commands and ESTOP remain functional. Suppress ALL outbound text:
   * ACK/NACK, calibration messages, banners and legacy diagnostic records.
   */
  (void)data;
  (void)length;
  return 0U;
}
