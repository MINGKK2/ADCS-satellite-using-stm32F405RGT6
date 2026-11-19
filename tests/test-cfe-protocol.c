#include "telemetry_protocol.h"
#include "telemetry_uart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned sends;
static int reject_send;
static uint8_t captured[TELEMETRY_PACKET_BYTES];
int TelemetryUART_Send(const void *data, size_t size, int priority, unsigned stream)
{
  assert(data && size == 52U && !priority && stream == 0U);
  memcpy(captured, data, size);
  sends++;
  return !reject_send;
}
static uint16_t be16(const uint8_t *b) { return (uint16_t)((b[0] << 8) | b[1]); }
static float value(unsigned index)
{
  const uint8_t *b = captured + 12U + index * 4U;
  uint32_t bits = 0;
  for (unsigned i = 0; i < 4; ++i)
    bits |= (uint32_t)b[i] << ((TELEMETRY_PAYLOAD_BIG_ENDIAN ? 3U-i : i) * 8U);
  float f; memcpy(&f, &bits, 4); return f;
}
int main(void)
{
  TelemetryData s = {0};
  s.sv = 7; s.attitude_valid = 1; s.gps_fix = 1;
  s.bmp_centi_c = 2590; s.aht_milli_c = 26000; s.pressure_pa = 101325;
  s.angle[0] = 1000; s.angle[1] = -2000; s.angle[2] = 169000;
  s.current_ma10 = 12345; s.altitude_mm = 125500; s.voltage_mv = 12400;
  s.gps_lat_e7 = 210288330; s.gps_lon_e7 = 1058520000;
  TelemetryProtocol_Init();
  TelemetryProtocol_Service(NULL);
  s.uptime_ms = 49; TelemetryProtocol_Service(&s); assert(!sends);
  s.uptime_ms = 50; TelemetryProtocol_Service(&s); assert(sends == 1);
  assert(be16(captured) == TELEMETRY_SENSOR_MSG_ID);
  assert(be16(captured + 2) == 0xC000);
  assert(be16(captured + 4) == 45);
  assert(captured[6] == 0 && captured[7] == 0 && captured[8] == 0 && captured[9] == 0);
  assert(be16(captured + 10) == 3276);
  const float expected[10] = {25.95f,101325.0f,1.0f,-2.0f,169.0f,
      1.2345f,125.5f,21.028833f,105.852f,12.4f};
  for (unsigned i = 0; i < 10; ++i) assert(fabsf(value(i) - expected[i]) < 0.00001f);
  /* Binary representation of 1.0f at the Roll offset. */
  const uint8_t one_le[] = {0,0,0x80,0x3F}, one_be[] = {0x3F,0x80,0,0};
  assert(!memcmp(captured + 20, TELEMETRY_PAYLOAD_BIG_ENDIAN ? one_be : one_le, 4));
  s.uptime_ms = 99; TelemetryProtocol_Service(&s); assert(sends == 1);
  assert(!TelemetryProtocol_Event((const uint8_t *)"ACK CMD=STATUS\r\n",16));
  assert(!TelemetryProtocol_Event((const uint8_t *)"CAL START\r\n",11));
  assert(!TelemetryProtocol_Event(NULL,0)); assert(sends == 1);
  s.uptime_ms = 1250; s.current_ma10 = -12345; TelemetryProtocol_Service(&s);
  assert(be16(captured + 2) == 0xC001 && captured[9] == 1);
  assert(be16(captured + 10) == 16384 && value(5) < 0);
  s.uptime_ms += 50; s.sv = 3; TelemetryProtocol_Service(&s);
  assert(isnan(value(0))); assert(value(1) == 101325.0f); /* AHT alone missing */
  s.uptime_ms += 50; s.sv = 6; TelemetryProtocol_Service(&s);
  assert(isnan(value(0)) && isnan(value(1)) && isnan(value(6))); /* BMP missing */
  s.uptime_ms += 50; s.sv = 0; s.attitude_valid = 0; s.gps_fix = 0;
  TelemetryProtocol_Service(&s);
  for(unsigned i = 0; i < 10; ++i) assert(isnan(value(i)));
  s.uptime_ms += 50; s.gps_fix = 1; s.gps_lat_e7 = INT32_MAX;
  TelemetryProtocol_Service(&s); assert(isnan(value(7)) && isnan(value(8)));
  TelemetryProtocol_Init();
  for(unsigned i = 0; i < 16385U; ++i) {
    s.uptime_ms = 50U * (i + 1U); TelemetryProtocol_Service(&s);
    assert(be16(captured + 2) == (0xC000U | (i & 0x3FFFU)));
  }
  TelemetryProtocol_Init(); reject_send = 1;
  s.uptime_ms = 50; TelemetryProtocol_Service(&s);
  reject_send = 0; s.uptime_ms = 100; TelemetryProtocol_Service(&s);
  assert(be16(captured + 2) == 0xC000);
  TelemetryProtocol_Init();
  s.uptime_ms = UINT32_MAX - 31U; TelemetryProtocol_Service(&s);
  unsigned before = sends; s.uptime_ms += 50U; TelemetryProtocol_Service(&s);
  assert(sends == before + 1);
  puts("PASS: 52-byte header/payload, average temperature, units/order, timing, NaN, silent events, sequence/tick wrap, queue rejection");
  return 0;
}
