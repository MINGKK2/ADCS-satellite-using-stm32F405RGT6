#include "gps_receiver.h"

#include <string.h>

enum ReceiveState { SEARCH, NMEA, UBX_SYNC, UBX_HEADER, UBX_PAYLOAD, UBX_CKA, UBX_CKB };

static enum ReceiveState state;
static char sentence[128];
static uint8_t header[4], payload[92], checksum_a, checksum_b;
static uint16_t received, payload_length;
static uint32_t last_byte_ms, fix_ms;
static int32_t latitude, longitude;
static bool valid;

void GpsReceiver_Reset(void)
{
  state = SEARCH;
  valid = false;
}

bool GpsReceiver_Position(uint32_t now_ms, int32_t *lat_e7, int32_t *lon_e7)
{
  bool fresh = valid && now_ms - fix_ms < GPS_POSITION_TIMEOUT_MS;
  *lat_e7 = fresh ? latitude : INT32_MAX;
  *lon_e7 = fresh ? longitude : INT32_MAX;
  return fresh;
}

static void publish_position(int32_t lat, int32_t lon, uint32_t now_ms)
{
  valid = lat >= -900000000 && lat <= 900000000 &&
          lon >= -1800000000 && lon <= 1800000000;
  if (valid) {
    latitude = lat;
    longitude = lon;
    fix_ms = now_ms;
  }
}

static int hex_value(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/* NMEA uses ddmm.m... for latitude and dddmm.m... for longitude. */
static bool nmea_coordinate(const char *field, const char *hemisphere,
                            unsigned degree_digits, int32_t *out)
{
  unsigned degrees = 0, minutes = 0;
  uint32_t fraction = 0, scale = 1;
  for (unsigned i = 0; i < degree_digits + 2U; ++i) {
    if (*field < '0' || *field > '9') return false;
    unsigned digit = (unsigned)(*field++ - '0');
    if (i < degree_digits) degrees = degrees * 10U + digit;
    else minutes = minutes * 10U + digit;
  }
  if (*field++ != '.' || *field == '\0' || minutes >= 60U) return false;
  while (*field != '\0') {
    if (*field < '0' || *field > '9') return false;
    if (scale < 10000000U) {
      fraction = fraction * 10U + (uint32_t)(*field - '0');
      scale *= 10U;
    }
    ++field;
  }
  if (*hemisphere == '\0' || hemisphere[1] != '\0') return false;
  bool is_latitude = degree_digits == 2U;
  bool negative = *hemisphere == (is_latitude ? 'S' : 'W');
  if (!negative && *hemisphere != (is_latitude ? 'N' : 'E')) return false;
  uint32_t limit = is_latitude ? 90U : 180U;
  if (degrees > limit || (degrees == limit && (minutes || fraction))) return false;
  uint64_t denominator = 60ULL * scale;
  uint64_t numerator = ((uint64_t)minutes * scale + fraction) * 10000000ULL;
  int32_t magnitude = (int32_t)(degrees * 10000000U +
                                (numerator + denominator / 2U) / denominator);
  *out = negative ? -magnitude : magnitude;
  return true;
}

static void decode_nmea(uint32_t now_ms)
{
  char *star = strchr(sentence, '*');
  if (!star || strlen(star) != 3U) return;
  int high = hex_value(star[1]);
  int low = hex_value(star[2]);
  if (high < 0 || low < 0) return;
  uint8_t checksum = 0;
  for (const char *p = sentence; p < star; ++p) checksum ^= (uint8_t)*p;
  if (checksum != (uint8_t)(high * 16 + low)) return;
  *star = '\0';

  char *fields[7] = {sentence};
  unsigned count = 1;
  for (char *p = sentence; *p != '\0'; ++p) {
    if (*p == ',') {
      *p = '\0';
      if (count == 7U) break;
      fields[count++] = p + 1;
    }
  }
  if (count != 7U || strlen(fields[0]) != 5U) return;
  bool rmc = strcmp(fields[0] + 2, "RMC") == 0;
  bool gga = strcmp(fields[0] + 2, "GGA") == 0;
  if (!rmc && !gga) return;
  if ((rmc && strcmp(fields[2], "A") != 0) ||
      (gga && strcmp(fields[6], "1") && strcmp(fields[6], "2") &&
       strcmp(fields[6], "4") && strcmp(fields[6], "5"))) {
    valid = false;
    return;
  }

  unsigned index = rmc ? 3U : 2U;
  int32_t lat, lon;
  if (!nmea_coordinate(fields[index], fields[index + 1], 2U, &lat) ||
      !nmea_coordinate(fields[index + 2], fields[index + 3], 3U, &lon)) {
    valid = false;
    return;
  }
  publish_position(lat, lon, now_ms);
}

static int32_t le_i32(const uint8_t *p)
{
  uint32_t value = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
                   (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
  return value <= INT32_MAX ? (int32_t)value : -1 - (int32_t)(UINT32_MAX - value);
}

static void decode_ubx(uint32_t now_ms)
{
  if (header[0] != 1U || header[1] != 7U || payload_length != sizeof(payload)) return;
  /* UBX-NAV-PVT position and validity fields, u-blox M10 interface description. */
  uint8_t fix_type = payload[20];
  if (fix_type < 2U || fix_type > 4U || !(payload[21] & 1U) || (payload[78] & 1U)) {
    valid = false;
    return;
  }
  publish_position(le_i32(&payload[28]), le_i32(&payload[24]), now_ms);
}

static void ubx_checksum(uint8_t byte)
{
  checksum_a += byte;
  checksum_b += checksum_a;
}

void GpsReceiver_Feed(uint8_t byte, uint32_t now_ms)
{
  if (now_ms - last_byte_ms > 250U) state = SEARCH;
  last_byte_ms = now_ms;
  switch (state) {
  case NMEA:
    if (byte == '$') {
      received = 0;
    } else if (byte == '\r' || byte == '\n') {
      sentence[received] = '\0';
      decode_nmea(now_ms);
      state = SEARCH;
    } else if (byte >= 32U && byte <= 126U && received < sizeof(sentence) - 1U) {
      sentence[received++] = (char)byte;
    } else {
      state = byte == 0xB5U ? UBX_SYNC : SEARCH;
    }
    break;
  case UBX_SYNC:
    if (byte == 0x62U) {
      state = UBX_HEADER;
      checksum_a = checksum_b = 0;
      received = 0;
      break;
    }
    /* A failed sync byte can itself begin the next frame. */
    /* fall through */
  case SEARCH:
    if (byte == '$') {
      state = NMEA;
      received = 0;
    } else {
      state = byte == 0xB5U ? UBX_SYNC : SEARCH;
    }
    break;
  case UBX_HEADER:
    header[received++] = byte;
    ubx_checksum(byte);
    if (received == sizeof(header)) {
      payload_length = (uint16_t)header[2] | (uint16_t)header[3] << 8;
      received = 0;
      if (payload_length > 2048U) state = SEARCH;
      else state = payload_length ? UBX_PAYLOAD : UBX_CKA;
    }
    break;
  case UBX_PAYLOAD:
    if (received < sizeof(payload)) payload[received] = byte;
    ubx_checksum(byte);
    if (++received == payload_length) state = UBX_CKA;
    break;
  case UBX_CKA:
    state = byte == checksum_a ? UBX_CKB : SEARCH;
    break;
  case UBX_CKB:
    if (byte == checksum_b) decode_ubx(now_ms);
    state = SEARCH;
    break;
  }
}
