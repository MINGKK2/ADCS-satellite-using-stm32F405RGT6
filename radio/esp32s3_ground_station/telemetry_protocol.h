#pragma once

#include <Arduino.h>
#include <stddef.h>

static constexpr uint16_t TELEMETRY_MAGIC = 0x4354;
static constexpr uint8_t TELEMETRY_VERSION = 1;
static constexpr size_t TELEMETRY_MAX_PAYLOAD = 200;

enum TelemetryType : uint8_t {
  TYPE_UART0_TO_GROUND = 1,
  TYPE_GROUND_TO_UARTS = 2,
  TYPE_UART1_TO_GROUND = 3,
  TYPE_C3_USB_TO_GROUND = 4,
  TYPE_DIAGNOSTIC = 0x7F,
};

struct __attribute__((packed)) TelemetryPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint32_t sequence;
  uint16_t length;
  uint16_t crc;
  uint8_t payload[TELEMETRY_MAX_PAYLOAD];
};

static constexpr size_t TELEMETRY_HEADER_SIZE = offsetof(TelemetryPacket, payload);

inline uint16_t crc16Update(uint16_t crc, const uint8_t *data, size_t length) {
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline uint16_t packetCrc(const TelemetryPacket &packet) {
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, reinterpret_cast<const uint8_t *>(&packet),
                    offsetof(TelemetryPacket, crc));
  return crc16Update(crc, packet.payload, packet.length);
}

inline bool packetIsValid(const TelemetryPacket &packet, size_t receivedLength) {
  if (receivedLength < TELEMETRY_HEADER_SIZE ||
      packet.magic != TELEMETRY_MAGIC ||
      packet.version != TELEMETRY_VERSION ||
      packet.length > TELEMETRY_MAX_PAYLOAD ||
      receivedLength != TELEMETRY_HEADER_SIZE + packet.length) {
    return false;
  }
  return packet.crc == packetCrc(packet);
}
