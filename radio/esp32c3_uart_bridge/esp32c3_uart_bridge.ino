#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/gpio.h>

#include "telemetry_protocol.h"

// COM34 uses the C3 USB Serial/JTAG peripheral and does not consume UART0/1.
static constexpr int UART0_RX_PIN = 20;
static constexpr int UART0_TX_PIN = 21;
static constexpr int UART1_RX_PIN = 4;
static constexpr int UART1_TX_PIN = 5;
static constexpr uint32_t UART_BAUD = 115200;
static constexpr uint8_t ESPNOW_CHANNEL = 1;
static constexpr uint32_t UART_BATCH_TIMEOUT_US = 3000;

// ESP32-S3 Wi-Fi STA MAC (paired receiver): dc:b4:d9:0b:64:3c
static const uint8_t S3_ADDRESS[6] = {
    0xDC, 0xB4, 0xD9, 0x0B, 0x64, 0x3C};

HardwareSerial TelemetryUart0(0);
HardwareSerial TelemetryUart1(1);
#include "firmware_updater.h"
FirmwareUpdater updater(TelemetryUart0,S3_ADDRESS);
QueueHandle_t downlinkQueue = nullptr;
uint32_t uart0Sequence = 0;
uint32_t uart1Sequence = 0;
uint32_t usbSequence = 0;
uint32_t uart0ByteCount = 0;
uint32_t uart1ByteCount = 0;
uint32_t uart0PacketCount = 0;
uint32_t uart1PacketCount = 0;
uint32_t sendErrorCount = 0;

struct ReceivedPacket {
  size_t size;
  TelemetryPacket packet;
};

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  if (!info || memcmp(info->src_addr,S3_ADDRESS,6))return;
  if (length < 0 || static_cast<size_t>(length) > sizeof(TelemetryPacket)) {
    return;
  }

  ReceivedPacket item{};
  item.size = static_cast<size_t>(length);
  memcpy(&item.packet, data, item.size);
  if (packetIsValid(item.packet, item.size) &&
      (item.packet.type == TYPE_GROUND_TO_UARTS || item.packet.type == TYPE_UPDATE_REQUEST)) {
    xQueueSend(downlinkQueue, &item, 0);
  }
}

bool addS3Peer() {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, S3_ADDRESS, sizeof(S3_ADDRESS));
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

bool startRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  if (!addS3Peer()) {
    return false;
  }
  return esp_now_register_recv_cb(onEspNowReceive) == ESP_OK;
}

void sendUartBatch(HardwareSerial &port, uint8_t packetType,
                   uint32_t &sequence, uint32_t &byteCount,
                   uint32_t &packetCount) {
  if (!port.available()) {
    return;
  }

  TelemetryPacket packet{};
  packet.magic = TELEMETRY_MAGIC;
  packet.version = TELEMETRY_VERSION;
  packet.type = packetType;
  packet.sequence = sequence++;

  const uint32_t deadline = micros() + UART_BATCH_TIMEOUT_US;
  while (packet.length < TELEMETRY_MAX_PAYLOAD) {
    while (port.available() && packet.length < TELEMETRY_MAX_PAYLOAD) {
      packet.payload[packet.length++] = static_cast<uint8_t>(port.read());
    }
    if (static_cast<int32_t>(micros() - deadline) >= 0) {
      break;
    }
    delayMicroseconds(100);
  }

  Serial.write(packet.payload, packet.length);
  Serial.println();
  packet.crc = packetCrc(packet);
  byteCount += packet.length;
  packetCount++;
  if (esp_now_send(S3_ADDRESS, reinterpret_cast<uint8_t *>(&packet),
                   TELEMETRY_HEADER_SIZE + packet.length) != ESP_OK) {
    sendErrorCount++;
  }
}

void writeCommandsToUarts() {
  ReceivedPacket item{};
  while (xQueueReceive(downlinkQueue, &item, 0) == pdTRUE) {
    if(item.packet.type==TYPE_UPDATE_REQUEST){updater.handle(item.packet);continue;}
    if(updater.busy())continue;
    Serial.print("[S3 -> C3 UART0+UART1] ");
    Serial.write(item.packet.payload, item.packet.length);
    Serial.println();
    // Ground commands are mirrored to both physical UART ports.
    TelemetryUart0.write(item.packet.payload, item.packet.length);
    TelemetryUart1.write(item.packet.payload, item.packet.length);
  }
}

void forwardUsbInput() {
  if (!Serial.available()) {
    return;
  }

  TelemetryPacket packet{};
  packet.magic = TELEMETRY_MAGIC;
  packet.version = TELEMETRY_VERSION;
  packet.type = TYPE_C3_USB_TO_GROUND;
  packet.sequence = usbSequence++;

  const uint32_t deadline = micros() + UART_BATCH_TIMEOUT_US;
  while (packet.length < TELEMETRY_MAX_PAYLOAD) {
    while (Serial.available() && packet.length < TELEMETRY_MAX_PAYLOAD) {
      packet.payload[packet.length++] = static_cast<uint8_t>(Serial.read());
    }
    if (static_cast<int32_t>(micros() - deadline) >= 0) {
      break;
    }
    delayMicroseconds(100);
  }

  // A message typed in the C3 Arduino Serial Monitor goes everywhere.
  TelemetryUart0.write(packet.payload, packet.length);
  TelemetryUart1.write(packet.payload, packet.length);
  packet.crc = packetCrc(packet);
  esp_now_send(S3_ADDRESS, reinterpret_cast<uint8_t *>(&packet),
               TELEMETRY_HEADER_SIZE + packet.length);
  Serial.print("[C3 USB -> UART0+UART1+S3] ");
  Serial.write(packet.payload, packet.length);
  Serial.println();
}

void setup() {
  Serial.begin(115200); // Native USB debug; enable "USB CDC On Boot".
  TelemetryUart0.begin(UART_BAUD, SERIAL_8N1, UART0_RX_PIN, UART0_TX_PIN);
  TelemetryUart1.begin(UART_BAUD, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  // UART idle is HIGH. Prevent random bytes when a source is unplugged.
  gpio_set_pull_mode(static_cast<gpio_num_t>(UART0_RX_PIN), GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(static_cast<gpio_num_t>(UART1_RX_PIN), GPIO_PULLUP_ONLY);
  downlinkQueue = xQueueCreate(16, sizeof(ReceivedPacket));

  if (downlinkQueue == nullptr || !startRadio()) {
    Serial.println("FATAL: telemetry radio initialization failed");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("ESP32-C3 telemetry + STM32 wireless updater V1 ready");
}

void loop() {
  if (!updater.busy() && TelemetryUart0.available()) {
    Serial.print("[C3 UART0 -> S3] ");
    sendUartBatch(TelemetryUart0, TYPE_UART0_TO_GROUND,
                  uart0Sequence, uart0ByteCount, uart0PacketCount);
  }
  if (!updater.busy() && TelemetryUart1.available()) {
    Serial.print("[C3 UART1 -> S3] ");
    sendUartBatch(TelemetryUart1, TYPE_UART1_TO_GROUND,
                  uart1Sequence, uart1ByteCount, uart1PacketCount);
  }
  if(!updater.busy())forwardUsbInput();
  writeCommandsToUarts();
  delay(1);
}
