#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "telemetry_protocol.h"

// COM34 uses the C3 USB Serial/JTAG peripheral and does not consume UART0/1.
static constexpr int UART0_RX_PIN = 20;
static constexpr int UART0_TX_PIN = 21;
static constexpr int UART1_RX_PIN = 4;
static constexpr int UART1_TX_PIN = 5;
static constexpr uint32_t UART_BAUD = 115200;
static constexpr uint8_t ESPNOW_CHANNEL = 1;
static constexpr uint32_t UART_BATCH_TIMEOUT_US = 3000;
static constexpr bool USB_VERBOSE=false;

// ESP32-S3 Wi-Fi STA MAC (paired receiver): dc:b4:d9:0b:64:3c
static const uint8_t S3_ADDRESS[6] = {
    0xDC, 0xB4, 0xD9, 0x0B, 0x64, 0x3C};

HardwareSerial TelemetryUart0(0);
HardwareSerial TelemetryUart1(1);
QueueHandle_t downlinkQueue = nullptr;
volatile bool emergencyPending=false;
uint32_t uart0Sequence = 0;
uint32_t uart1Sequence = 0;
uint32_t usbSequence = 0;
uint32_t uart0ByteCount = 0;
uint32_t uart1ByteCount = 0;
uint32_t uart0PacketCount = 0;
uint32_t uart1PacketCount = 0;
uint32_t sendErrorCount = 0;
volatile uint32_t radioRx=0,radioPeerRx=0,radioBadFrame=0,radioTxAck=0,radioTxNoAck=0;
volatile bool radioTxBusy=false;
volatile esp_now_send_status_t radioLastStatus=ESP_NOW_SEND_FAIL;
volatile int8_t lastRssi=-127;
void onEspNowSend(const esp_now_send_info_t *,esp_now_send_status_t status){
  if(status==ESP_NOW_SEND_SUCCESS)radioTxAck=radioTxAck+1;
  else radioTxNoAck=radioTxNoAck+1;
  radioLastStatus=status;radioTxBusy=false;
}
static esp_err_t radioSend(const uint8_t *address,const uint8_t *data,size_t size){
  if(radioTxBusy)return ESP_ERR_ESPNOW_NO_MEM;
  radioTxBusy=true;radioLastStatus=ESP_NOW_SEND_FAIL;
  esp_err_t status=esp_now_send(address,data,size);
  if(status!=ESP_OK){radioTxBusy=false;sendErrorCount++;}
  return status;
}

struct ReceivedPacket {
  size_t size;
  TelemetryPacket packet;
};

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  radioRx=radioRx+1;
  if (!info || memcmp(info->src_addr,S3_ADDRESS,6))return;
  radioPeerRx=radioPeerRx+1;
  if(info->rx_ctrl)lastRssi=info->rx_ctrl->rssi;
  if (length < 0 || static_cast<size_t>(length) > sizeof(TelemetryPacket)) {
    return;
  }

  ReceivedPacket item{};
  item.size = static_cast<size_t>(length);
  memcpy(&item.packet, data, item.size);
  if(!packetIsValid(item.packet,item.size)){radioBadFrame=radioBadFrame+1;return;}
  if (packetIsValid(item.packet, item.size) &&
      item.packet.type == TYPE_GROUND_TO_UARTS) {
    if ((item.packet.length==6 && !memcmp(item.packet.payload,"ESTOP\n",6))
        || (item.packet.length==15 && !memcmp(item.packet.payload,"EMERGENCY_STOP\n",15))) {
      xQueueReset(downlinkQueue);emergencyPending=true;
    } else xQueueSend(downlinkQueue,&item,0);
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
  // Set Arduino's retained policy before STA_START can reapply it.
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  if(esp_wifi_set_ps(WIFI_PS_NONE)!=ESP_OK)return false;
  // The driver is started after set_channel; Arduino's async STA.started()
  // flag may still lag here. IDF uses quarter-dBm: 34 = 8.5 dBm.
  if(esp_wifi_set_max_tx_power(34)!=ESP_OK)return false;
  if (!addS3Peer()) {
    return false;
  }
  if(esp_now_register_send_cb(onEspNowSend)!=ESP_OK)return false;
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

  if(USB_VERBOSE){Serial.write(packet.payload, packet.length);Serial.println();}
  packet.crc = packetCrc(packet);
  byteCount += packet.length;
  packetCount++;
  (void)radioSend(S3_ADDRESS,reinterpret_cast<uint8_t *>(&packet),TELEMETRY_HEADER_SIZE+packet.length);
}

void writeCommandsToUarts() {
  if(emergencyPending){
    emergencyPending=false;
    const uint8_t stop[]="\r\nESTOP\r\n";
    TelemetryUart0.write(stop,sizeof(stop)-1);TelemetryUart1.write(stop,sizeof(stop)-1);
  }
  ReceivedPacket item{};
  while (xQueueReceive(downlinkQueue, &item, 0) == pdTRUE) {
    if(USB_VERBOSE){Serial.print("[S3 -> C3 UART0+UART1] ");Serial.write(item.packet.payload, item.packet.length);Serial.println();}
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
  if(USB_VERBOSE){Serial.print("[C3 USB -> UART0+UART1+S3] ");Serial.write(packet.payload, packet.length);Serial.println();}
}

void setup() {
  Serial.begin(115200); // Native USB debug; enable "USB CDC On Boot".
  Serial.setTxTimeoutMs(0); // Debug USB must not stall UART/radio when closed.
  // UART0 is the STM32 protocol wire, including binary ROM mode. SDK error
  // logging must not inject text into it; explicit diagnostics use USB below.
  esp_log_level_set("*",ESP_LOG_NONE);
  TelemetryUart0.setRxBufferSize(8192);
  TelemetryUart1.setRxBufferSize(1024);
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
  Serial.println("ESP32-C3 telemetry radio V1 ready");
}

void loop() {
  writeCommandsToUarts();
  static uint32_t diagTick=0;
  static char diag[384];static size_t diagSize=0,diagOffset=0;
  if(!Serial){diagSize=0;diagOffset=0;}
  if(Serial && diagOffset==diagSize && millis()-diagTick>=2000){
    diagTick=millis();uint8_t channel=0;wifi_second_chan_t second;wifi_ps_type_t ps;int8_t power=0;esp_wifi_get_channel(&channel,&second);esp_wifi_get_ps(&ps);esp_wifi_get_max_tx_power(&power);
    int n=snprintf(diag,sizeof(diag),"C3STAT FW=RADIO_ONLY_V1 MAC=%s CH=%u PS=%u PWR_qdbm=%d RSSI=%d RX=%lu PEER=%lu BAD=%lu TXACK=%lu TXFAIL=%lu SENDERR=%lu BUSY=%u U0=%lu\r\n",WiFi.macAddress().c_str(),channel,(unsigned)ps,(int)power,(int)lastRssi,(unsigned long)radioRx,(unsigned long)radioPeerRx,(unsigned long)radioBadFrame,(unsigned long)radioTxAck,(unsigned long)radioTxNoAck,(unsigned long)sendErrorCount,radioTxBusy,(unsigned long)uart0ByteCount);
    diagOffset=0;diagSize=(n>0 && (size_t)n<sizeof(diag))?(size_t)n:0;
  }
  // Native USB can have far less than a full record free. Send bounded chunks;
  // a closed or unread debug port must never gate UART0/radio forwarding.
  if(diagOffset<diagSize){
    int room=Serial.availableForWrite();
    if(room>0){size_t n=diagSize-diagOffset;if(n>(size_t)room)n=(size_t)room;
      diagOffset+=Serial.write((const uint8_t *)diag+diagOffset,n);}
  }
  if (!radioTxBusy && TelemetryUart0.available()) {
    if(USB_VERBOSE)Serial.print("[C3 UART0 -> S3] ");
    sendUartBatch(TelemetryUart0, TYPE_UART0_TO_GROUND,
                  uart0Sequence, uart0ByteCount, uart0PacketCount);
  }
  // Only UART0 is a telemetry source; UART1 is not forwarded over radio.
  forwardUsbInput();
  writeCommandsToUarts();
  delay(1);
}
