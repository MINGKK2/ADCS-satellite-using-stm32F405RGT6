#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "telemetry_protocol.h"
#include "update_wire.h"

static constexpr uint8_t ESPNOW_CHANNEL = 1;
// Only STM32 on C3 UART0 (GPIO20 RX / GPIO21 TX) is the telemetry source.
static constexpr uint8_t PRIMARY_TELEMETRY_TYPE = TYPE_UART0_TO_GROUND;
static constexpr uint32_t USB_BATCH_TIMEOUT_US = 3000;
// ESP32-C3 Wi-Fi STA MAC (paired bridge): b0:a6:04:40:c2:74
static const uint8_t C3_ADDRESS[6] = {
    0xB0, 0xA6, 0x04, 0x40, 0xC2, 0x74};

QueueHandle_t telemetryQueue = nullptr;
uint32_t commandSequence = 0;

struct ReceivedPacket {
  size_t size;
  TelemetryPacket packet;
};

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  if(!info || memcmp(info->src_addr,C3_ADDRESS,6))return;
  if (length < 0 || static_cast<size_t>(length) > sizeof(TelemetryPacket)) {
    return;
  }

  ReceivedPacket item{};
  item.size = static_cast<size_t>(length);
  memcpy(&item.packet, data, item.size);
  if (packetIsValid(item.packet, item.size) &&
      (item.packet.type == PRIMARY_TELEMETRY_TYPE || item.packet.type == TYPE_UPDATE_RESPONSE)) {
    xQueueSend(telemetryQueue, &item, 0);
  }
}

bool startRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, C3_ADDRESS, sizeof(C3_ADDRESS));
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    return false;
  }
  return esp_now_register_recv_cb(onEspNowReceive) == ESP_OK;
}

void writeTelemetryToUsb() {
  ReceivedPacket item{};
  while (xQueueReceive(telemetryQueue, &item, 0) == pdTRUE) {
    // Transparent output for Web Serial/attitude-viewer: never add labels,
    // delimiters or debug text to the STM32 byte stream.
    if(item.packet.type==TYPE_UPDATE_RESPONSE){
      if(item.packet.length<6)continue;
      const uint8_t *v=item.packet.payload;
      Serial.printf("%s %lu %lu %u %u ",v[1]==FW_PROGRESS?"@FWPROGRESS":"@FWACK",(unsigned long)wire32(v+2),(unsigned long)item.packet.sequence,v[0],v[1]);
      for(size_t i=6;i<item.packet.length;i++)Serial.printf("%02X",v[i]);
      Serial.print("\r\n");
    }else Serial.write(item.packet.payload, item.packet.length);
  }
}

static int hexNibble(char c){if(c>='0'&&c<='9')return c-'0';if(c>='A'&&c<='F')return c-'A'+10;if(c>='a'&&c<='f')return c-'a'+10;return -1;}
static void hostLine(char *line,size_t length){
  TelemetryPacket packet{};packet.magic=TELEMETRY_MAGIC;packet.version=TELEMETRY_VERSION;
  if(!strncmp(line,"@FW ",4)){
    unsigned long session=0,seq=0;unsigned op=0;int consumed=0;
    if(sscanf(line,"@FW %lu %lu %u %n",&session,&seq,&op,&consumed)!=3 || !session || !seq || op>255 || consumed<=0){Serial.println("@FWERR HOST_HEADER");return;}
    const char *hex=line+consumed;size_t digits=strlen(hex);
    if(digits==1 && hex[0]=='-')digits=0;
    if((digits&1U) || digits/2>TELEMETRY_MAX_PAYLOAD-5){Serial.println("@FWERR HOST_LENGTH");return;}
    packet.type=TYPE_UPDATE_REQUEST;packet.sequence=(uint32_t)seq;packet.payload[0]=(uint8_t)op;put32(packet.payload+1,(uint32_t)session);
    packet.length=(uint16_t)(5+digits/2);
    for(size_t i=0;i<digits/2;i++){int hi=hexNibble(hex[2*i]),lo=hexNibble(hex[2*i+1]);if(hi<0||lo<0){Serial.println("@FWERR HOST_HEX");return;}packet.payload[5+i]=(uint8_t)((hi<<4)|lo);}
  }else{
    if(!length || length+1>TELEMETRY_MAX_PAYLOAD)return;
    packet.type=TYPE_GROUND_TO_UARTS;packet.sequence=commandSequence++;packet.length=(uint16_t)(length+1);
    memcpy(packet.payload,line,length);packet.payload[length]='\n';
  }
  packet.crc=packetCrc(packet);
  esp_now_send(C3_ADDRESS,reinterpret_cast<uint8_t *>(&packet),TELEMETRY_HEADER_SIZE+packet.length);
}
void sendUsbCommands(){
  static char line[512];static size_t length=0;static bool discard=false;
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\r')continue;
    if(c=='\n'){if(!discard && length){line[length]=0;hostLine(line,length);}length=0;discard=false;continue;}
    if(discard)continue;
    if((uint8_t)c<32 || (uint8_t)c>126 || length>=sizeof(line)-1){discard=true;length=0;continue;}
    line[length++]=c;
  }
}

void setup() {
  Serial.begin(115200);
  telemetryQueue = xQueueCreate(16, sizeof(ReceivedPacket));
  if (telemetryQueue == nullptr || !startRadio()) {
    Serial.println("FATAL: telemetry radio initialization failed");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  writeTelemetryToUsb();
  sendUsbCommands();
  delay(1);
}
