#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <ctype.h>
#include "telemetry_protocol.h"
#include "line_assembler.h"
static constexpr uint8_t CHANNEL=1;
static const uint8_t C3_MAC[6]={0xB0,0xA6,0x04,0x40,0xC2,0x74};
struct UsbLine {uint16_t length;char data[1024];};
QueueHandle_t rxQueue,commandQueue,usbQueue,controlQueue;
RadioLineAssembler assembler;
volatile bool radioBusy=false;
volatile uint32_t radioRx=0,radioPeer=0,radioBad=0,rxDrops=0,txAck=0,txFail=0;
uint32_t commandSequence=0,sendErrors=0,lineDrops=0,commandDrops=0,usbBytes=0;
bool statPending=false;
void onSend(const esp_now_send_info_t *,esp_now_send_status_t status){
  if(status==ESP_NOW_SEND_SUCCESS)txAck=txAck+1;else txFail=txFail+1;
  radioBusy=false;
}
void onReceive(const esp_now_recv_info_t *info,const uint8_t *bytes,int length){
  radioRx=radioRx+1;
  if(!info || memcmp(info->src_addr,C3_MAC,6))return;
  radioPeer=radioPeer+1;
  if(length<(int)TELEMETRY_HEADER_SIZE || length>(int)sizeof(TelemetryPacket)){radioBad=radioBad+1;return;}
  TelemetryPacket p{};memcpy(&p,bytes,(size_t)length);
  if(!packetIsValid(p,(size_t)length)){radioBad=radioBad+1;return;}
  if(p.type==TYPE_UART0_TO_GROUND && xQueueSend(rxQueue,&p,0)!=pdTRUE)rxDrops=rxDrops+1;
}
bool startRadio(){
  // Retained Arduino policy must precede the asynchronous STA_START event.
  WiFi.setSleep(false);WiFi.setAutoReconnect(false);WiFi.mode(WIFI_STA);WiFi.disconnect();
  if(esp_wifi_set_channel(CHANNEL,WIFI_SECOND_CHAN_NONE)!=ESP_OK || esp_wifi_set_ps(WIFI_PS_NONE)!=ESP_OK)return false;
  if(esp_now_init()!=ESP_OK)return false;
  esp_now_peer_info_t p{};memcpy(p.peer_addr,C3_MAC,6);p.channel=CHANNEL;p.ifidx=WIFI_IF_STA;p.encrypt=false;
  return esp_now_add_peer(&p)==ESP_OK && esp_now_register_send_cb(onSend)==ESP_OK && esp_now_register_recv_cb(onReceive)==ESP_OK;
}
bool queueLine(const char *data,size_t size,bool priority){
  if(!size || size>sizeof(UsbLine::data)){lineDrops++;return false;}
  UsbLine line{};line.length=(uint16_t)size;memcpy(line.data,data,size);
  if(xQueueSend(priority?controlQueue:usbQueue,&line,0)!=pdTRUE){lineDrops++;return false;}
  return true;
}
void handleLine(char *data,size_t length){
  char upper[200];for(size_t i=0;i<=length;i++)upper[i]=(char)toupper((unsigned char)data[i]);
  if(!strcmp(upper,"@LINKSTAT")){statPending=true;return;}
  TelemetryPacket p{};p.magic=TELEMETRY_MAGIC;p.version=TELEMETRY_VERSION;p.type=TYPE_GROUND_TO_UARTS;
  p.sequence=commandSequence++;p.length=(uint16_t)(length+1);memcpy(p.payload,data,length);p.payload[length]='\n';p.crc=packetCrc(p);
  if(!strcmp(upper,"ESTOP") || !strcmp(upper,"EMERGENCY_STOP")){
    // Explicit stop supersedes older commands that have not been sent yet.
    xQueueReset(commandQueue);
    if(xQueueSendToFront(commandQueue,&p,0)!=pdTRUE)commandDrops++;
  }else if(xQueueSend(commandQueue,&p,0)!=pdTRUE)commandDrops++;
}
void readCommands(){
  static char data[200];static size_t used=0;static bool discard=false;
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\r')continue;
    if(c=='\n'){if(!discard && used){data[used]=0;handleLine(data,used);}used=0;discard=false;continue;}
    if(discard)continue;
    if((uint8_t)c<32 || (uint8_t)c>126 || used>=sizeof(data)-2U){used=0;discard=true;commandDrops++;continue;}
    data[used++]=c;
  }
}
void sendCommands(){
  if(radioBusy)return;
  TelemetryPacket p{};if(xQueueReceive(commandQueue,&p,0)!=pdTRUE)return;
  radioBusy=true;
  if(esp_now_send(C3_MAC,reinterpret_cast<const uint8_t *>(&p),TELEMETRY_HEADER_SIZE+p.length)!=ESP_OK){radioBusy=false;sendErrors++;}
}
void readRadio(){
  TelemetryPacket p{};
  while(xQueueReceive(rxQueue,&p,0)==pdTRUE){
    assembler.packet(p.sequence,p.payload,p.length,[](const char *data,size_t size){
      bool priority=(size>=4&&!memcmp(data,"ACK ",4)) || (size>=5&&!memcmp(data,"NACK ",5)) || (size>=4&&!memcmp(data,"CAL ",4));
      queueLine(data,size,priority);
    });
  }
}
void reportStats(){
  if(!statPending || uxQueueSpacesAvailable(controlQueue)==0)return;
  uint8_t channel=0;wifi_second_chan_t secondary;wifi_ps_type_t ps;
  esp_wifi_get_channel(&channel,&secondary);esp_wifi_get_ps(&ps);
  char line[384];int n=snprintf(line,sizeof(line),
      "S3STAT FW=RADIO_ONLY_V1 MAC=%s CH=%u PS=%u RX=%lu MATCH=%lu BAD=%lu QDROP=%lu TXACK=%lu TXFAIL=%lu SENDERR=%lu GAPS=%lu DUP=%lu OLD=%lu LONG=%lu LINE_DROP=%lu CMD_DROP=%lu USB_BYTES=%lu\r\n",
      WiFi.macAddress().c_str(),channel,(unsigned)ps,(unsigned long)radioRx,(unsigned long)radioPeer,(unsigned long)radioBad,(unsigned long)rxDrops,
      (unsigned long)txAck,(unsigned long)txFail,(unsigned long)sendErrors,(unsigned long)assembler.gaps,(unsigned long)assembler.duplicates,
      (unsigned long)assembler.reordered,(unsigned long)assembler.overlong,(unsigned long)lineDrops,(unsigned long)commandDrops,(unsigned long)usbBytes);
  if(n>0 && (size_t)n<sizeof(line) && queueLine(line,(size_t)n,true))statPending=false;
}
void writeUsb(){
  static UsbLine current{};static size_t offset=0;
  if(offset==current.length){
    if(xQueueReceive(controlQueue,&current,0)!=pdTRUE && xQueueReceive(usbQueue,&current,0)!=pdTRUE)return;
    offset=0;
  }
  int room=Serial.availableForWrite();if(room<=0)return;
  size_t remaining=current.length-offset,n=remaining<(size_t)room?remaining:(size_t)room;
  size_t sent=Serial.write(reinterpret_cast<const uint8_t *>(current.data)+offset,n);offset+=sent;usbBytes+=(uint32_t)sent;
}
void setup(){
  Serial.setRxBufferSize(4096);Serial.setTxBufferSize(4096);Serial.begin(115200);
  esp_log_level_set("*",ESP_LOG_NONE);
  rxQueue=xQueueCreate(32,sizeof(TelemetryPacket));commandQueue=xQueueCreate(16,sizeof(TelemetryPacket));
  usbQueue=xQueueCreate(16,sizeof(UsbLine));controlQueue=xQueueCreate(8,sizeof(UsbLine));
  if(!rxQueue || !commandQueue || !usbQueue || !controlQueue || !startRadio()){
    Serial.println("FATAL: telemetry radio initialization failed");while(true)delay(1000);
  }
}
void loop(){readCommands();sendCommands();readRadio();reportStats();writeUsb();delay(1);}
