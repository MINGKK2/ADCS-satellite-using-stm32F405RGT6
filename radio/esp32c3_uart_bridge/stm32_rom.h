#pragma once
#include <Arduino.h>
#include "update_wire.h"
// ST AN3155. No option-byte, unprotect or mass-erase commands are exposed.
class Stm32Rom {
  HardwareSerial &port;
  bool ack(uint32_t timeout=1500){uint8_t b;if(!receive(&b,1,timeout)){lastAck=-1;return false;}lastAck=b;return b==0x79;}
  bool command(uint8_t c){uint8_t v[2]={c,(uint8_t)~c};return port.write(v,2)==2&&ack();}
  bool address(uint32_t a){uint8_t v[5];for(unsigned i=0;i<4;i++)v[i]=(uint8_t)(a>>(24-8*i));v[4]=v[0]^v[1]^v[2]^v[3];return port.write(v,5)==5&&ack();}
public:
  uint8_t eraseCommand=0;bool connected=false;
  const char *probeStage="NOT_STARTED";int lastAck=-1;
  uint32_t receivedBytes=0;uint16_t deviceId=0,flashKB=0;
  explicit Stm32Rom(HardwareSerial &p):port(p){}
  bool receive(uint8_t *p,size_t n,uint32_t timeout=1500){
    uint32_t start=millis();size_t i=0;
    while(i<n && (uint32_t)(millis()-start)<timeout){if(port.available())p[i++]=(uint8_t)port.read();else delay(1);}
    receivedBytes+=(uint32_t)i;return i==n;
  }
  void normal(){port.flush();port.end();port.begin(115200,SERIAL_8N1,20,21);connected=false;}
  bool probe(){
    if(connected)return true;
    connected=false;eraseCommand=0;lastAck=-1;receivedBytes=0;deviceId=0;flashKB=0;
    port.end();port.begin(115200,SERIAL_8E1,20,21);delay(20);
    while(port.available()){port.read();}
    probeStage="SYNC";port.write((uint8_t)0x7F);if(!ack())return false;
    probeStage="GET_COMMAND_ACK";
    if(!command(0x00))return false;
    probeStage="GET_COMMAND_BODY";
    uint8_t n,commands[32];if(!receive(&n,1)||n>=sizeof(commands)||!receive(commands,(size_t)n+1)||!ack())return false;
    bool rd=false,wr=false,go=false;
    for(unsigned i=1;i<=(unsigned)n;i++){if(commands[i]==0x44)eraseCommand=0x44;else if(commands[i]==0x43 && !eraseCommand)eraseCommand=0x43;rd|=commands[i]==0x11;wr|=commands[i]==0x31;go|=commands[i]==0x21;}
    probeStage="COMMAND_SUPPORT";if(!rd || !wr || !go || !eraseCommand)return false;
    probeStage="GET_ID_ACK";if(!command(0x02))return false;
    probeStage="GET_ID_BODY";
    uint8_t id[2];if(!receive(&n,1)||n!=1||!receive(id,2)||!ack())return false;
    deviceId=((uint16_t)id[0]<<8)|id[1];if(deviceId!=0x0413)return false;
    probeStage="FLASH_SIZE_READ";
    uint8_t size[2];if(!read(0x1FFF7A22UL,size,2))return false;
    flashKB=(uint16_t)size[0]|((uint16_t)size[1]<<8);if(flashKB!=1024)return false;
    probeStage="READY";connected=true;return true;
  }
  bool read(uint32_t a,uint8_t *p,size_t n){
    if(!n||n>256)return false;
    if(!((a>=STM32_FLASH_BASE && a<0x08100000UL && n<=0x08100000UL-a)||(a==0x1FFF7A22UL && n==2)))return false;
    if(!command(0x11)||!address(a))return false;
    uint8_t v[2]={(uint8_t)(n-1),(uint8_t)~(n-1)};return port.write(v,2)==2&&ack()&&receive(p,n,2000);
  }
  bool eraseImage(uint32_t size){
    int last=imageLastSector(size);if(!connected || last<0 || last>10 || !eraseCommand)return false;
    if(!command(eraseCommand))return false;
    uint8_t v[32];size_t n=0;
    if(eraseCommand==0x44){v[n++]=0;v[n++]=(uint8_t)last;for(int i=0;i<=last;i++){v[n++]=0;v[n++]=(uint8_t)i;}}
    else {v[n++]=(uint8_t)last;for(int i=0;i<=last;i++)v[n++]=(uint8_t)i;}
    uint8_t crc=0;for(size_t i=0;i<n;i++)crc^=v[i];v[n++]=crc;
    return port.write(v,n)==n&&ack(25000);
  }
  bool write(uint32_t a,const uint8_t *p,size_t n){
    if(!connected || !n || n>256 || (n&3U)||(a&3U)||a<STM32_FLASH_BASE||a>=STM32_CAL_BASE||n>STM32_CAL_BASE-a)return false;
    if(!command(0x31)||!address(a))return false;
    uint8_t v[258];v[0]=(uint8_t)(n-1);memcpy(v+1,p,n);uint8_t crc=v[0];for(size_t i=1;i<=n;i++)crc^=v[i];v[n+1]=crc;
    return port.write(v,n+2)==n+2&&ack(3000);
  }
  bool go(){return connected&&command(0x21)&&address(STM32_FLASH_BASE);}
};
