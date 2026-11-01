#pragma once
#include <LittleFS.h>
#include <mbedtls/sha256.h>
#include "stm32_rom.h"
#include "telemetry_protocol.h"
bool FirmwareUpdaterTransport_Send(const uint8_t *peer,const uint8_t *data,size_t size);
class FirmwareUpdater {
  Stm32Rom rom;const uint8_t *peer;RequestGuard guard;
  TelemetryPacket cached{};bool haveCache=false,active=false,ready=false,verified=false,calCaptured=false;
  File incoming;uint32_t imageSize=0,imageCrc=0,received=0,currentSeq=0,currentSession=0;uint8_t currentOp=0;
  uint8_t calBefore[32]{};
  static constexpr const char *path="/stm32-update.bin";
  void send(const TelemetryPacket &p){FirmwareUpdaterTransport_Send(peer,reinterpret_cast<const uint8_t *>(&p),TELEMETRY_HEADER_SIZE+p.length);}
  void response(uint8_t status,const void *body=nullptr,size_t n=0,bool cache=true){
    TelemetryPacket p{};p.magic=TELEMETRY_MAGIC;p.version=TELEMETRY_VERSION;p.type=TYPE_UPDATE_RESPONSE;p.sequence=currentSeq;
    n=n>TELEMETRY_MAX_PAYLOAD-6?TELEMETRY_MAX_PAYLOAD-6:n;
    p.payload[0]=currentOp;p.payload[1]=status;put32(p.payload+2,currentSession);
    if(n)memcpy(p.payload+6,body,n);p.length=(uint16_t)(n+6);p.crc=packetCrc(p);
    if(cache){cached=p;haveCache=true;}send(p);
  }
  void error(uint8_t status,const char *message){response(status,message,strlen(message));}
  void progress(uint8_t phase,uint8_t percent){uint8_t v[2]={phase,percent};response(FW_PROGRESS,v,2,false);delay(1);}
  bool fileValid(){
    File file=LittleFS.open(path,"r");if(!file || file.size()!=imageSize)return false;
    uint8_t block[256];uint32_t count=0,crc=~0U;bool vector=false;
    while(file.available()){
      size_t n=file.read(block,sizeof(block));if(!n)return false;
      if(!count)vector=imageVectorValid(block,imageSize);
      crc=imageCrcUpdate(crc,block,n);count+=(uint32_t)n;delay(1);
    }
    return vector && count==imageSize && ~crc==imageCrc;
  }
  bool calHash(uint8_t output[32]){
    if(!rom.connected)return false;
    mbedtls_sha256_context sha;mbedtls_sha256_init(&sha);
    bool ok=mbedtls_sha256_starts(&sha,0)==0;uint8_t block[256];
    for(uint32_t offset=0;ok && offset<0x20000UL;offset+=sizeof(block)){
      ok=rom.read(STM32_CAL_BASE+offset,block,sizeof(block))&&mbedtls_sha256_update(&sha,block,sizeof(block))==0;
      if(offset%8192==0)progress(1,(uint8_t)(offset*100/0x20000UL));delay(1);
    }
    if(ok)ok=mbedtls_sha256_finish(&sha,output)==0;mbedtls_sha256_free(&sha);return ok;
  }
public:
  FirmwareUpdater(HardwareSerial &uart,const uint8_t *address):rom(uart),peer(address){}
  bool busy() const{return active;}
  void handle(const TelemetryPacket &request){
    if(request.length<5)return;
    currentOp=request.payload[0];currentSession=wire32(request.payload+1);currentSeq=request.sequence;
    int verdict=guard.check(currentSession,currentSeq,request.crc,currentOp==FW_PING);
    if(verdict==1){if(haveCache)send(cached);return;}
    if(verdict<0){const char reason[]="SESSION_OR_SEQUENCE";response(FW_SEQUENCE,reason,sizeof(reason)-1,false);return;}
    const uint8_t *body=request.payload+5;size_t n=request.length-5;
    switch(currentOp){
    case FW_PING:{char id[160];int n=snprintf(id,sizeof(id),"C3_STM32_UPDATER_V1 MANUAL_BOOT UART0=20,21 READY=%u SIZE=%lu CRC=%08lX",ready,(unsigned long)imageSize,(unsigned long)imageCrc);response(FW_OK,id,(size_t)n);break;}
    case FW_START:{
      if(n!=8){error(FW_BAD_REQUEST,"START_SIZE_CRC");break;}
      uint32_t size=wire32(body);if(size<8 || size>STM32_MAX_IMAGE || (size&3U)){error(FW_BAD_REQUEST,"IMAGE_RANGE_ALIGNMENT");break;}
      if(incoming)incoming.close();active=true;ready=false;verified=false;calCaptured=false;received=0;
      imageSize=size;imageCrc=wire32(body+4);
      // This bridge did not previously use its filesystem; mount/format only
      // its dedicated data partition, never an STM32 or ESP application region.
      if(!LittleFS.begin(true)){error(FW_IO,"FILESYSTEM_MOUNT");break;}
      incoming=LittleFS.open(path,"w");if(!incoming){error(FW_IO,"IMAGE_OPEN");break;}
      response(FW_OK);break;
    }
    case FW_CHUNK:{
      if(!active || !incoming || n<=4 || wire32(body)!=received || n-4>imageSize-received){error(FW_BAD_REQUEST,"CHUNK_OFFSET_OR_SIZE");break;}
      size_t written=incoming.write(body+4,n-4);if(written!=n-4){error(FW_IO,"IMAGE_WRITE");break;}
      received+=(uint32_t)written;uint8_t offset[4];put32(offset,received);response(FW_OK,offset,4);break;
    }
    case FW_SEAL:{
      if(n || !active || !incoming || received!=imageSize){error(FW_NOT_READY,"INCOMPLETE_IMAGE");break;}
      incoming.flush();incoming.close();ready=fileValid();if(!ready){error(FW_CRC,"IMAGE_CRC_OR_VECTOR");break;}
      response(FW_OK);break;
    }
    case FW_PROBE:{
      if(n || !ready){error(FW_NOT_READY,"SEAL_IMAGE_FIRST");break;}
      if(!rom.probe()){error(FW_ROM_ERROR,"NO_F405_1MB_BOOTLOADER_CHECK_B0_RESET_UART");break;}
      const char id[]="STM32F405_ID0413_FLASH1024KB";response(FW_OK,id,sizeof(id)-1);break;
    }
    case FW_CAL_HASH:{
      uint8_t hash[32];if(n || !calHash(hash)){error(FW_ROM_ERROR,"CAL_READ");break;}
      if(!calCaptured){memcpy(calBefore,hash,32);calCaptured=true;}
      response(FW_OK,hash,32);break;
    }
    case FW_PROGRAM:{
      if(n || !ready || !rom.connected || !calCaptured){error(FW_NOT_READY,"IMAGE_PROBE_CAL_REQUIRED");break;}
      if(verified){response(FW_OK,calBefore,32);break;} // idempotent even with a new request id
      if(!fileValid()){error(FW_CRC,"IMAGE_RECHECK_FAILED");break;}
      uint8_t hash[32];if(!calHash(hash)||memcmp(hash,calBefore,32)){error(FW_CAL_CHANGED,"CAL_CHANGED_BEFORE_ERASE");break;}
      progress(2,0);if(!rom.eraseImage(imageSize)){error(FW_ROM_ERROR,"SECTOR_ERASE_FAILED");break;}
      File file=LittleFS.open(path,"r");if(!file){error(FW_IO,"IMAGE_REOPEN");break;}
      uint8_t block[256],readback[256];bool ok=true;
      for(uint32_t offset=0;offset<imageSize;){
        size_t length=(imageSize-offset>sizeof(block))?sizeof(block):(imageSize-offset);
        if(file.read(block,length)!=length || !rom.write(STM32_FLASH_BASE+offset,block,length)
            || !rom.read(STM32_FLASH_BASE+offset,readback,length) || memcmp(block,readback,length)){ok=false;break;}
        offset+=(uint32_t)length;if(offset%4096==0 || offset==imageSize)progress(3,(uint8_t)(offset*100/imageSize));delay(1);
      }
      file.close();if(!ok){error(FW_VERIFY,"PROGRAM_OR_READBACK_FAILED_REMAIN_IN_BOOTLOADER");break;}
      if(!calHash(hash)||memcmp(hash,calBefore,32)){error(FW_CAL_CHANGED,"CAL_VERIFY_FAILED_NO_GO");break;}
      verified=true;response(FW_OK,hash,32);break;
    }
    case FW_GO:{
      if(n || !verified || !rom.go()){error(FW_NOT_READY,"VERIFIED_PROGRAM_REQUIRED");break;}
      response(FW_OK);rom.normal();active=false;break;
    }
    case FW_ABORT:{
      if(incoming)incoming.close();active=false;rom.normal();response(FW_OK);break;
    }
    default:error(FW_BAD_REQUEST,"UNKNOWN_OP");break;
    }
  }
};
