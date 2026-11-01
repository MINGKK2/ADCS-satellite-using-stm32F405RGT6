#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
enum UpdateOp : uint8_t { FW_PING=1,FW_START,FW_CHUNK,FW_SEAL,FW_PROBE,FW_CAL_HASH,FW_PROGRAM,FW_GO,FW_ABORT };
enum UpdateStatus : uint8_t { FW_OK=0,FW_BAD_REQUEST,FW_SEQUENCE,FW_IO,FW_CRC,FW_NOT_READY,FW_ROM_ERROR,FW_WRONG_MCU,FW_VERIFY,FW_CAL_CHANGED,FW_PROGRESS=254 };
constexpr uint8_t TYPE_UPDATE_REQUEST=0x40,TYPE_UPDATE_RESPONSE=0x41;
constexpr uint32_t STM32_FLASH_BASE=0x08000000UL,STM32_CAL_BASE=0x080E0000UL;
constexpr uint32_t STM32_MAX_IMAGE=STM32_CAL_BASE-STM32_FLASH_BASE;
inline uint32_t wire32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
inline void put32(uint8_t *p,uint32_t n){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(n>>(8*i));}
inline uint32_t imageCrcUpdate(uint32_t crc,const uint8_t *p,size_t n){while(n--){crc^=*p++;for(unsigned i=0;i<8;i++)crc=(crc>>1)^((crc&1)?0xEDB88320UL:0);}return crc;}
inline bool imageVectorValid(const uint8_t *v,uint32_t size){
  if(size<8 || size>STM32_MAX_IMAGE || (size&3U))return false;
  const uint32_t stack=wire32(v),reset=wire32(v+4);
  return stack>0x20000000UL && stack<=0x20020000UL && !(stack&7U)
      && (reset&1U) && (reset&~1UL)>=STM32_FLASH_BASE && (reset&~1UL)<STM32_FLASH_BASE+size;
}
inline int imageLastSector(uint32_t size){
  if(!size || size>STM32_MAX_IMAGE)return -1;
  const uint32_t end=STM32_FLASH_BASE+size-1;
  if(end<0x08010000UL)return (int)((end-STM32_FLASH_BASE)/0x4000UL);
  if(end<0x08020000UL)return 4;
  return 5+(int)((end-0x08020000UL)/0x20000UL);
}
struct RequestGuard {
  uint32_t session=0,last=0;uint16_t crc=0;
  // 0=new, 1=exact retry, -1=out-of-order/session/changed payload.
  int check(uint32_t s,uint32_t seq,uint16_t c,bool ping){
    if(!s || !seq)return -1;
    if(ping && s!=session){session=s;last=0;crc=0;}
    if(s!=session || seq<last || (seq==last && c!=crc))return -1;
    if(seq==last)return 1;
    last=seq;crc=c;return 0;
  }
};
