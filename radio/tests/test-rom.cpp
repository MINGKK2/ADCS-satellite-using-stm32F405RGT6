#include "stm32_rom.h"
#include <cassert>
#include <cstdio>
int main(){
  uint8_t vector[8];put32(vector,0x20020000);put32(vector+4,0x08000101);
  assert(imageVectorValid(vector,120000));assert(!imageVectorValid(vector,0xE0004));
  put32(vector+4,0x080E0001);assert(!imageVectorValid(vector,120000));
  assert(imageLastSector(1)==0&&imageLastSector(0x10000)==3&&imageLastSector(0x10001)==4);
  assert(imageLastSector(0x20000)==4&&imageLastSector(0x20001)==5&&imageLastSector(0xE0000)==10);
  assert(imageLastSector(0xE0001)==-1);
  const uint8_t text[]="123456789";assert(~imageCrcUpdate(~0U,text,9)==0xCBF43926U);
  RequestGuard guard;assert(guard.check(42,1,100,true)==0);assert(guard.check(42,1,100,true)==1);
  assert(guard.check(42,1,101,true)==-1);assert(guard.check(42,2,102,false)==0);
  assert(guard.check(42,1,100,false)==-1);assert(guard.check(43,3,103,false)==-1);
  HardwareSerial uart;Stm32Rom rom(uart);uint8_t bytes[256]={0};
  assert(!rom.write(STM32_FLASH_BASE,bytes,256));assert(!rom.eraseImage(120000));assert(uart.tx.empty());
  uart.replies={{0x79},{0x79,4,0x31,0x11,0x31,0x44,0x21,0x79},
      {0x79,1,0x04,0x13,0x79},{0x79},{0x79},{0x79,0,4}};
  assert(rom.probe()&&rom.connected&&rom.eraseCommand==0x44&&uart.config==SERIAL_8E1);
  assert(uart.tx[0]==std::vector<uint8_t>({0x7F}));size_t old=uart.tx.size();assert(rom.probe()&&uart.tx.size()==old);
  assert(!rom.write(STM32_CAL_BASE,bytes,4));assert(!rom.write(STM32_CAL_BASE-4,bytes,8));
  assert(!rom.write(STM32_FLASH_BASE+1,bytes,4));assert(!rom.write(STM32_FLASH_BASE,bytes,3));
  assert(!rom.read(0xFFFFFFF0,bytes,256));assert(!rom.eraseImage(0xE0004));assert(uart.tx.size()==old);
  uart.replies={{0x79},{0x79}};assert(rom.eraseImage(120000));
  assert(uart.tx[old]==std::vector<uint8_t>({0x44,0xBB}));
  const auto &erase=uart.tx[old+1];assert(erase.size()==13&&erase[0]==0&&erase[1]==4);
  for(unsigned i=0;i<5;i++)assert(erase[2+2*i]==0&&erase[3+2*i]==i);
  uint8_t crc=0;for(auto b:erase)crc^=b;assert(!crc);
  uart.replies={{0x79},{0x79},{0x79}};assert(rom.write(STM32_FLASH_BASE,bytes,256));
  assert(uart.tx.back().size()==258&&uart.tx.back()[0]==255);
  uart.replies={{0x1F}};old=uart.tx.size();assert(!rom.write(STM32_FLASH_BASE,bytes,256));assert(uart.tx.size()==old+1);
  rom.normal();assert(!rom.connected&&uart.config==SERIAL_8N1);
  HardwareSerial silent;Stm32Rom absent(silent);assert(!absent.probe());assert(!absent.connected);
  puts("PASS F405 ROM protocol: 8E1/sync/ID/flash size, command/address ACK, NACK/timeout, sector-only erase, no sector11/overflow/unaligned write, 8N1 restore");
  puts("PASS image vector/range/CRC and retry/session guard: duplicate request cannot append or erase twice");
}
