#include "../esp32s3_ground_station/line_assembler.h"
#include "../esp32c3_uart_bridge/telemetry_protocol.h"
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>
#include <cstring>
int main(){
  static_assert(TELEMETRY_HEADER_SIZE==12&&sizeof(TelemetryPacket)==212);
  const uint8_t digits[]="123456789";assert(crc16Update(0xFFFF,digits,9)==0x29B1);
  TelemetryPacket p{};p.magic=TELEMETRY_MAGIC;p.version=TELEMETRY_VERSION;p.type=TYPE_UART0_TO_GROUND;
  p.sequence=42;p.length=200;memset(p.payload,'A',200);p.crc=packetCrc(p);
  assert(packetIsValid(p,sizeof(p)));p.payload[1]^=1;assert(!packetIsValid(p,sizeof(p)));p.payload[1]^=1;
  assert(!packetIsValid(p,sizeof(p)-1));
  RadioLineAssembler a;std::vector<std::string> rows;
  auto emit=[&](const char *s,size_t n){rows.emplace_back(s,n);};
  auto push=[&](uint32_t seq,const std::string &s){a.packet(seq,(const uint8_t *)s.data(),s.size(),emit);};
  push(10,"partial\r\nSEQ=1 T_ms=0\r\nSL");push(11,"OW=1 T_ms=0\r\n");
  assert(rows.size()==2&&rows[0]=="SEQ=1 T_ms=0\r\n"&&rows[1]=="SLOW=1 T_ms=0\r\n");
  push(11,"OW=1 T_ms=0\r\n");assert(rows.size()==2&&a.duplicates==1);
  push(13,"broken\nSEQ=2 T_ms=50\n");assert(rows.size()==3&&a.gaps==1&&rows.back()=="SEQ=2 T_ms=50\n");
  push(12,"OLD\n");assert(rows.size()==3&&a.reordered==1);
  push(14,std::string(1100,'X')+"\nSEQ=3\n");assert(a.overlong==1&&rows.size()==4&&rows.back()=="SEQ=3\n");
  push(0,"restart\nSEQ=4\n");assert(a.gaps==2&&rows.back()=="SEQ=4\n");
  RadioLineAssembler wrap;rows.clear();wrap.packet(0xFFFFFFFFU,(const uint8_t *)"x\nSEQ=6\n",8,emit);
  wrap.packet(0,(const uint8_t *)"SEQ=7\n",6,emit);assert(rows.size()==2&&wrap.gaps==0);
  const std::string wire="discard-first\nSEQ=1 R=0 P=0 Y=0\r\nSLOW=1 FW=AXIS_FRAME_RP_UART_V2\r\nDIAG=1 EC3=0,1,2\r\n";
  for(unsigned size=1;size<=200;size++){
    RadioLineAssembler f;rows.clear();uint32_t seq=0;
    for(size_t i=0;i<wire.size();i+=size)f.packet(seq++,(const uint8_t *)wire.data()+i,std::min((size_t)size,wire.size()-i),emit);
    assert(rows.size()==3&&rows[0]=="SEQ=1 R=0 P=0 Y=0\r\n"&&rows[1]=="SLOW=1 FW=AXIS_FRAME_RP_UART_V2\r\n");
  }
  puts("PASS radio legacy wire CRC/header, arbitrary 1..200-byte fragments, whole lines, lost/duplicate/reordered packets, peer restart, sequence wrap and overlong resync");
}
