#pragma once
#include <stdint.h>
#include <stddef.h>
#include <deque>
#include <vector>
inline uint32_t simulatedMs;
inline uint32_t millis(){return simulatedMs;}
inline void delay(uint32_t n){simulatedMs+=n;}
constexpr int SERIAL_8E1=1,SERIAL_8N1=2;
class HardwareSerial {
public:
  std::deque<std::vector<uint8_t>> replies;
  std::deque<uint8_t> rx;
  std::vector<std::vector<uint8_t>> tx;
  int config=0;
  void begin(int,int c,int,int){config=c;}
  void end(){}void flush(){}
  int available(){return (int)rx.size();}
  int read(){if(rx.empty())return -1;int b=rx.front();rx.pop_front();return b;}
  size_t write(const uint8_t *p,size_t n){tx.emplace_back(p,p+n);if(!replies.empty()){for(auto b:replies.front())rx.push_back(b);replies.pop_front();}return n;}
  size_t write(uint8_t b){return write(&b,1);}
};
