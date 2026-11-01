#pragma once
#include <stdint.h>
#include <stddef.h>
class RadioLineAssembler {
  char line[1024]{};
  size_t used=0;
  bool synchronized=false,haveSequence=false;
  uint32_t expected=0,last=0;
public:
  uint32_t gaps=0,duplicates=0,reordered=0,overlong=0;
  template<class Emit> void packet(uint32_t sequence,const uint8_t *data,size_t size,Emit emit){
    if(haveSequence){
      if(sequence==last){duplicates++;return;}
      if(sequence!=expected){
        if(sequence && (int32_t)(sequence-expected)<0){reordered++;return;}
        gaps++;used=0;synchronized=false;
      }
    }
    last=sequence;expected=sequence+1U;haveSequence=true;
    for(size_t i=0;i<size;i++){
      const char c=(char)data[i];
      if(!synchronized){if(c=='\n')synchronized=true;continue;}
      if(c=='\n'){line[used++]='\n';emit(line,used);used=0;continue;}
      if(((uint8_t)c<32U && c!='\r') || (uint8_t)c>126U){used=0;synchronized=false;continue;}
      if(used>=sizeof(line)-2U){overlong++;used=0;synchronized=false;continue;}
      line[used++]=c;
    }
  }
};
