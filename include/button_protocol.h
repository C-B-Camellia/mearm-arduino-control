#pragma once
#include <stdint.h>
namespace ButtonWire {
constexpr uint8_t Size=8,Hello=0,Press=1;
inline uint8_t crc(const uint8_t* bytes) {
    uint8_t value=0;
    for(uint8_t i=0;i<6;++i) {
        value^=bytes[i];
        for(uint8_t b=0;b<8;++b) value=value&0x80?(value<<1)^0x07:value<<1;
    }
    return value;
}
inline void encode(uint8_t* bytes,uint8_t type,uint16_t epoch,uint8_t sequence,uint8_t button) {
    bytes[0]=0xA5; bytes[1]=type; bytes[2]=epoch; bytes[3]=epoch>>8;
    bytes[4]=sequence; bytes[5]=button; bytes[6]=crc(bytes); bytes[7]=0x5A;
}
inline bool valid(const uint8_t* bytes) {
    return bytes[0]==0xA5 && bytes[7]==0x5A && bytes[6]==crc(bytes) &&
        ((bytes[1]==Hello && bytes[5]==0) || (bytes[1]==Press && bytes[5]>=1 && bytes[5]<=4));
}
}
