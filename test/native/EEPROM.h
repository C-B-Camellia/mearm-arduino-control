#pragma once
#include <stdint.h>
#include <string.h>
struct EEPROMStub {
    uint8_t bytes[1024];
    EEPROMStub() { reset(); }
    uint8_t read(int at) const { return bytes[at]; }
    void update(int at,uint8_t value) { bytes[at]=value; }
    void reset() { memset(bytes,255,sizeof(bytes)); }
};
extern EEPROMStub EEPROM;
