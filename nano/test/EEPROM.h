#pragma once
#include <stdint.h>
#include <string.h>
struct EepromMock {
    uint8_t bytes[1024];
    EepromMock() { memset(bytes,255,sizeof(bytes)); }
    uint8_t read(int at) { return bytes[at]; }
    void update(int at,uint8_t value) { bytes[at]=value; }
};
extern EepromMock EEPROM;
