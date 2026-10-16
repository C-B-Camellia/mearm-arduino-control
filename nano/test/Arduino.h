#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>
constexpr int LOW=0,HIGH=1,INPUT=0,INPUT_PULLUP=2;
constexpr int A0=14,A1=15,A2=16,A3=17;
class __FlashStringHelper;
#define F(s) reinterpret_cast<const __FlashStringHelper*>(s)
extern uint32_t mockTime;
extern int levels[20],modes[20];
inline uint32_t millis() { return mockTime; }
inline int digitalRead(int pin) { return levels[pin]; }
inline void pinMode(int pin,int mode) { modes[pin]=mode; }
struct SerialMock {
    unsigned long baud=0;
    int room=64;
    std::vector<uint8_t> bytes;
    void begin(unsigned long b) { baud=b; }
    int availableForWrite() const { return room; }
    void write(const uint8_t* p,size_t count) { bytes.insert(bytes.end(),p,p+count); }
};
extern SerialMock Serial;
