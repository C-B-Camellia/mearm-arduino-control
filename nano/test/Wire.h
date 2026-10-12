#pragma once
#include <stdint.h>
struct WireMock {
    uint32_t timeout=0;
    void begin() {}
    void setWireTimeout(uint32_t t,bool reset) { timeout=reset?t:0; }
};
extern WireMock Wire;
