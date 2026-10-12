#pragma once
#include "Arduino.h"
constexpr int U8G2_R0=0,U8X8_PIN_NONE=255;
const uint8_t u8g2_font_6x10_tr[]={0};
struct U8G2_SSD1306_128X64_NONAME_1_HW_I2C {
    uint8_t address=0,page=0;
    unsigned pages=0,refreshes=0;
    uint32_t clock=0;
    std::vector<std::string> lines;
    U8G2_SSD1306_128X64_NONAME_1_HW_I2C(int,int) {}
    void setI2CAddress(uint8_t a) { address=a; }
    void setBusClock(uint32_t c) { clock=c; }
    void begin() {}
    void firstPage() { page=0; ++refreshes; }
    bool nextPage() { ++pages; mockTime+=3; return ++page<8; }
    void setFont(const uint8_t*) {}
    void drawStr(int,int,const char* p) { lines.emplace_back(p); }
    void setCursor(int,int) { lines.emplace_back(); }
    void print(const __FlashStringHelper* p) { lines.back()+=reinterpret_cast<const char*>(p); }
    void print(char c) { lines.back()+=c; }
    void print(uint8_t n) { lines.back()+=std::to_string(n); }
};
