#include "local_display.h"
#ifdef MEARM_NANO_OLED
#include <U8g2lib.h>
#include <Wire.h>
namespace {
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0,U8X8_PIN_NONE);
bool paging=false;
uint32_t refreshedAt=0;
uint8_t key=0,sent=0;
bool dropped=false;
void renderPage() {
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(0,10,"MeArm Nano");
    display.drawStr(0,20,"LINK WAIT (one-way)");
    display.setCursor(0,30); display.print(F("LOCAL KEY: "));
    if(key) { display.print('B'); display.print(key); } else display.print('-');
    display.setCursor(0,40); display.print(F("TX sequence: ")); display.print(sent);
    display.drawStr(0,50,dropped?"KEY QUEUE FULL":"TX != accepted");
    display.drawStr(0,60,"PC = SYSTEM STATUS");
}
}
void initializeLocalDisplay() {
    display.setI2CAddress(MEARM_OLED_ADDRESS<<1);
    Wire.begin(); Wire.setWireTimeout(2500,true);
    display.setBusClock(400000); display.begin();
}
void updateLocalDisplay(uint32_t now,uint8_t lastButton,uint8_t sequence,bool overflowed) {
    if(!paging) {
        if(now-refreshedAt<300) return;
        refreshedAt=now; key=lastButton; sent=sequence; dropped=overflowed;
        display.firstPage(); paging=true;
    }
    // One page per loop: scan inputs and drain TX between pages.
    renderPage(); paging=display.nextPage();
}
#else
void initializeLocalDisplay() {}
void updateLocalDisplay(uint32_t,uint8_t,uint8_t,bool) {}
#endif
