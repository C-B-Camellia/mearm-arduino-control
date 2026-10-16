#include "local_display.h"
#ifdef MEARM_NANO_OLED
#include <U8g2lib.h>
#include <Wire.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#endif
namespace {
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0,U8X8_PIN_NONE);
bool paging=false,dirty=true,dropped=false;
uint8_t localPage=0,renderedPage=0,localMode=0,renderedMode=0;
const char pages[5][6][22] PROGMEM={
    {"NORMAL","B1 PICK A/B/C","B2 RECORD","B3 PLAY","B4 HOME","H1 CAL H2 BASIC"},
    {"BASIC DRAW","B1 LINE","B2 N","B3 BACK","B4 HOME","H2 EXIT"},
    {"HEIGHT CAL","B1 DRAWZ","B2 TEACHZ","B3 TRAVELZ","B4 EXIT","H1EXIT H2CLR H3MARK"},
    {"TEACH","B1 MARK","B2 POLY","B3 CURVE","B4 UNDO","H1 EXIT H2 CLEAR"},
    {"DRAWING","B1 PAUSE","B2 RESUME","B3 CANCEL","B4 --","H2 BACK (PC STATUS)"}
};
void renderPage() {
    display.setFont(u8g2_font_6x10_tr);
    char line[22];
    for(uint8_t row=0;row<6;++row) {
        for(uint8_t i=0;i<22;++i) line[i]=pgm_read_byte(&pages[renderedPage][row][i]);
        display.drawStr(0,(row+1)*10,line);
    }
    // L1..L5 denotes local mapping only, never an acknowledgement from Uno.
    display.setCursor(104,10); display.print(dropped?'!':'L'); display.print(static_cast<uint8_t>(renderedMode+1));
}
}
void initializeLocalDisplay() {
    localPage=renderedPage=localMode=renderedMode=0; dirty=true; paging=false; dropped=false;
    display.setI2CAddress(MEARM_OLED_ADDRESS<<1);
    Wire.begin(); Wire.setWireTimeout(2500,true);
    display.setBusClock(400000); display.begin();
}
void selectLocalDisplayPage(PanelMode page) { localPage=static_cast<uint8_t>(page); dirty=true; }
void updateLocalDisplay(uint32_t,PanelMode mode,bool overflowed) {
    if(overflowed!=dropped || localMode!=static_cast<uint8_t>(mode)) { dropped=overflowed; localMode=static_cast<uint8_t>(mode); dirty=true; }
    if(!paging) {
        if(!dirty) return;
        dirty=false; renderedPage=localPage; renderedMode=localMode;
        display.firstPage(); paging=true;
    }
    renderPage(); paging=display.nextPage();
}
#else
void initializeLocalDisplay() {}
void selectLocalDisplayPage(PanelMode) {}
void updateLocalDisplay(uint32_t,PanelMode,bool) {}
#endif
