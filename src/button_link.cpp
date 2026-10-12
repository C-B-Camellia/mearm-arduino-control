#include "button_link.h"
#include "app_controller.h"
#include "control_error.h"
#if defined(MEARM_BUTTON_RX) && defined(MEARM_BUTTON_TX) && defined(MEARM_BUTTON_BAUD)
#include <SoftwareSerial.h>
static_assert(MEARM_BUTTON_RX>=2 && MEARM_BUTTON_RX<=13 && (MEARM_BUTTON_RX<6 || MEARM_BUTTON_RX>9),"button RX conflicts with USB/Servo/ADC pins");
static_assert(MEARM_BUTTON_TX>=2 && MEARM_BUTTON_TX<=13 && (MEARM_BUTTON_TX<6 || MEARM_BUTTON_TX>9) && MEARM_BUTTON_TX!=MEARM_BUTTON_RX,"button TX conflicts with occupied pins");
namespace { SoftwareSerial link(MEARM_BUTTON_RX,MEARM_BUTTON_TX); }
#define MEARM_LINK_CONFIGURED
#endif
namespace { char frame[3]; uint8_t used=0,lastButton=0; bool discard=false; uint32_t lastEvent=0; }
bool buttonLinkConfigured() {
#ifdef MEARM_LINK_CONFIGURED
    return true;
#else
    return false;
#endif
}
void initializeButtonLink() {
    used=lastButton=0; discard=false; lastEvent=0;
#ifdef MEARM_LINK_CONFIGURED
    link.begin(MEARM_BUTTON_BAUD);
#endif
}
void feedButtonByte(char c,uint32_t now) {
    if(c=='\r') return;
    if(c=='\n') {
        if(!discard && used==2 && frame[0]=='B' && frame[1]>='1' && frame[1]<='4') {
            uint8_t button=frame[1]-'0';
            if(button!=lastButton || static_cast<uint32_t>(now-lastEvent)>=30) {
                lastButton=button; lastEvent=now;
                if(onButtonPressed(button)) Serial.println(F("OK BTN")); else printControlError();
            }
        } else if(used || discard) { rejectControl(ControlError::Invalid); printControlError(); }
        used=0; discard=false; return;
    }
    if(discard) return;
    if(used<2) frame[used++]=c; else discard=true;
}
void pollButtonLink() {
#ifdef MEARM_LINK_CONFIGURED
    for(uint8_t count=0;count<8 && link.available();++count) feedButtonByte(static_cast<char>(link.read()),millis());
#endif
}
