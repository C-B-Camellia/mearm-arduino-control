#if defined(MEARM_NANO_LINK)
#include "button_link.h"
#include "button_protocol.h"
#include "pin_uart.h"
#include "app_controller.h"
#include "control_error.h"
namespace {
uint8_t frame[ButtonWire::Size],used=0,lastSequence=0;
uint16_t epoch=0;
uint32_t lastByte=0;
bool synchronized=false;
}
bool buttonLinkConfigured() { return true; }
void initializeButtonLink() { used=0; synchronized=false; initializePinUart(); }
void feedButtonByte(char c,uint32_t now) {
    if(used && now-lastByte>40) { used=0; synchronized=false; }
    lastByte=now;
    uint8_t value=static_cast<uint8_t>(c);
    if(!used && value!=0xA5) return;
    frame[used++]=value;
    if(used!=ButtonWire::Size) return;
    used=0;
    if(!ButtonWire::valid(frame)) { synchronized=false; return; }
    uint16_t incoming=frame[2]|(static_cast<uint16_t>(frame[3])<<8);
    if(frame[1]==ButtonWire::Hello) {
        epoch=incoming; lastSequence=frame[4]; synchronized=true; return;
    }
    if(!synchronized || incoming!=epoch) return;
    if(frame[4]==lastSequence) return;
    if(frame[4]!=static_cast<uint8_t>(lastSequence+1)) { synchronized=false; return; }
    lastSequence=frame[4];
    if(dispatchAction({static_cast<AppAction>(frame[5]),0})) Serial.println(F("OK REMOTE")); else printControlError();
}
void pollButtonLink() {
    if(pinUartOverflow()) { used=0; synchronized=false; }
    for(uint8_t count=0;count<16;++count) {
        int value=readPinUart(); if(value<0) break;
        feedButtonByte(static_cast<char>(value),millis());
    }
}
#else
#include "button_link.h"
bool buttonLinkConfigured() { return false; }
void initializeButtonLink() {}
void feedButtonByte(char,uint32_t) {}
void pollButtonLink() {}
#endif
