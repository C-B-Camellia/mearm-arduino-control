#include <Arduino.h>
#include <EEPROM.h>
#include "button_scan.h"
#include "../../include/button_protocol.h"
namespace {
DebouncedButton buttons[4];
uint16_t epoch=0;
uint8_t sequence=0,queue[8],head=0,tail=0,lastButton=0;
uint32_t helloAt=0;
bool helloPending=true;
void sendFrame(uint8_t type,uint8_t button) {
    uint8_t bytes[ButtonWire::Size];
    ButtonWire::encode(bytes,type,epoch,sequence,button);
    Serial.write(bytes,sizeof(bytes));
}
}
void setup() {
    Serial.begin(9600);
    // Persistent boot epoch prevents old-session events being accepted after reset.
    epoch=EEPROM.read(0)|(static_cast<uint16_t>(EEPROM.read(1))<<8);
    ++epoch; EEPROM.update(0,epoch); EEPROM.update(1,epoch>>8);
    for(uint8_t i=0;i<4;++i) { pinMode(4+i,INPUT_PULLUP); buttons[i].initialize(digitalRead(4+i)==LOW,millis()); }
}
void loop() {
    uint32_t now=millis();
    for(uint8_t i=0;i<4;++i) {
        if(buttons[i].observe(digitalRead(4+i)==LOW,now)) {
            uint8_t next=(head+1)&7;
            if(next!=tail) { queue[head]=i+1; head=next; }
        }
    }
    if(Serial.availableForWrite()<ButtonWire::Size) return;
    if(helloPending || (head==tail && now-helloAt>=1000)) {
        sendFrame(ButtonWire::Hello,0); helloAt=now; helloPending=false;
    } else if(head!=tail) {
        lastButton=queue[tail]; tail=(tail+1)&7; ++sequence;
        sendFrame(ButtonWire::Press,lastButton);
    }
}
