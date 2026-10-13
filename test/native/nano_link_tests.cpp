#include "Arduino.h"
#include "EEPROM.h"
#include "button_link.h"
#include "button_protocol.h"
#include "pin_uart.h"
#include "recording.h"
#include "arm.h"
#include <iostream>
#include <cstdlib>
uint32_t simulatedTime=0;
std::vector<ServoWrite> servoWrites;
std::vector<int> attachedPins;
std::vector<uint32_t> delayCalls;
int analogValues[4]={506,519,515,519};
SerialStub Serial;
EEPROMStub EEPROM;
void setup(); void loop();
namespace {
std::vector<uint8_t> rx;
bool rxError=false;
unsigned checks=0;
void check(bool ok,int line) { ++checks; if(!ok) { std::cerr<<"FAIL LINK "<<line<<"\n"; std::exit(1); } }
#define CHECK(x) check((x),__LINE__)
void packet(uint8_t type,uint16_t epoch,uint8_t seq,uint8_t key,uint8_t* out) { ButtonWire::encode(out,type,epoch,seq,key); }
void feed(uint8_t type,uint16_t epoch,uint8_t seq,uint8_t key) {
    uint8_t bytes[8]; packet(type,epoch,seq,key,bytes);
    for(uint8_t byte:bytes) feedButtonByte(byte,simulatedTime++);
}
void enqueue(uint8_t type,uint16_t epoch,uint8_t seq,uint8_t key) {
    uint8_t bytes[8]; packet(type,epoch,seq,key,bytes); rx.insert(rx.end(),bytes,bytes+8);
}
}
void initializePinUart() { rx.clear(); rxError=false; }
int readPinUart() { if(rx.empty()) return -1; uint8_t v=rx.front(); rx.erase(rx.begin()); return v; }
bool pinUartOverflow() { bool e=rxError; rxError=false; if(e) rx.clear(); return e; }
int main() {
    setup(); simulatedTime+=20; loop(); servoWrites.clear();
    CHECK(buttonLinkConfigured() && Serial.baud==115200);
    feed(ButtonWire::Press,42,1,2); CHECK(!isRecording()); // No HELLO, no action.
    feed(ButtonWire::Hello,42,0,0); CHECK(!isRecording());
    feed(ButtonWire::Press,42,1,2); CHECK(recordingStatus().state==RecordState::Arming);
    simulatedTime+=100; feed(ButtonWire::Press,42,1,2); CHECK(isRecording()); // Duplicate.
    uint8_t invalid[8]; packet(ButtonWire::Press,42,2,2,invalid); invalid[6]^=1;
    for(uint8_t byte:invalid) feedButtonByte(byte,simulatedTime++);
    feed(ButtonWire::Press,42,2,2); CHECK(isRecording()); // CRC error requires HELLO.
    feed(ButtonWire::Hello,42,2,0); feed(ButtonWire::Press,42,3,2); CHECK(!isRecording());
    feed(ButtonWire::Press,43,4,2); CHECK(!isRecording()); // Wrong boot epoch.
    feed(ButtonWire::Press,42,4,2); CHECK(isRecording());
    feed(ButtonWire::Press,42,6,2); CHECK(isRecording()); // Gap disarms.
    feed(ButtonWire::Hello,43,0,0); feed(ButtonWire::Press,42,7,2); CHECK(isRecording());
    feed(ButtonWire::Press,43,1,2); CHECK(!isRecording());
    // Invalid key/header/end/type and valid CRC must still fail safely.
    for(uint8_t kind=0;kind<5;++kind) {
        feed(ButtonWire::Hello,43,1,0);
        packet(ButtonWire::Press,43,2,kind==0?0:kind==1?5:2,invalid);
        if(kind==2) invalid[0]=0;
        if(kind==3) invalid[7]=0;
        if(kind==4) invalid[1]=9;
        invalid[6]=ButtonWire::crc(invalid);
        for(uint8_t byte:invalid) feedButtonByte(byte,simulatedTime++);
        CHECK(!isRecording()); // Malformed bytes themselves never invoke a key.
        feed(ButtonWire::Press,43,2,2);
        CHECK(kind==2?isRecording():!isRecording()); // Header noise is skipped; complete bad frames disarm.
        clearRecording(); initializeButtonLink();
    }
    feed(ButtonWire::Hello,43,255,0); feed(ButtonWire::Press,43,0,2); CHECK(isRecording());
    feed(ButtonWire::Hello,44,10,0); // Reboot HELLO itself does not cancel/start.
    CHECK(isRecording()); feed(ButtonWire::Press,44,11,2); CHECK(!isRecording());
    packet(ButtonWire::Press,44,12,2,invalid);
    for(uint8_t i=0;i<3;++i) feedButtonByte(invalid[i],simulatedTime++);
    simulatedTime+=41;
    for(uint8_t i=3;i<8;++i) feedButtonByte(invalid[i],simulatedTime++);
    feed(ButtonWire::Press,44,12,2); CHECK(!isRecording());
    initializeButtonLink(); feed(ButtonWire::Hello,44,12,0);
    feed(ButtonWire::Press,44,13,2); CHECK(isRecording());
    clearRecording(); initializeButtonLink();
    feed(ButtonWire::Hello,45,0,0);
    enqueue(ButtonWire::Press,45,1,2); rxError=true; pollButtonLink();
    CHECK(!isRecording() && rx.empty());
    enqueue(ButtonWire::Press,45,1,2); pollButtonLink(); CHECK(!isRecording());
    // Real main loop: PC STATUS remains available with Nano frames queued.
    enqueue(ButtonWire::Hello,45,1,0); enqueue(ButtonWire::Press,45,2,2); enqueue(ButtonWire::Press,45,3,2);
    Serial.output.clear(); Serial.input="STATUS\n"; simulatedTime+=20; loop();
    CHECK(Serial.input.empty() && Serial.output.find("task=")!=std::string::npos);
    CHECK(Serial.output.find("J=")!=std::string::npos && Serial.baud==115200);
    CHECK(isRecording() && rx.size()==8); // Bounded 16-byte drain.
    loop(); CHECK(!isRecording() && rx.empty());
    // Timeout arithmetic across millis wrap.
    initializeButtonLink(); feed(ButtonWire::Hello,46,0,0);
    packet(ButtonWire::Press,46,1,2,invalid); simulatedTime=UINT32_MAX-3;
    for(uint8_t i=0;i<3;++i) feedButtonByte(invalid[i],simulatedTime++);
    simulatedTime+=10;
    for(uint8_t i=3;i<8;++i) feedButtonByte(invalid[i],simulatedTime++);
    CHECK(isRecording());
    CHECK(servoWrites.empty()); // Protocol tests never command an actual output.
    std::cout<<"PASS Nano protocol + real Uno PC loop: "<<checks<<" checks\n";
}
