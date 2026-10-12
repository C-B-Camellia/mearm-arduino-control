#include "Arduino.h"
#include "EEPROM.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
uint32_t mockTime=0;
int levels[16],modes[16];
SerialMock Serial;
EepromMock EEPROM;
#ifdef MEARM_NANO_OLED
#include "Wire.h"
WireMock Wire;
#endif
// Compile the real firmware loop. Only hardware peripherals are substituted.
#include "../src/main.cpp"
#include "../src/local_display.cpp"
namespace {
int checks=0;
void check(bool value,int line) { ++checks; if(!value) { std::cerr<<"FAIL nano "<<line<<'\n'; std::exit(1); } }
#define CHECK(v) check((v),__LINE__)
void tick(uint32_t dt=5) { mockTime+=dt; loop(); }
void wait(uint32_t ms) { for(uint32_t i=0;i<ms;i+=5) tick(); }
void reboot() {
    head=tail=lastButton=sequence=0; helloAt=0; helloPending=true; overflowed=false;
    mockTime=0; Serial.bytes.clear(); Serial.room=64;
#ifdef MEARM_NANO_OLED
    paging=false; refreshedAt=0; display.pages=display.refreshes=0; display.lines.clear();
#endif
    setup();
}
int presses() {
    CHECK(Serial.bytes.size()%8==0); int n=0;
    for(size_t i=0;i<Serial.bytes.size();i+=8) {
        CHECK(ButtonWire::valid(&Serial.bytes[i])); n+=Serial.bytes[i+1]==ButtonWire::Press;
    }
    return n;
}
}
int main() {
    for(int& pin:levels) pin=HIGH;
    levels[5]=LOW; reboot(); wait(1000);
    CHECK(Serial.baud==9600 && presses()==0);
    for(int i=4;i<=7;++i) CHECK(modes[i]==INPUT_PULLUP);
    levels[5]=HIGH; wait(50);
    levels[5]=LOW; tick(); levels[5]=HIGH; tick(); levels[5]=LOW; tick();
    wait(50); CHECK(presses()==1); wait(3000); CHECK(presses()==1);
    levels[5]=HIGH; wait(50); levels[5]=LOW; wait(50); CHECK(presses()==2);
    // The real transmitter emits one valid frame per simultaneous press.
    for(int& pin:levels) { pin=HIGH; }
    reboot(); tick();
    for(int i=4;i<=7;++i) levels[i]=LOW;
    wait(100); CHECK(presses()==4);
    for(size_t i=8;i<40;i+=8) CHECK(Serial.bytes[i+5]==(i/8));
    uint16_t oldEpoch=epoch;
    reboot(); wait(1000); CHECK(epoch!=oldEpoch && presses()==0); // held on Nano reset
    for(int& pin:levels) pin=HIGH;
    reboot(); tick(); Serial.room=0;
    for(int round=0;round<3;++round) {
        for(int i=4;i<=7;++i) { levels[i]=LOW; }
        wait(50);
        for(int i=4;i<=7;++i) { levels[i]=HIGH; }
        wait(50);
    }
    CHECK(overflowed && presses()==0 && ((head-tail)&7)==7);
    Serial.room=64; wait(100); CHECK(presses()==7); // bounded queue, no invented events
    for(int& pin:levels) pin=HIGH;
    reboot(); tick(); sequence=255;
    levels[4]=LOW; wait(50); CHECK(Serial.bytes[8+4]==0 && presses()==1);
#ifdef MEARM_NANO_OLED
    CHECK(display.address==0x78 && display.clock==400000 && Wire.timeout==2500);
    wait(1000); CHECK(display.pages>=8 && display.refreshes>0);
    bool linkWait=false,authority=false,accepted=false;
    for(const std::string& line:display.lines) {
        CHECK(line.size()<=21);
        linkWait|=line=="LINK WAIT (one-way)";
        accepted|=line=="TX != accepted";
        authority|=line.find("REC START")!=std::string::npos || line.find("HOME DONE")!=std::string::npos;
    }
    CHECK(linkWait && accepted && !authority);
    // Scanning continues while individual mocked I2C pages consume 3 ms.
    int before=presses(); levels[4]=HIGH; wait(60); levels[4]=LOW; wait(60);
    CHECK(presses()==before+1);
#endif
    std::cout<<"PASS Nano firmware loop: "<<checks<<" checks\n";
}
