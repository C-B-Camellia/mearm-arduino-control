#include "Arduino.h"
#include "EEPROM.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
uint32_t mockTime=0;
int levels[20],modes[20];
SerialMock Serial;
EepromMock EEPROM;
#ifdef MEARM_NANO_OLED
#include "Wire.h"
WireMock Wire;
#endif
#include "../src/main.cpp"
#include "../src/local_display.cpp"
namespace {
unsigned checks=0;
void check(bool value,int line) { ++checks; if(!value) { std::cerr<<"FAIL panel "<<line<<'\n'; std::exit(1); } }
#define CHECK(v) check((v),__LINE__)
void tick(uint32_t dt=5) { mockTime+=dt; loop(); }
void wait(uint32_t ms) { for(uint32_t i=0;i<ms;i+=5) tick(); }
void reboot() {
    mockTime=0; Serial.bytes.clear(); Serial.room=64;
#ifdef MEARM_NANO_OLED
    display.pages=display.refreshes=0; display.lines.clear();
#endif
    for(int& m:modes)m=-1;
    setup();wait(100);
}
std::vector<uint8_t> commands() {
    CHECK(Serial.bytes.size()%8==0); std::vector<uint8_t> out;
    uint8_t prior=0;
    for(size_t i=0;i<Serial.bytes.size();i+=8) {
        CHECK(ButtonWire::valid(&Serial.bytes[i]));
        if(Serial.bytes[i+1]==ButtonWire::Command) {
            CHECK(Serial.bytes[i+4]==static_cast<uint8_t>(prior+1));prior=Serial.bytes[i+4];
            out.push_back(Serial.bytes[i+5]);
        } else prior=Serial.bytes[i+4];
    }
    return out;
}
void released() { for(int& p:levels)p=HIGH; }
void press(uint8_t key,bool held=false) { levels[A0+key-1]=LOW;wait(held?950:60);levels[A0+key-1]=HIGH;wait(60); }
}
int main(int argc,char** argv) {
    // Full physical gesture -> encoded wire command matrix. Zero means local only/no-op.
    const uint8_t shortIds[5][4]={{1,2,3,4},{14,15,0,4},{6,7,8,10},{11,16,17,12},{18,19,20,0}};
    const uint8_t longIds[5][4]={{5,0,0,0},{0,0,0,0},{10,13,9,0},{10,13,0,0},{0,0,0,0}};
    const PanelMode N=PanelMode::Normal,B=PanelMode::Basic,C=PanelMode::Cal,T=PanelMode::Teach,D=PanelMode::Drawing;
    const PanelMode shortModes[5][4]={{N,N,N,N},{D,D,N,B},{C,C,C,N},{T,D,D,T},{D,D,D,D}};
    const PanelMode longModes[5][4]={{C,B,N,B},{B,N,B,C},{N,C,T,T},{N,C,T,D},{D,B,D,N}};
    for(uint8_t m=0;m<5;++m)for(uint8_t key=1;key<=4;++key)for(bool held:{false,true}) {
        released();reboot();selectMode(static_cast<PanelMode>(m));wait(100);
        press(key,held);auto c=commands();uint8_t want=held?longIds[m][key-1]:shortIds[m][key-1];
        CHECK(c.size()==(want?1u:0u));if(want)CHECK(c[0]==want);
        CHECK(mode==(held?longModes[m][key-1]:shortModes[m][key-1]));
    }
    // H4 in every page: hold >2s advances exactly once, no business frame or short on release.
    for(uint8_t m=0;m<5;++m) {
        released();reboot();selectMode(static_cast<PanelMode>(m));wait(100);
        uint8_t oldSequence=sequence;levels[A3]=LOW;wait(2600);
        CHECK(mode==static_cast<PanelMode>((m+1)%5));CHECK(commands().empty() && sequence==oldSequence);
#ifdef MEARM_NANO_OLED
        CHECK(localPage==static_cast<uint8_t>(mode));
        CHECK(renderedPage==localPage && renderedMode==localPage);
        const char* titles[]={"NORMAL","BASIC DRAW","HEIGHT CAL","TEACH","DRAWING"};
        CHECK(display.lines[display.lines.size()-7]==titles[(m+1)%5]);
#endif
        levels[A3]=HIGH;wait(60);CHECK(commands().empty() && sequence==oldSequence);
    }
    // Enter each page only via physical H4; check every resulting short mapping.
    // Repeat the full cycle back to NORMAL, because LINE/POLY themselves switch to DRAWING.
    for(uint8_t target=0;target<=5;++target)for(uint8_t key=1;key<=4;++key) {
        released();reboot();for(uint8_t i=0;i<target;++i)press(4,true);
        uint8_t m=target%5;CHECK(mode==static_cast<PanelMode>(m) && commands().empty());
#ifdef MEARM_NANO_OLED
        CHECK(localPage==m);
#endif
        press(key);auto c=commands();uint8_t want=shortIds[m][key-1];
        CHECK(c.size()==(want?1u:0u));if(want)CHECK(c[0]==want);
        CHECK(mode==shortModes[m][key-1]);
#ifdef MEARM_NANO_OLED
        CHECK(localPage==static_cast<uint8_t>(mode));
#endif
    }
    // Full transmit queue cannot prevent purely local H4 cycling or add a mode command.
    released();reboot();Serial.room=0;for(int i=0;i<7;++i)press(3);
    CHECK(((head-tail)&7)==7);press(4,true);CHECK(mode==B && ((head-tail)&7)==7);
    press(4,true);CHECK(mode==C && ((head-tail)&7)==7);
    Serial.room=64;wait(100);CHECK(commands()==std::vector<uint8_t>(7,3));
    // Every held-at-boot input is suppressed through release.
    for(int pin=A0;pin<=A3;++pin) {
        released();levels[pin]=LOW;reboot();wait(2500);CHECK(commands().empty() && mode==N);
        levels[pin]=HIGH;wait(60);CHECK(commands().empty());
    }
    released();reboot();CHECK(Serial.baud==9600);
    for(int i=A0;i<=A3;++i)CHECK(modes[i]==INPUT);
    for(int i=4;i<=7;++i)CHECK(modes[i]==-1);
    // B4 one-shot, no short on release, page and mapping advance together.
    levels[A3]=LOW;wait(40);mockTime=pressedAt[3]+799;loop();
#ifdef MEARM_NANO_OLED
    CHECK(localPage==0);
#endif
    mockTime=pressedAt[3]+800;loop();wait(2500);CHECK(mode==B && commands().empty());
#ifdef MEARM_NANO_OLED
    CHECK(localPage==1);
#endif
    levels[A3]=HIGH;wait(60);CHECK(commands().empty());
    // B4 entered BASIC: B1 now sends fixed LINE, then enters DRAWING.
    press(1);CHECK(commands().size()==1 && commands()[0]==14 && mode==D);
    press(2,true);CHECK(mode==B);
    press(4,true);CHECK(mode==C && commands().size()==1);
#ifdef MEARM_NANO_OLED
    CHECK(localPage==2);
#endif
    press(4,true);CHECK(mode==T && commands().size()==1);
    auto beforePage=commands().size();press(4,true);CHECK(mode==D && commands().size()==beforePage);
    press(4,true);CHECK(mode==N && commands().size()==beforePage);
    // Local drawing recovery has no frame; keep original TEACH/BASIC return mapping.
    selectMode(T);press(3);CHECK(mode==D);auto n=commands().size();press(2,true);CHECK(mode==T && commands().size()==n);
    selectMode(B);press(1);CHECK(mode==D);n=commands().size();press(2);CHECK(commands().size()==n+1 && commands().back()==19 && mode==D);
    n=commands().size();press(2,true);CHECK(mode==B && commands().size()==n);
    // Short is emitted only after release for a key with a long operation.
    released();reboot();levels[A0]=LOW;wait(60);CHECK(commands().empty());levels[A0]=HIGH;wait(60);CHECK(commands().back()==1);
    // 799ms release remains short; 800ms release is long even if the loop was late.
    released();reboot();levels[A3]=LOW;wait(40);mockTime=pressedAt[3]+799;levels[A3]=HIGH;loop();wait(40);CHECK(commands().size()==1 && commands()[0]==4);
    levels[A3]=LOW;wait(40);mockTime=pressedAt[3]+800;levels[A3]=HIGH;loop();wait(40);CHECK(commands().size()==1);
    // Debounce and millis rollover.
    released();reboot();levels[A0]=LOW;tick();levels[A0]=HIGH;tick();levels[A0]=LOW;wait(40);
    levels[A0]=HIGH;tick();levels[A0]=LOW;tick();levels[A0]=HIGH;wait(40);CHECK(commands().size()==1);
    mockTime=0xFFFFFF00u;levels[A3]=LOW;wait(900);levels[A3]=HIGH;wait(50);CHECK(commands().size()==1);
    // Queue overflow must not pretend a mode-changing command was queued.
    released();reboot();Serial.room=0;for(int i=0;i<8;++i)press(3);CHECK(overflowed && ((head-tail)&7)==7);
    press(1,true);CHECK(mode==N);Serial.room=64;wait(100);CHECK(commands().size()==7);
    // HELLO/epoch and command sequence wrap retain the original framing.
    uint16_t before=epoch;released();reboot();CHECK(epoch!=before && commands().empty());
    sequence=255;helloPending=true;tick();press(3);CHECK(commands().size()==1 && sequence==0);
#ifdef MEARM_NANO_OLED
    CHECK(display.address==0x78 && display.clock==400000 && Wire.timeout==2500);
    unsigned refreshes=display.refreshes;wait(1000);CHECK(display.refreshes==refreshes);
    for(int i=0;i<5;++i) { press(4,true);wait(100); }
    for(const auto& line:display.lines) {
        CHECK(line.size()<=21);
        CHECK(line.find("READY")==std::string::npos && line.find("SUCCESS")==std::string::npos && line.find("DONE")==std::string::npos);
    }
    // The mocked OLED consumes 3ms per band; DRAWING short actions still dispatch on release.
    selectMode(D);wait(100);n=commands().size();selectLocalDisplayPage(D);levels[A0]=LOW;wait(40);
    CHECK(commands().size()==n);levels[A0]=HIGH;wait(40);CHECK(commands().size()==n+1 && commands().back()==18);
    levels[A2]=LOW;wait(40);CHECK(commands().back()==18);levels[A2]=HIGH;wait(40);CHECK(commands().back()==20);
    released();reboot();CHECK(localPage==0 && mode==N);
#endif
    // Produce real sender bytes for the Uno cross-board integration test.
    if(argc==2) {
        released();reboot();press(1,true);press(1);press(2);press(3);press(3,true);
        for(int i=0;i<5;++i)press(1);
        press(4);press(1);press(2);press(1);press(2);press(3);
        press(2,true);press(2,true);press(4);
        CHECK(mode==N);
        const std::vector<uint8_t> expected={5,6,7,8,9,11,11,11,11,11,12,11,16,18,19,20,13,10};
        CHECK(commands()==expected);
        std::ofstream out(argv[1],std::ios::binary);out.write(reinterpret_cast<const char*>(Serial.bytes.data()),Serial.bytes.size());CHECK(bool(out));
    }
    std::cout<<"PASS panel business gestures + wire + OLED: "<<checks<<" checks\n";
}
