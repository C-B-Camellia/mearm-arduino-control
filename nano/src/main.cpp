#include <Arduino.h>
#include <EEPROM.h>
#include "button_scan.h"
#include "local_display.h"
#include "button_protocol.h"
#include "business_commands.h"
namespace {
DebouncedButton buttons[4];
uint16_t epoch=0;
uint8_t sequence=0,queue[8],head=0,tail=0;
uint32_t helloAt=0;
bool helloPending=true,overflowed=false;
PanelMode mode=PanelMode::Normal,returnMode=PanelMode::Basic;
uint32_t pressedAt[4];
PanelMode pressedMode[4];
uint8_t pending=0;
bool enqueue(AppAction action) {
    uint8_t next=(head+1)&7;
    if(next==tail) { overflowed=true; return false; }
    queue[head]=static_cast<uint8_t>(action); head=next; return true;
}
void selectMode(PanelMode next) { mode=next; selectLocalDisplayPage(next); }
void gesture(uint8_t key,bool held,PanelMode m) {
    // A key pressed before a local mode transition cannot execute in the new mode.
    if(m!=mode) return;
    if(held && key==4) {
        PanelMode next=static_cast<PanelMode>((static_cast<uint8_t>(mode)+1)%5);
        // H4 selects local page/mapping only; Uno state is never inferred here.
        if(next==PanelMode::Drawing) returnMode=PanelMode::Teach;
        selectMode(next); return;
    }
    if(held) {
        if(key==1) {
            if(m==PanelMode::Normal) { if(enqueue(AppAction::TeachStart)) selectMode(PanelMode::Cal); }
            else if(m==PanelMode::Cal || m==PanelMode::Teach) { if(enqueue(AppAction::TeachExit)) selectMode(PanelMode::Normal); }
        } else if(key==2) {
            if(m==PanelMode::Normal) selectMode(PanelMode::Basic);
            else if(m==PanelMode::Basic) selectMode(PanelMode::Normal);
            else if(m==PanelMode::Drawing) selectMode(returnMode); // PC-confirmed completion only; no wire command.
            else if(enqueue(AppAction::Clear)) selectMode(PanelMode::Cal);
        } else if(key==3 && m==PanelMode::Cal) {
            if(enqueue(AppAction::Marking)) selectMode(PanelMode::Teach); // Request sent, not READY acknowledgement.
        }
        return;
    }
    switch(m) {
    case PanelMode::Normal: {
        const AppAction actions[]={AppAction::PickNext,AppAction::RecordToggle,AppAction::Play,AppAction::Home};
        enqueue(actions[key-1]); break;
    }
    case PanelMode::Basic:
        if(key<=2) {
            if(enqueue(key==1?AppAction::DrawLine:AppAction::DrawShape)) { returnMode=m; selectMode(PanelMode::Drawing); }
        } else if(key==3) selectMode(PanelMode::Normal);
        else enqueue(AppAction::Home);
        break;
    case PanelMode::Cal: {
        const AppAction actions[]={AppAction::DrawHeight,AppAction::TeachAlignment,AppAction::TravelHeight,AppAction::TeachExit};
        if(enqueue(actions[key-1]) && key==4) selectMode(PanelMode::Normal);
        break;
    }
    case PanelMode::Teach:
        if(key==1) enqueue(AppAction::Mark);
        else if(key==4) enqueue(AppAction::Undo);
        else if(enqueue(key==2?AppAction::Polyline:AppAction::Curve)) { returnMode=m; selectMode(PanelMode::Drawing); }
        break;
    case PanelMode::Drawing:
        if(key==1) enqueue(AppAction::Hold);
        else if(key==2) enqueue(AppAction::Go);
        else if(key==3) enqueue(AppAction::Cancel);
        break;
    }
}
void sendFrame(uint8_t type,uint8_t command) {
    uint8_t bytes[ButtonWire::Size];
    ButtonWire::encode(bytes,type,epoch,sequence,command);
    Serial.write(bytes,sizeof(bytes));
}
}
void setup() {
    Serial.begin(9600);
    epoch=EEPROM.read(0)|(static_cast<uint16_t>(EEPROM.read(1))<<8);
    ++epoch; EEPROM.update(0,epoch); EEPROM.update(1,epoch>>8);
    head=tail=sequence=pending=0; helloAt=0; helloPending=true; overflowed=false;
    mode=PanelMode::Normal; returnMode=PanelMode::Basic;
    for(uint8_t i=0;i<4;++i) { pinMode(A0+i,INPUT); buttons[i].initialize(digitalRead(A0+i)==LOW,millis()); }
    initializeLocalDisplay();
}
void loop() {
    uint32_t now=millis();
    for(uint8_t i=0;i<4;++i) {
        if(buttons[i].observe(digitalRead(A0+i)==LOW,now)) {
            pressedMode[i]=mode; pressedAt[i]=now;
            // Every key waits for release/800ms so an unassigned hold cannot fire a short action.
            pending|=1<<i;
        }
        if(pending&(1<<i)) {
            if(!buttons[i].stable) {
                pending&=~(1<<i);
                gesture(i+1,buttons[i].changed-pressedAt[i]>=800,pressedMode[i]);
            } else if(buttons[i].raw && now-pressedAt[i]>=800) {
                pending&=~(1<<i); gesture(i+1,true,pressedMode[i]);
            }
        }
    }
    if(Serial.availableForWrite()>=ButtonWire::Size) {
        if(helloPending || (head==tail && now-helloAt>=1000)) {
            sendFrame(ButtonWire::Hello,0); helloAt=now; helloPending=false;
        } else if(head!=tail) {
            uint8_t command=queue[tail]; tail=(tail+1)&7; ++sequence;
            sendFrame(ButtonWire::Command,command);
        }
    }
    updateLocalDisplay(now,mode,overflowed);
}
