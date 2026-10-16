#include "serial_format.h"
#include "pick_place.h"
#include "hardware_config.h"
#include "motion.h"
#include "recording.h"
#include "control_error.h"
#include "command_text.h"
#include "commands.h"
#include <EEPROM.h>
#ifdef __AVR__
#include <avr/eeprom.h>
#endif
namespace {
// 3 masks + 24 stages (four integer angles, deg/s, 10 ms dwell units).
// Two committed slots and an unpublished edit area; no program RAM copy.
constexpr uint8_t PROGRAM_BYTES=49, PAYLOAD_BYTES=147, HEADER_BYTES=8;
constexpr uint16_t SLOT_BYTES=155, WORK=310;
constexpr uint8_t VERSION=1, COMMITTED=0xA5;
static_assert(WORK+PAYLOAD_BYTES<=1024,"pick EEPROM exceeds ATmega328P");
static __attribute__((noinline)) uint8_t readByte(uint16_t at) { return EEPROM.read(at); }
static __attribute__((noinline)) void writeByte(uint16_t at,uint8_t value) { EEPROM.update(at,value); }
bool saving=false;
uint8_t activeSlot=255,writeSlot=0,writeStep=0;
uint16_t generation=0,writeCrc=0;
uint16_t crcByte(uint16_t crc,uint8_t value) {
    crc^=value;
    for(uint8_t i=0;i<8;++i) crc=(crc>>1)^((crc&1)?0xA001:0);
    return crc;
}
uint16_t address(uint8_t object,uint8_t stage) { return WORK+object*PROGRAM_BYTES+1+stage*6; }
uint16_t readWord(uint16_t at) { return readByte(at)|(static_cast<uint16_t>(readByte(at+1))<<8); }
bool validSlot(uint8_t slot) {
    uint16_t base=slot*SLOT_BYTES;
    if(readByte(base)!=0x4D || readByte(base+1)!=VERSION ||
       readByte(base+6)!=PAYLOAD_BYTES || readByte(base+7)!=COMMITTED) return false;
    uint16_t crc=crcByte(crcByte(0xFFFF,readByte(base+2)),readByte(base+3));
    for(uint8_t i=0;i<PAYLOAD_BYTES;++i) {
        uint8_t value=readByte(base+HEADER_BYTES+i);
        crc=crcByte(crc,value);
        uint8_t field=i%PROGRAM_BYTES;
        if(field && (readByte(base+HEADER_BYTES+(i/PROGRAM_BYTES)*PROGRAM_BYTES)&(1<<((field-1)/6)))) {
            uint8_t member=(field-1)%6;
            if((member<4 && value>180) || (member==4 && (value<5 || value>90))) return false;
        }
    }
    return crc==readWord(base+4);
}
bool editable() {
    if(saving || isMotionBusy() || isRecording() || hasPendingPose()) return rejectControl(ControlError::Busy);
    return true;
}
void resetWorking() { for(uint8_t i=0;i<3;++i) writeByte(WORK+i*PROGRAM_BYTES,0); }
int stageNumber(const char* text) {
    if(text[0]>='1' && text[0]<='8' && text[1]==0) return text[0]-'1';
    return commandIndex(text,commandNames("APPROACH\0SRC\0GRIP\0LIFT\0TRANSFER\0DST\0RELEASE\0RETREAT\0"));
}
bool capture(uint8_t object,uint8_t stage,const Pose& pose) {
    Pose quantized;
    if(!prepareOutputPose(pose,quantized)) return rejectControl(ControlError::Unsafe);
    uint16_t at=address(object,stage),maskAt=WORK+object*PROGRAM_BYTES;
    uint8_t mask=readByte(maskAt);
    writeByte(maskAt,mask&~(1<<stage));
    for(uint8_t i=0;i<JointCount;++i) writeByte(at+i,static_cast<uint8_t>(quantized.angles[i]));
    writeByte(at+4,60); writeByte(at+5,10);
    writeByte(maskAt,mask|(1<<stage));
    return true;
}
}
bool isPickSaving() { return saving; }
uint8_t pickStageMask(uint8_t object) { return object<3?readByte(WORK+object*PROGRAM_BYTES):0; }
static __attribute__((noinline)) bool restore() {
    bool a=validSlot(0),b=validSlot(1);
    if(!a && !b) return rejectControl(ControlError::NoData);
    uint16_t ga=readWord(2),gb=readWord(SLOT_BYTES+2);
    activeSlot=b && (!a || static_cast<int16_t>(gb-ga)>0)?1:0;
    generation=activeSlot?gb:ga;
    for(uint8_t i=0;i<PAYLOAD_BYTES;++i) writeByte(WORK+i,readByte(activeSlot*SLOT_BYTES+HEADER_BYTES+i));
    return true;
}
bool loadPickConfiguration() { if(!editable()) return false; holdArm(); return restore(); }
void initializePickConfiguration() {
    saving=false; activeSlot=255; generation=0;
    // Never activate interrupted or unsaved edits after reset.
    if(!restore()) resetWorking();
    clearControlError();
}
bool savePickConfiguration() {
    if(!editable()) return false;
    holdArm(); // Persistence is an explicit stationary operation.
    writeSlot=activeSlot==0?1:0; ++generation;
    writeStep=0; writeCrc=crcByte(crcByte(0xFFFF,generation&255),generation>>8); saving=true;
    return true;
}
void updatePickPersistence() {
    if(!saving) return;
#ifdef __AVR__
    if(!eeprom_is_ready()) return;
#endif
    if(writeStep==9+PAYLOAD_BYTES) { activeSlot=writeSlot; saving=false; printLine(F("PICK SAVED; HELD")); return; }
    uint16_t base=writeSlot*SLOT_BYTES;
    uint8_t value;
    uint16_t at;
    if(writeStep==0) { at=base+7; value=0; }
    else if(writeStep<=4) {
        at=base+writeStep-1;
        value=writeStep==1?0x4D:writeStep==2?VERSION:writeStep==3?generation&255:generation>>8;
    }
    else if(writeStep==5) { at=base+6; value=PAYLOAD_BYTES; }
    else if(writeStep<6+PAYLOAD_BYTES) {
        uint8_t index=writeStep-6;
        at=base+HEADER_BYTES+index; value=readByte(WORK+index); writeCrc=crcByte(writeCrc,value);
    }
    else if(writeStep<8+PAYLOAD_BYTES) {
        at=base+4+writeStep-(6+PAYLOAD_BYTES);
        value=writeStep==6+PAYLOAD_BYTES?writeCrc&255:writeCrc>>8;
    }
    else { at=base+7; value=COMMITTED; }
    writeByte(at,value);
    ++writeStep;
}
bool pickCalibrationValid(uint8_t object) {
    return pickStageMask(object)==255 && gripperCalibrationValid() && joints[Base].outputEnabled;
}
uint8_t pickStageSpeed(uint8_t object,uint8_t stage) { return readByte(address(object,stage)+4); }
uint16_t pickStageDwell(uint8_t object,uint8_t stage) { return static_cast<uint16_t>(readByte(address(object,stage)+5))*10; }
bool getPickStage(uint8_t object,uint8_t stage,Pose& pose,uint16_t& dwell) {
    if(stage>=PICK_STAGE_COUNT || !pickCalibrationValid(object)) return false;
    uint8_t grip=readByte(address(object,stage)+3);
    uint8_t speed=pickStageSpeed(object,stage);
    // Every taught stage owns all four axes; path/output validation stays in motion.
    if(speed<5 || speed>90) return false;
    for(uint8_t i=0;i<Gripper;++i) pose.angles[i]=readByte(address(object,stage)+i);
    pose.angles[Gripper]=grip;
    dwell=pickStageDwell(object,stage); return true;
}
__attribute__((noinline)) bool executePickCommand(char* const* t,uint8_t count) {
    if(count<2) return rejectControl(ControlError::Invalid);
    int op=commandIndex(t[1],commandNames("SET\0SHOW\0SAVE\0LOAD\0CLEAR\0SPEED\0WAIT\0"));
    if(count==2 && op==2) return savePickConfiguration();
    if(count==2 && op==3) return loadPickConfiguration();
    if(count<3 || t[2][1]!=0 || t[2][0]<'A' || t[2][0]>'C') return rejectControl(ControlError::Invalid);
    uint8_t object=t[2][0]-'A';
    if(count==3 && op==1) {
        printText(F("PICK ")); Serial.print(t[2]); printText(F(" mask=")); printlnUnsigned(pickStageMask(object));
        for(uint8_t stage=0;stage<8;++stage) if(pickStageMask(object)&(1<<stage)) {
            printUnsigned(stage+1);
            for(uint8_t i=0;i<6;++i) { printChar(' '); uint16_t v=readByte(address(object,stage)+i); printUnsigned(i==5?v*10:v); }
            printNewline();
        }
        return true;
    }
    if(!editable()) return false;
    if(count==3 && op==4) { writeByte(WORK+object*PROGRAM_BYTES,0); return true; }
    if(count<4) return rejectControl(ControlError::Invalid);
    int stage=stageNumber(t[3]);
    if(stage<0) return rejectControl(ControlError::Invalid);
    if(count==4 && op==0) {
        if(!gripperCalibrationValid() || !joints[Base].outputEnabled) return rejectControl(ControlError::Uncalibrated);
        return capture(object,stage,commandedPose());
    }
    if(count==5 && (op==5 || op==6)) {
        uint16_t v=0; const char* p=t[4];
        if(!*p || !(pickStageMask(object)&(1<<stage))) return rejectControl(ControlError::Invalid);
        while(*p) {
            if(*p<'0' || *p>'9' || v>255) return rejectControl(ControlError::Invalid);
            v=v*10+(*p++-'0');
        }
        if(v>2550) return rejectControl(ControlError::Invalid);
        if(op==5?(v<5 || v>90):(v%10!=0)) return rejectControl(ControlError::Invalid);
        writeByte(address(object,stage)+(op==5?4:5),op==5?v:v/10); return true;
    }
    return rejectControl(ControlError::Invalid);
}
#ifdef MEARM_NATIVE_TEST
void setSyntheticPickProgram(uint8_t object,const Pose (&poses)[PICK_STAGE_COUNT]) {
    if(object>=3) return;
    writeByte(WORK+object*PROGRAM_BYTES,0);
    for(uint8_t i=0;i<PICK_STAGE_COUNT;++i) if(!capture(object,i,poses[i])) { writeByte(WORK+object*PROGRAM_BYTES,0); return; }
}
void clearSyntheticPickPrograms() { EEPROM.reset(); }
#endif
