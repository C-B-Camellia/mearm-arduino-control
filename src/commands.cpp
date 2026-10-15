#include "commands.h"
#include "command_text.h"
#include "arm.h"
#include "config.h"
#include "kinematics.h"
#include "motion.h"
#include "manual_control.h"
#include "hardware_config.h"
#include "control_settings.h"
#include "control_error.h"
#include "app_controller.h"
#include "recording.h"
#include "teaching.h"
#include "serial_format.h"
#include "pick_place.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

bool parseNumber(const char* text,float& value) {
    if(!text || !*text) return false;
    const char* p=text; if(*p=='+' || *p=='-') ++p;
    bool digit=false,dot=false;
    float number=0,fraction=0.1f;
    bool negative=*text=='-';
    while(*p) {
        if(*p>='0' && *p<='9') {
            digit=true;
            if(dot) { number+=(*p-'0')*fraction; fraction*=0.1f; }
            else number=number*10.0f+(*p-'0');
            if(!isfinite(number)) return false;
        }
        else if(*p=='.' && !dot) dot=true;
        else return false;
        ++p;
    }
    if(!digit) return false;
    value=negative?-number:number; return true;
}
bool parseNumbers(char* const* text,uint8_t count,float* values) {
    for(uint8_t i=0;i<count;++i) if(!parseNumber(text[i],values[i])) return false;
    return true;
}
namespace {
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialIndex=0;
bool discardLine=false;
void result(bool success) {
    if(!success) { printControlError(); return; }
    printText(F("OK"));
    // strtok splits in place; echo tokens directly, with no second input buffer.
    bool gap=true;
    for(uint8_t i=0;i<serialIndex;++i) {
        char c=serialBuffer[i];
        if(!c || c==' ' || c=='\t') { gap=true; continue; }
        if(gap) printChar(' ');
        gap=false; printChar(c);
    }
    printNewline();
}
void invalid() { rejectControl(ControlError::Invalid); printControlError(); }
void printIK(const IKResult& ik) {
    if(!ik.valid) { rejectControl(ControlError::Unreachable); printControlError(); return; }
    printText(F("IK J1=")); printDecimal(ik.j1); printText(F(" J2=")); printDecimal(ik.j2); printNewline();
}
void printStatus() {
    MotionStatus s=motionStatus();
    uint8_t enabled=0;
    printText(F("J=")); for(uint8_t i=0;i<JointCount;++i) { printDecimal(joints[i].commandedAngle,0); printChar(' '); if(joints[i].outputEnabled) enabled|=1<<i; }
    printText(F("autoSpeed=")); printDecimal(automaticSpeed(),1);
    printText(F(" manualScale=")); printDecimal(manualSpeedScale());
    printText(F(" mode=")); Serial.print(getControlMode()==ControlMode::Joint?F("JOINT"):F("CART"));
    printText(F(" task=")); printUnsigned(static_cast<int>(s.kind)); printText(F(" phase=")); printUnsigned(static_cast<int>(s.state));
    printText(F(" move=")); printUnsigned(s.completedWaypoints); printChar('/'); printUnsigned(s.totalWaypoints);
    printText(F(" held=")); printUnsigned(isArmHeld()); printText(F(" cal=")); printlnUnsigned(cartesianCalibrationValid());
    printText(F("enabled=")); printUnsigned(enabled); printText(F(" error=")); printlnUnsigned(static_cast<int>(lastControlError()));
    printApplicationStatus();
}
bool angleTriple(char* line) {
    Pose pose=commandedPose(); uint8_t seen=0; char* save=nullptr;
    for(char* field=strtok_r(line,",",&save);field;field=strtok_r(nullptr,",",&save)) {
        uint8_t id=field[0]=='x'?0:field[0]=='y'?1:field[0]=='z'?2:3;
        float value;
        if(id==3 || (seen&(1<<id)) || !parseNumber(field+1,value)) return rejectControl(ControlError::Invalid);
        seen|=1<<id; pose.angles[id]=value;
    }
    if(seen!=7) return rejectControl(ControlError::Invalid);
    if(!joints[Base].outputEnabled) return rejectControl(ControlError::Disabled);
    return startJointMove(pose);
}
void executeLine(char* line) {
    if(!*line) return;
    if(strchr(line,',')) {
        size_t length=strlen(line);
        if(line[0]==',' || line[length-1]==',' || strstr(line,",,")) { invalid(); return; }
        result(angleTriple(line)); return;
    }
    char* tokens[7]; uint8_t count=0; char* save=nullptr;
    for(char* token=strtok_r(line," \t",&save);token;token=strtok_r(nullptr," \t",&save)) {
        if(count==7) { invalid(); return; } tokens[count++]=token;
    }
    if(!count) return;
    bool appSuccess;
    if(executeAppCommand(tokens,count,appSuccess)) { result(appSuccess); return; }
    float v[3]={}; bool numbers=count>=2 && count<=4 && parseNumbers(tokens+1,count-1,v);
    const char* c=tokens[0];
    int op=commandIndex(c,commandNames("IK\0XY\0LINE\0RUNLINE\0XYZ\0IK3\0MODE\0SPEED\0H\0L\0O\0S\0HOLD\0GO\0CANCEL\0STATUS\0"));
    if(count==3 && numbers && (op==0 || op==1 || op==2 || op==3)) {
        if(op==0) printIK(solveIK(v[0],v[1]));
        else if(op==1) {
            IKResult ik=solveIK(v[0],v[1]); if(!ik.valid) { rejectControl(ControlError::Unreachable); printControlError(); return; }
            Pose p=commandedPose(); p.angles[Shoulder]=ik.j1; p.angles[Elbow]=ik.j2; result(startJointMove(p));
        } else result(startLine({v[0],v[1]},op==2?TaskKind::LinePreview:TaskKind::LineExecution));
        return;
    }
    if(count==4 && numbers && (op==4 || op==5)) {
        Point3D p={v[0],v[1],v[2]};
        if(op==4) result(startCartesianMove(p));
        else {
            if(!cartesianCalibrationValid()) { rejectControl(ControlError::Uncalibrated); printControlError(); return; }
            IKResult3D ik=solveIK3D(p);
            if(!ik.valid) { rejectControl(ControlError::Unreachable); printControlError(); }
            else { printText(F("IK3 ")); printDecimal(ik.j0); printChar(' '); printDecimal(ik.j1); printChar(' '); printDecimal(ik.j2); printNewline(); }
        }
        return;
    }
    if(count==2 && numbers && c[0]=='J' && c[1]>='0' && c[1]<='3' && c[2]==0) {
        uint8_t id=c[1]-'0';
        if(!joints[id].outputEnabled) { rejectControl(ControlError::Disabled); printControlError(); return; }
        Pose p=commandedPose(); p.angles[id]=v[0]; result(startJointMove(p)); return;

    }
    if(count==2 && op==6) {
        if(commandIs(tokens[1],"JOINT")) result(setControlMode(ControlMode::Joint));
        else if(commandIs(tokens[1],"CART")) result(setControlMode(ControlMode::Cartesian));
        else invalid();
        return;
    }
    if(count==2 && numbers && op==7) {
        if(isMotionBusy()) result(rejectControl(ControlError::Busy));
        else if(setAutomaticSpeed(v[0])) result(true);
        else invalid();
        return;
    }
    if(count!=1) { invalid(); return; }
    if(op==8 || op==9) { adjustManualSpeedScale(op==8); result(true); }
    else if(op==10 || op==11) {
        if(!gripperCalibrationValid()) result(rejectControl(ControlError::Uncalibrated));
        else { Pose p=commandedPose(); p.angles[Gripper]=op==10?hardwareConfig.gripOpen:kPenGripAngle; result(startJointMove(p)); }
    }
    else if(op==12) { if(!pauseMotion(millis())) holdArm(); result(true); }
    else if(op==13) { if(isPickSaving()) { result(rejectControl(ControlError::Busy)); return; } if(!resumeMotion(millis())) { resumeArm(); syncManualTarget(millis()); } result(true); }
    else if(op==14) {
        if(isRecording()) clearRecording();
        if(drawingTaskActive()) result(cancelAndReturnHome());
        else { cancelMotion(); result(true); }
    }
    else if(op==15) printStatus();
    else invalid();
}
}
// Release parser/diagnostic temporaries before the motion tick and its IK calls.
__attribute__((noinline)) void pollSerial() {
    for(uint8_t consumed=0;consumed<SERIAL_BYTES_PER_LOOP && Serial.available()>0;++consumed) {
        char c=static_cast<char>(Serial.read());
        if(c=='\r') continue;
        if(c=='\n') {
            if(!discardLine) { serialBuffer[serialIndex]=0; executeLine(serialBuffer); }
            serialIndex=0; discardLine=false; return;
        }
        if(discardLine) continue;
        if((c>=32 && c<=126) || c=='\t') {
            if(serialIndex<SERIAL_BUFFER_SIZE-1) serialBuffer[serialIndex++]=c;
            else { discardLine=true; printLine(F("ERR command too long")); }
        } else { discardLine=true; invalid(); }
    }
}
