#include "commands.h"
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
#include <math.h>
#include <stdlib.h>
#include <string.h>

bool parseNumber(const char* text,float& value) {
    if(!text || !*text) return false;
    const char* p=text; if(*p=='+' || *p=='-') ++p;
    bool digit=false,dot=false;
    while(*p) {
        if(*p>='0' && *p<='9') digit=true;
        else if(*p=='.' && !dot) dot=true;
        else return false;
        ++p;
    }
    if(!digit) return false;
    value=atof(text); return isfinite(value);
}
namespace {
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialIndex=0;
bool discardLine=false;
void result(bool success) { if(success) Serial.println(F("OK")); else printControlError(); }
void invalid() { rejectControl(ControlError::Invalid); printControlError(); }
void printIK(const IKResult& ik) {
    if(!ik.valid) { rejectControl(ControlError::Unreachable); printControlError(); return; }
    Serial.print(F("IK J1=")); Serial.print(ik.j1,2); Serial.print(F(" J2=")); Serial.println(ik.j2,2);
}
void printStatus() {
    Pose p=commandedPose(); MotionStatus s=motionStatus();
    Serial.print(F("J=")); for(uint8_t i=0;i<JointCount;++i) { Serial.print(p.angles[i],0); Serial.print(' '); }
    Serial.print(F("autoSpeed=")); Serial.print(automaticSpeed(),1);
    Serial.print(F(" manualScale=")); Serial.print(manualSpeedScale(),2);
    Serial.print(F(" mode=")); Serial.print(getControlMode()==ControlMode::Joint?F("JOINT"):F("CART"));
    Serial.print(F(" task=")); Serial.print(static_cast<int>(s.kind)); Serial.print(F(" phase=")); Serial.print(static_cast<int>(s.state));
    Serial.print(F(" move=")); Serial.print(s.completedWaypoints); Serial.print('/'); Serial.print(s.totalWaypoints);
    Serial.print(F(" held=")); Serial.print(isArmHeld()); Serial.print(F(" cal=")); Serial.println(cartesianCalibrationValid());
    printApplicationStatus();
}
void printHelp() {
    Serial.println(F("IK/XY/LINE/RUNLINE x z; J0..J3 deg; XYZ/IK3 x y z"));
    Serial.println(F("x90,y120,z95 = J0,J1,J2 angles (atomic)"));
    Serial.println(F("MODE JOINT|CART; O/S; H/L joystick scale 0.75..1.5"));
    Serial.println(F("SPEED 5..90 automatic deg/s; HOLD/GO/CANCEL; STATUS/HELP"));
    Serial.println(F("A/B/C; REC [START|STOP]; PLAY; HOME; BTN 1..4"));
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
    float v[3]={}; bool numbers=count>=2 && count<=4;
    for(uint8_t i=1;i<count && i<=3;++i) if(!parseNumber(tokens[i],v[i-1])) numbers=false;
    const char* c=tokens[0];
    float shortcut;
    if(count==2 && parseNumber(c,shortcut) && numbers) { printIK(solveIK(shortcut,v[0])); return; }
    if(count==3 && numbers && (!strcmp(c,"IK") || !strcmp(c,"XY") || !strcmp(c,"LINE") || !strcmp(c,"RUNLINE"))) {
        if(!strcmp(c,"IK")) printIK(solveIK(v[0],v[1]));
        else if(!strcmp(c,"XY")) {
            IKResult ik=solveIK(v[0],v[1]); if(!ik.valid) { rejectControl(ControlError::Unreachable); printControlError(); return; }
            Pose p=commandedPose(); p.angles[Shoulder]=ik.j1; p.angles[Elbow]=ik.j2; result(startJointMove(p));
        } else result(startLine({v[0],v[1]},!strcmp(c,"LINE")?TaskKind::LinePreview:TaskKind::LineExecution));
        return;
    }
    if(count==4 && numbers && (!strcmp(c,"XYZ") || !strcmp(c,"IK3"))) {
        Point3D p={v[0],v[1],v[2]};
        if(!strcmp(c,"XYZ")) result(startCartesianMove(p));
        else {
            if(!cartesianCalibrationValid()) { rejectControl(ControlError::Uncalibrated); printControlError(); return; }
            IKResult3D ik=solveIK3D(p);
            if(!ik.valid) { rejectControl(ControlError::Unreachable); printControlError(); }
            else { Serial.print(F("IK3 ")); Serial.print(ik.j0,2); Serial.print(' '); Serial.print(ik.j1,2); Serial.print(' '); Serial.println(ik.j2,2); }
        }
        return;
    }
    if(count==2 && numbers && c[0]=='J' && c[1]>='0' && c[1]<='3' && c[2]==0) {
        uint8_t id=c[1]-'0';
        if(!joints[id].outputEnabled) { rejectControl(ControlError::Disabled); printControlError(); return; }
        Pose p=commandedPose(); p.angles[id]=v[0]; result(startJointMove(p)); return;
    }
    if(count==2 && !strcmp(c,"MODE")) {
        if(!strcmp(tokens[1],"JOINT")) result(setControlMode(ControlMode::Joint));
        else if(!strcmp(tokens[1],"CART")) result(setControlMode(ControlMode::Cartesian));
        else invalid();
        return;
    }
    if(count==2 && numbers && !strcmp(c,"SPEED")) {
        if(isMotionBusy()) result(rejectControl(ControlError::Busy));
        else if(setAutomaticSpeed(v[0])) result(true);
        else invalid();
        return;
    }
    if(count!=1) { invalid(); return; }
    if(!strcmp(c,"H") || !strcmp(c,"L")) { adjustManualSpeedScale(!strcmp(c,"H")); result(true); }
    else if(!strcmp(c,"O") || !strcmp(c,"S")) {
        if(!gripperCalibrationValid()) result(rejectControl(ControlError::Uncalibrated));
        else { Pose p=commandedPose(); p.angles[Gripper]=!strcmp(c,"O")?hardwareConfig.gripOpen:hardwareConfig.gripClosed; result(startJointMove(p)); }
    }
    else if(!strcmp(c,"HOLD")) { if(!pauseMotion(millis())) holdArm(); result(true); }
    else if(!strcmp(c,"GO")) { if(!resumeMotion(millis())) { resumeArm(); syncManualTarget(millis()); } result(true); }
    else if(!strcmp(c,"CANCEL")) { if(isRecording()) clearRecording(); cancelMotion(); result(true); }
    else if(!strcmp(c,"STATUS")) printStatus();
    else if(!strcmp(c,"HELP")) printHelp();
    else invalid();
}
}
void pollSerial() {
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
            else { discardLine=true; Serial.println(F("ERR command too long")); }
        }
    }
}
