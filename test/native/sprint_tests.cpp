#include "Arduino.h"
#include "EEPROM.h"
#include "arm.h"
#include "commands.h"
#include "config.h"
#include "joystick.h"
#include "kinematics.h"
#include "manual_control.h"
#include "motion.h"
#include "hardware_config.h"
#include "control_settings.h"
#include "control_error.h"
#include "recording.h"
#include "pick_place.h"
#include "app_controller.h"
#include "button_link.h"
#include "teaching.h"
#include "path_planner.h"
#include "serial_format.h"
#include <cstdlib>
#include <iostream>
#include <limits>
#ifdef LEGACY_REFERENCE
#include "legacy_reference.h"
#endif

uint32_t simulatedTime = 0;
std::vector<ServoWrite> servoWrites;
std::vector<int> attachedPins;
std::vector<uint32_t> delayCalls;
int analogValues[4] = {506, 519, 515, 519};
SerialStub Serial;
EEPROMStub EEPROM;
void setup();
void loop();
namespace {
int assertions = 0;
JointControl original[JointCount];
void check(bool ok, const char* expression, int line) {
    ++assertions;
    if (!ok) { std::cerr << "FAIL " << line << ": " << expression << '\n'; std::exit(1); }
}
#define CHECK(e) check((e), #e, __LINE__)
bool contains(const char* s) { return Serial.output.find(s) != std::string::npos; }
void advance(uint32_t ms = CONTROL_INTERVAL_MS) { simulatedTime += ms; loop(); }
void runFor(uint32_t ms) { for (uint32_t t=0; t<ms; t+=CONTROL_INTERVAL_MS) advance(); }
void send(const std::string& s) {
    Serial.input += s + '\n';
    for (int i=0; !Serial.input.empty() && i<20; ++i) loop();
    CHECK(Serial.input.empty());
}
void fresh() {
    Serial.input="\n"; pollSerial(); cancelMotion();
    for (uint8_t i=0;i<JointCount;++i) joints[i]=original[i];
    hardwareConfig=HardwareConfig{}; resetControlSettings(); clearSyntheticPickPrograms();
    analogValues[0]=506; analogValues[1]=519; analogValues[2]=515; analogValues[3]=519;
    simulatedTime=0; Serial.printCostMs=0; servoWrites.clear(); attachedPins.clear(); delayCalls.clear();
    Serial.input.clear(); Serial.output.clear(); setup(); advance(); advance();
    CHECK(attachedPins.size()==2 && attachedPins[0]==8 && attachedPins[1]==7);
    CHECK(delayCalls.size()==1 && delayCalls[0]==STARTUP_DELAY_MS);
    servoWrites.clear(); Serial.output.clear();
}
void finishTask() {
    for (int i=0; isMotionBusy() && i<20000; ++i) advance();
    CHECK(!isMotionBusy());
}
void checkWritesSafe() {
    CHECK(servoWrites.size()%2==0);
    for (size_t i=0;i<servoWrites.size();i+=2) {
        CHECK(servoWrites[i].pin==8 && servoWrites[i+1].pin==7);
        CHECK(isSafePlanarPose(servoWrites[i].angle,servoWrites[i+1].angle));
    }
}
void testSafety() {
    fresh();
    Pose before=commandedPose(), candidate=before, output{};
    candidate.angles[Shoulder]=20.9f; candidate.angles[Elbow]=127.1f;
    CHECK(isSafePose(candidate));
    CHECK(prepareOutputPose(candidate,output)); CHECK(isSafePose(output));
    CHECK(output.angles[Shoulder]+output.angles[Elbow]==148);
    CHECK(servoWrites.empty() && !hasPendingPose());
    CHECK(commandedPose().angles[Shoulder]==before.angles[Shoulder]);
    candidate.angles[Shoulder]=20; candidate.angles[Elbow]=128;
    CHECK(prepareOutputPose(candidate,output)); CHECK(requestPose(candidate)); CHECK(applyPendingPose()); checkWritesSafe();
    candidate=commandedPose(); candidate.angles[Base]=91; CHECK(!requestPose(candidate));
    candidate=commandedPose(); candidate.angles[Shoulder]=std::numeric_limits<float>::quiet_NaN();
    CHECK(!requestPose(candidate));
    CHECK(!isSafePlanarPose(19.9f,130)); CHECK(!isSafePlanarPose(125.1f,95));
    CHECK(!isSafePlanarPose(120,39.9f)); CHECK(!isSafePlanarPose(120,130.1f));
    fresh(); candidate=commandedPose(); candidate.angles[Shoulder]=119.9f;
    CHECK(requestPose(candidate)); advance(); CHECK(commandedPose().angles[Shoulder]==119);
}
void testMath() {
    fresh();
    for(int j1=20;j1<=125;j1+=3) for(int j2=40;j2<=130;j2+=3) {
        // Exact hardware boundaries can round outward in the unchanged 2D inverse.
        if(!isSafePlanarPose(j1-0.01f,j2-0.01f) || j1==20 || j2==40 || j1==125 || j2==130) continue;
        FKResult fk=solveFK(j1,j2); IKResult ik=solveIK(fk.position.x,fk.position.z);
        CHECK(ik.valid); CHECK(std::fabs(ik.j1-j1)<0.001f); CHECK(std::fabs(ik.j2-j2)<0.001f);
#ifdef LEGACY_REFERENCE
        auto oldFK=legacy::solveFK(j1,j2); auto oldIK=legacy::solveIK(fk.position.x,fk.position.z);
        CHECK(oldFK.position.x==fk.position.x && oldFK.position.z==fk.position.z);
        CHECK(oldIK.valid==ik.valid && oldIK.j1==ik.j1 && oldIK.j2==ik.j2);
#endif
    }
    CHECK(!solveIK(0,0).valid); CHECK(!solveIK(1000,1000).valid);
    CHECK(!solveIK(std::numeric_limits<float>::infinity(),0).valid);
}
void testLine() {
    fresh(); send("LINE 71.6 42.5"); finishTask(); CHECK(contains("DONE LINE") && servoWrites.empty());
    Serial.output.clear(); send("RUNLINE 71.6 42.5");
    CHECK(isMotionBusy()); send("HOLD"); Pose stopped=commandedPose();
    runFor(1000); CHECK(commandedPose().angles[Shoulder]==stopped.angles[Shoulder]);
    send("GO"); finishTask(); CHECK(contains("DONE RUNLINE")); checkWritesSafe();
    FKResult end=solveFK(joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    CHECK(std::fabs(end.position.x-71.6f)<3 && std::fabs(end.position.z-42.5f)<3);
    fresh(); send("RUNLINE 0 0"); finishTask(); CHECK(servoWrites.empty());
    fresh(); send("RUNLINE 71.6 42.5"); send("CANCEL"); runFor(200); CHECK(!isMotionBusy());
}
void testParser() {
    fresh();
    const char* invalid[]={"IK", "IK nan 51", "IK 1e2 51", "IK 42 51 extra", "J1 120 extra",
        "J2 .", "J1 +", "SPEED 99999999999999999999999999999999", "HOLD extra", "UNKNOWN"};
    for(const char* s:invalid) { Serial.output.clear(); send(s); CHECK(contains("ERR")); CHECK(servoWrites.empty()); }
    send("IK +42.7 51."); CHECK(contains("IK J1="));
    Serial.input="IK 42."; loop(); Serial.input="7 51\r\n"; loop(); CHECK(contains("IK J1="));
    send(std::string(80,'A')); CHECK(contains("ERR E05")); send("STATUS"); CHECK(contains("task="));
    CHECK(contains("J=120 60") || contains("J=90 120 60 50"));
    Serial.output.clear(); send("HELP"); CHECK(contains("E05") && !isMotionBusy());
    for(const char* command:{"DRAW LINE 1 2 bad 4","DRAW SHAPE N 1 2 3 nan","XYZ 1 2 bad","J1 110x"}) {
        Serial.output.clear(); send(command); CHECK(contains("E05") && servoWrites.empty() && !isMotionBusy());
    }
}
void testHoldAndJoystick() {
    fresh(); CHECK(!isArmHeld()); CHECK(!setControlMode(ControlMode::Cartesian));
    CHECK(lastControlError()==ControlError::Uncalibrated);
    CHECK(setControlMode(ControlMode::Joint));
    send("HOLD"); CHECK(isArmHeld()); CHECK(!requestPose(commandedPose()));
    send("GO"); CHECK(!isArmHeld() && !hasPendingPose());
    for(int raw=0;raw<=1023;++raw) {
        analogValues[0]=raw; JoystickInput in=readJoysticks();
        if(raw<486) CHECK(in.lx>0);
        else if(raw>526) CHECK(in.lx<0);
        else CHECK(in.lx==0);
    }
    analogValues[0]=0; CHECK(readJoysticks().lx==2);
    analogValues[0]=1023; CHECK(readJoysticks().lx==-2);
    // Old firmware switches to fast speed at exactly |raw-center| == 500.
    analogValues[0]=506-499; CHECK(readJoysticks().lx==1);
    analogValues[0]=506-500; CHECK(readJoysticks().lx==2);
    analogValues[0]=506+499; CHECK(readJoysticks().lx==-1);
    analogValues[0]=506+500; CHECK(readJoysticks().lx==-2);
    analogValues[0]=506;
    CHECK(MANUAL_JOINT_SLOW==25.0f && MANUAL_JOINT_FAST==90.0f);
    CHECK(MANUAL_CART_SLOW==25.0f && MANUAL_CART_FAST==100.0f);
    CHECK(joints[Shoulder].commandedAngle==120 && joints[Elbow].commandedAngle==60);
    const int centers[4]={506,519,515,519};
    const int signs[4]={1,-1,1,1};
    for(uint8_t channel=0;channel<4;++channel) {
        for(int offset:{-30,-21,-20,0,20,21,30}) {
            for(uint8_t i=0;i<4;++i) analogValues[i]=centers[i];
            analogValues[channel]=centers[channel]+offset;
            JoystickInput input=readJoysticks(); const int actual[4]={input.lx,input.ly,input.rx,input.ry};
            for(uint8_t i=0;i<4;++i) CHECK(actual[i]==(i!=channel || abs(offset)<=20?0:(offset<0?signs[i]:-signs[i])));
        }
        analogValues[channel]=0; JoystickInput low=readJoysticks(); const int lo[4]={low.lx,low.ly,low.rx,low.ry}; CHECK(lo[channel]==2*signs[channel]);
        analogValues[channel]=1023; JoystickInput high=readJoysticks(); const int hi[4]={high.lx,high.ly,high.rx,high.ry}; CHECK(hi[channel]==-2*signs[channel]);
    }
}
void syntheticCalibration() {
    // Test-only values. They are never compiled into hardware_config.cpp.
    hardwareConfig.base={true,10,170,90}; hardwareConfig.gripper={true,20,100,50};
    hardwareConfig.cartesian={true,ToolModel::FixedPlanar,1,90,5,30,3,2,4};
    configureCalibratedJoints();
}
void test3DAndManual() {
    fresh(); CHECK(!solveIK3D({50,0,50}).valid); CHECK(!isfinite(solveFK3D(90,120,95).x));
    syntheticCalibration();
    for(ToolModel model:{ToolModel::FixedPlanar,ToolModel::DistalLink}) {
        hardwareConfig.cartesian.toolModel=model;
        for(int j0=30;j0<=150;j0+=20) for(int j1=70;j1<=120;j1+=10) for(int j2=90;j2<=120;j2+=10) {
            Point3D point=solveFK3D(j0,j1,j2); IKResult3D ik=solveIK3D(point);
            CHECK(ik.valid); CHECK(fabs(ik.j0-j0)<0.001f); CHECK(fabs(ik.j1-j1)<0.001f); CHECK(fabs(ik.j2-j2)<0.001f);
        }
    }
    CHECK(!solveIK3D({0,0,50}).valid); CHECK(!solveIK3D({1000,1000,1000}).valid);
    CHECK(setControlMode(ControlMode::Cartesian)); CHECK(servoWrites.empty());
    CHECK(setControlMode(ControlMode::Joint));
    fresh(); analogValues[1]=300; runFor(500);
    CHECK(joints[Shoulder].commandedAngle>=107 && joints[Shoulder].commandedAngle<=109);
    CHECK(automaticSpeed()==60 && manualSpeedScale()==1);
    send("H"); CHECK(manualSpeedScale()==1.25f && automaticSpeed()==60);
    send("H"); send("H"); CHECK(manualSpeedScale()==1.5f);
    for(int i=0;i<10;++i) send("L");
    CHECK(manualSpeedScale()==0.75f);
    checkWritesSafe();
    fresh(); send("J1 110"); finishTask(); CHECK(joints[Shoulder].commandedAngle==110);
    CHECK(!isArmHeld()); checkWritesSafe();
    fresh(); send("O"); CHECK(contains("E03") && servoWrites.empty());
    send("x90,y120,z95"); CHECK(contains("E04") && servoWrites.empty());
    send("x90,y120,x95"); CHECK(contains("E05") && servoWrites.empty());
    syntheticCalibration(); hardwareConfig.gripValid=true; hardwareConfig.gripOpen=40; hardwareConfig.gripClosed=90;
    send("S"); CHECK(contains("OK S\r\n")); finishTask(); CHECK(commandedPose().angles[Gripper]==85 && contains("DONE MOVE"));
    send("O"); finishTask(); CHECK(commandedPose().angles[Gripper]==40 && hardwareConfig.gripClosed==90);
}
void testIndependentSpeedAndClock() {
    fresh(); send("SPEED 30"); uint32_t start=simulatedTime; send("J1 100"); finishTask(); uint32_t slow=simulatedTime-start;
    fresh(); send("SPEED 60"); start=simulatedTime; send("J1 100"); send("H"); send("H"); finishTask(); uint32_t fast=simulatedTime-start;
    CHECK(slow>fast && automaticSpeed()==60 && manualSpeedScale()==1.5f);
    fresh(); simulatedTime=UINT32_MAX-40; loop(); send("J1 100");
    while(motionStatus().state==MotionState::Validating) loop();
    advance(20); send("HOLD"); size_t count=servoWrites.size();
    advance(1000); CHECK(servoWrites.size()==count); send("GO"); finishTask();
    CHECK(commandedPose().angles[Shoulder]==100); checkWritesSafe();
}
void recordMovingSession(uint32_t duration) {
    send("REC START"); CHECK(isRecording()); runFor(3000);
    CHECK(recordingStatus().state==RecordState::Recording && recordingStatus().duration==0);
    for(uint32_t t=20;t<=duration;t+=20) { analogValues[1]=(t%2000<1000)?700:300; advance(); }
    analogValues[1]=519; send("REC STOP");
}
void testRecordingAndButtons() {
    fresh(); send("PLAY"); CHECK(contains("E09"));
    send("BTN 2"); CHECK(recordingStatus().state==RecordState::Arming);
    runFor(1000); CHECK(contains("REC ARMING 2")); send("BTN 3"); CHECK(contains("E01"));
    send("BTN 2"); CHECK(!isRecording() && !hasRecording());
    fresh(); send("REC START"); runFor(3000); runFor(11000); send("REC STOP");
    CHECK(hasRecording() && recordingStatus().duration==11000); // Stationary/short takes are retained.
    CHECK(recordingStatus().count==2 && contains("REC 10s; B2 SAVE"));
    fresh(); recordMovingSession(15000); CHECK(hasRecording());
    RecordedFrame first,last; CHECK(getRecordedFrame(0,first));
    CHECK(getRecordedFrame(recordingStatus().count-1,last));
    CHECK(first.timeMs==0 && last.timeMs==15000);
    servoWrites.clear(); send("BTN 3"); CHECK(isMotionBusy());
    send("HOLD"); size_t count=servoWrites.size(); runFor(500); CHECK(count==servoWrites.size());
    send("GO"); finishTask(); CHECK(commandedPose().angles[Shoulder]==last.angles[Shoulder]); checkWritesSafe();
    fresh(); recordMovingSession(20000); CHECK(hasRecording() && !isRecording());
    CHECK(recordingStatus().duration==RECORD_MAX_MS && contains("REC MAX"));
    fresh(); feedButtonByte('B',simulatedTime); feedButtonByte('2',simulatedTime); feedButtonByte('\n',simulatedTime);
    CHECK(isRecording()); feedButtonByte('B',simulatedTime); feedButtonByte('2',simulatedTime); feedButtonByte('\n',simulatedTime);
    CHECK(isRecording()); feedButtonByte('B',simulatedTime); feedButtonByte('9',simulatedTime); feedButtonByte('\n',simulatedTime);
    CHECK(contains("E05")); CHECK(!buttonLinkConfigured());
    send("BTN 4"); CHECK(!isRecording() && !hasRecording() && isArmHeld() && contains("E03"));
    fresh(); send("REC START"); runFor(3000); runFor(100); send("CANCEL");
    CHECK(!isRecording() && !hasRecording() && isArmHeld());
}
float reconstruct(uint16_t time,uint8_t joint) {
    RecordedFrame a,b; CHECK(getRecordedFrame(0,a));
    for(uint8_t i=1;i<recordingStatus().count;++i) {
        CHECK(getRecordedFrame(i,b)); CHECK(b.timeMs>a.timeMs);
        if(time<=b.timeMs) return a.angles[joint]+(static_cast<int>(b.angles[joint])-a.angles[joint])*static_cast<float>(time-a.timeMs)/(b.timeMs-a.timeMs);
        a=b;
    }
    return a.angles[joint];
}
void testAdaptiveRecording() {
    // Desktop-only raw histories measure actual error against every observation.
    for(int scenario=0;scenario<6;++scenario) {
        fresh(); syntheticCalibration(); initializeArm(); advance(); advance();
        if(scenario==4) simulatedTime=UINT32_MAX-3100; // ARMING + record crosses millis wrap.
        CHECK(startRecording(simulatedTime)); runFor(3000);
        uint16_t duration=scenario==5?20000:15000;
        std::vector<Pose> raw{commandedPose()};
        for(uint16_t t=20;t<=duration;t+=20) {
            Pose p=raw.front();
            if(scenario==0) { p.angles[Base]=90+t/500; p.angles[Shoulder]=120-t/1000; }
            if(scenario==1) { // Motion, a real 5-second dwell, reverse motion.
                p.angles[Base]=90+(t<5000?t/250:t<10000?20:20-(t-10000)/250);
            }
            if(scenario==2) { p.angles[Base]=90+(t/20%2?6:-6); } // Worst-case reversals.
            if(scenario>=3) {
                p.angles[Base]=80+t/20%21;
                p.angles[Shoulder]=110+t/20%11;
                p.angles[Elbow]=95+t/20%16;
                p.angles[Gripper]=30+t/20%41;
            }
            CHECK(requestPose(p)); CHECK(applyPendingPose());
            simulatedTime+=20; updateRecording(simulatedTime,commandedPose()); raw.push_back(commandedPose());
            CHECK(recordingStatus().count<=RECORD_CAPACITY);
            CHECK(t<RECORD_MAX_MS?isRecording():hasRecording());
        }
        if(isRecording()) CHECK(stopRecording(simulatedTime));
        CHECK(recordingStatus().duration==duration);
        float error=0;
        for(size_t i=0;i<raw.size();++i) for(uint8_t j=0;j<JointCount;++j)
            error=max(error,fabsf(reconstruct(i*20,j)-raw[i].angles[j]));
        CHECK(error<=recordingStatus().errorBound+0.001f);
        if(scenario<2) CHECK(error<=1.001f);
        else CHECK(recordingStatus().degraded && recordingStatus().count>=75);
        std::cout<<"record scenario="<<scenario<<" frames="<<int(recordingStatus().count)<<" error="<<error<<" bound="<<int(recordingStatus().errorBound)<<"\n";
        RecordedFrame first,last; CHECK(getRecordedFrame(0,first)); CHECK(getRecordedFrame(recordingStatus().count-1,last));
        for(uint8_t j=0;j<JointCount;++j) CHECK(first.angles[j]==raw.front().angles[j] && last.angles[j]==raw.back().angles[j]);
        CHECK(first.timeMs==0 && last.timeMs==duration);
        if(scenario==3) {
            CHECK(startPlayback());
            finishTask(); CHECK(!isArmHeld());
            for(uint8_t j=0;j<JointCount;++j) CHECK(commandedPose().angles[j]==raw.back().angles[j]);
            for(size_t i=0;i+3<servoWrites.size();i+=4) {
                CHECK(servoWrites[i].angle>=10 && servoWrites[i].angle<=170);
                CHECK(isSafePlanarPose(servoWrites[i+1].angle,servoWrites[i+2].angle));
                CHECK(servoWrites[i+3].angle>=20 && servoWrites[i+3].angle<=100);
            }
        }
    }
    // Replay independent of global/manual speed; safe approach completes before clock starts.
    fresh(); recordMovingSession(15000); send("SPEED 5"); send("H");
    CHECK(startPlayback());
    for(int i=0;i<20000 && !contains("PLAY START");++i) advance();
    CHECK(contains("PLAY START")); uint32_t began=simulatedTime;
    runFor(4000); CHECK(pauseMotion(simulatedTime)); size_t writes=servoWrites.size();
    runFor(1700); CHECK(servoWrites.size()==writes); CHECK(resumeMotion(simulatedTime));
    finishTask(); CHECK(simulatedTime-began>=16700 && simulatedTime-began<=16740);
    checkWritesSafe();
    // A sub-tick final stop retains its actual timestamp, not a 20 ms rounding.
    fresh(); recordMovingSession(15000); // Start a separate take.
    send("REC START"); runFor(3000); runFor(15000);
    simulatedTime+=7; CHECK(stopRecording(simulatedTime));
    RecordedFrame ended; CHECK(getRecordedFrame(recordingStatus().count-1,ended));
    CHECK(ended.timeMs==15007);
    // Preparation time is excluded, including manual adjustment during ARMING.
    fresh(); send("BTN 2"); analogValues[1]=300; runFor(200); analogValues[1]=519;
    CHECK(recordingStatus().count==0); runFor(2800); CHECK(recordingStatus().duration==0);
    CHECK(stopRecording(simulatedTime)); RecordedFrame armed;
    CHECK(getRecordedFrame(0,armed)); CHECK(armed.angles[Shoulder]<120 && armed.timeMs==0);
    // Rate ceiling uniformly stretches the whole timeline, with PC disclosure.
    fresh(); syntheticCalibration(); initializeArm(); advance(); advance();
    CHECK(startRecording(simulatedTime)); runFor(3000);
    Pose jump=commandedPose(); jump.angles[Base]=150;
    CHECK(requestPose(jump)); CHECK(applyPendingPose());
    simulatedTime+=20; updateRecording(simulatedTime,commandedPose());
    for(int i=0;i<24;++i) { simulatedTime+=20; updateRecording(simulatedTime,commandedPose()); }
    CHECK(stopRecording(simulatedTime)); CHECK(startPlayback());
    for(int i=0;i<20000 && !contains("PLAY START");++i) advance();
    CHECK(contains("PLAY timeScale=25.00")); began=simulatedTime;
    finishTask(); CHECK(simulatedTime-began>=12500 && simulatedTime-began<=12540);
    CHECK(commandedPose().angles[Base]==150 && !isArmHeld());
    // B4 aborts both ARMING and recording through the same safe HOME owner.
    fresh(); syntheticCalibration(); initializeArm(); advance(); advance();
    hardwareConfig.homeValid=true; hardwareConfig.home=commandedPose();
    send("BTN 2"); runFor(3500); send("BTN 4");
    CHECK(!isRecording() && !hasRecording() && motionStatus().kind==TaskKind::Home);
    finishTask(); CHECK(!isArmHeld());
    // Delayed scheduler crosses several retained timestamps in one update.
    fresh(); recordMovingSession(15000); CHECK(startPlayback());
    for(int i=0;i<20000 && !contains("PLAY START");++i) advance();
    advance(1000); finishTask(); CHECK(!isArmHeld());
}
void testRecordingInputAndErrors() {
    const char* names[]={"E05","E01","E02","E03","E04","E05","E06","E07","E08","E09","E10","T01","T02","T03","D01","P01","P02"};
    for(uint8_t i=0;i<sizeof(names)/sizeof(names[0]);++i) {
        Serial.output.clear(); rejectControl(static_cast<ControlError>(i)); printControlError();
        CHECK(Serial.output==std::string("ERR ")+names[i]+"\r\n");
    }
    Serial.output.clear(); rejectControl(static_cast<ControlError>(255)); printControlError();
    CHECK(Serial.output=="ERR E05\r\n");
    for(float bad:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-1.0f,181.0f}) {
        fresh(); CHECK(startRecording(simulatedTime));
        Pose p=commandedPose(); p.angles[Base]=bad;
        updateRecording(simulatedTime+3000,p);
        CHECK(!isRecording() && !hasRecording() && recordingStatus().count==0);
        CHECK(lastControlError()==ControlError::Unsafe && servoWrites.empty());
    }
    fresh(); CHECK(startRecording(simulatedTime)); runFor(3000);
    Pose bad=commandedPose(); bad.angles[Shoulder]=20; // In range separately, unsafe combination.
    updateRecording(simulatedTime+20,bad);
    CHECK(!isRecording() && !hasRecording() && lastControlError()==ControlError::Unsafe);
}
void testHomeAndPick() {
    fresh(); send("HOME"); CHECK(contains("E03") && !isMotionBusy() && servoWrites.empty());
    send("GO"); send("A"); CHECK(contains("P01") && servoWrites.empty());
    syntheticCalibration(); hardwareConfig.gripValid=true; hardwareConfig.gripOpen=80; hardwareConfig.gripClosed=30;
    initializeArm(); advance(); CHECK(attachedPins.size()==6); servoWrites.clear();
    hardwareConfig.homeValid=true; hardwareConfig.home=commandedPose();
    hardwareConfig.home.angles[Shoulder]=110;
    CHECK(startHome()); finishTask(); CHECK(commandedPose().angles[Shoulder]==110);
    Pose poses[PICK_STAGE_COUNT]; for(uint8_t i=0;i<PICK_STAGE_COUNT;++i) { poses[i]=commandedPose(); poses[i].angles[Shoulder]=110+(i%2)*5; poses[i].angles[Gripper]=i<2 || i>=6?80:30; }
    setSyntheticPickProgram(0,poses); setSyntheticPickProgram(1,poses); setSyntheticPickProgram(2,poses);
    CHECK(onButtonPressed(1)); CHECK(!onButtonPressed(1)); finishTask();
    CHECK(onButtonPressed(1)); finishTask(); CHECK(onButtonPressed(1)); finishTask();
    CHECK(commandedPose().angles[Shoulder]==115);
    CHECK(cancelAndReturnHome()); finishTask(); CHECK(commandedPose().angles[Shoulder]==110);
}
void testCurveSampler() {
    TeachPoint points[5]={{0,0},{3,4},{9,1},{14,6},{20,0}};
    for(uint8_t segment=0;segment<4;++segment) {
        TeachPoint a=segment==0?TeachPoint{2*points[0].x-points[1].x,2*points[0].y-points[1].y}:points[segment-1];
        TeachPoint d=segment==3?TeachPoint{2*points[4].x-points[3].x,2*points[4].y-points[3].y}:points[segment+2];
        TeachPoint out;
        CHECK(centripetalSample(a,points[segment],points[segment+1],d,0,out));
        CHECK(pointDistance(out,points[segment])<0.0001f);
        CHECK(centripetalSample(a,points[segment],points[segment+1],d,1,out));
        CHECK(pointDistance(out,points[segment+1])<0.0001f);
        for(int i=0;i<=100;++i) { CHECK(centripetalSample(a,points[segment],points[segment+1],d,i/100.0f,out)); CHECK(isfinite(out.x) && isfinite(out.y)); }
    }
    TeachPoint out; CHECK(!centripetalSample(points[0],points[0],points[1],points[2],0.5f,out));
    CHECK(!centripetalSample(points[0],points[1],points[2],points[3],-0.1f,out));
    for(ShapeId shape:{ShapeId::N,ShapeId::Triangle,ShapeId::Z,ShapeId::V}) {
        for(uint8_t i=0;i<shapeVertexCount(shape);++i) { CHECK(shapeVertex(shape,i,{0,0},{10,10},out)); CHECK(out.x>=0 && out.x<=10 && out.y>=0 && out.y<=10); }
    }
    const TeachPoint expected[4][4]={{{0,0},{0,10},{10,0},{10,10}},{{0,0},{10,0},{5,10},{0,0}},{{0,10},{10,10},{0,0},{10,0}},{{0,10},{5,0},{10,10},{0,0}}};
    for(uint8_t shape=0;shape<4;++shape) for(uint8_t i=0;i<shapeVertexCount(static_cast<ShapeId>(shape));++i) {
        CHECK(shapeVertex(static_cast<ShapeId>(shape),i,{0,0},{10,10},out));
        CHECK(out.x==expected[shape][i].x && out.y==expected[shape][i].y);
    }
    CHECK(!shapeVertex(static_cast<ShapeId>(4),0,{0,0},{10,10},out));
}
void testPackedPickCommands() {
    fresh(); syntheticCalibration();
    hardwareConfig.gripValid=true; hardwareConfig.gripOpen=80; hardwareConfig.gripClosed=30;
    Pose poses[PICK_STAGE_COUNT];
    for(uint8_t i=0;i<PICK_STAGE_COUNT;++i) { poses[i]=commandedPose(); poses[i].angles[Shoulder]=110.75f+i; poses[i].angles[Gripper]=i<2 || i>=6?80:30; }
    setSyntheticPickProgram(0,poses); CHECK(pickCalibrationValid(0));
    for(uint8_t i=0;i<PICK_STAGE_COUNT;++i) {
        Pose unpacked; uint16_t dwell;
        CHECK(getPickStage(0,i,unpacked,dwell) && dwell==100);
        for(uint8_t j=0;j<JointCount;++j) CHECK(unpacked.angles[j]==static_cast<int>(poses[i].angles[j]));
        Pose safe; CHECK(prepareOutputPose(unpacked,safe));
    }
    poses[7].angles[Shoulder]=20.9f; poses[7].angles[Elbow]=127.1f;
    setSyntheticPickProgram(0,poses); CHECK(pickCalibrationValid(0)); // Legal lower boundary now quantizes safely.
    poses[7].angles[Shoulder]=std::numeric_limits<float>::quiet_NaN();
    setSyntheticPickProgram(1,poses); CHECK(!pickCalibrationValid(1));
    CHECK(servoWrites.empty());
}
void testMotionPayloadTransitions() {
    fresh(); syntheticCalibration();
    Pose target=commandedPose(); target.angles[Shoulder]=110;
    CHECK(startJointMove(target)); finishTask(); CHECK(commandedPose().angles[Shoulder]==110);
    Point3D cart=solveFK3D(90,115,100);
    CHECK(startCartesianMove(cart)); finishTask();
    Point3D actual=solveFK3D(joints[Base].commandedAngle,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    CHECK(fabs(actual.x-cart.x)<3 && fabs(actual.z-cart.z)<3);
    hardwareConfig.homeValid=true; hardwareConfig.home=commandedPose(); hardwareConfig.home.angles[Shoulder]=120;
    CHECK(startHome()); finishTask(); CHECK(commandedPose().angles[Shoulder]==120);
    Point2D planar=solveFK(110,100).position;
    CHECK(startLine(planar,TaskKind::LineExecution)); finishTask();
    CHECK(!isArmHeld()); CHECK(cancelAndReturnHome()); finishTask();
    CHECK(commandedPose().angles[Shoulder]==120 && !hasPendingPose());
}
void testRuntimePickPersistence() {
    fresh(); send("PICK LOAD"); CHECK(contains("E09") && isArmHeld()); send("GO");
    send("PICK SET A SRC"); CHECK(contains("E03") && pickStageMask(0)==0);
    syntheticCalibration(); hardwareConfig.gripValid=true; hardwareConfig.gripOpen=80; hardwareConfig.gripClosed=30;
    initializeArm(); advance(); advance();
    hardwareConfig.homeValid=true; hardwareConfig.home=commandedPose();
    // Eight independently taught accepted command poses; SRC/DST alone cannot run.
    send("PICK SET A SRC"); send("PICK SET A DST");
    CHECK(pickStageMask(0)==34 && pickStageMask(1)==0 && !startPickPlace(0));
    const char* stages[]={"APPROACH","SRC","GRIP","LIFT","TRANSFER","DST","RELEASE","RETREAT"};
    for(uint8_t i=0;i<8;++i) {
        Pose p=commandedPose(); p.angles[Gripper]=i<2 || i>=6?80:30;
        CHECK(requestPose(p) && applyPendingPose());
        send(std::string("PICK SET A ")+stages[i]); CHECK(!isMotionBusy());
    }
    CHECK(pickStageMask(0)==255 && pickCalibrationValid(0));
    send("PICK SPEED A 3 40"); send("PICK WAIT A GRIP 200");
    CHECK(pickStageSpeed(0,2)==40 && pickStageDwell(0,2)==200);
    // Different objects may need different measured clamping angles.
    Pose poses[8];
    for(uint8_t i=0;i<8;++i) { poses[i]=commandedPose(); poses[i].angles[Gripper]=i<2 || i>=6?80:50; }
    setSyntheticPickProgram(2,poses);
    Pose custom; uint16_t dwell;
    CHECK(getPickStage(2,2,custom,dwell) && custom.angles[Gripper]==50);
    poses[4].angles[Gripper]=55; setSyntheticPickProgram(2,poses);
    CHECK(!getPickStage(2,4,custom,dwell)); // Do not open/change grip during carry.
    send("PICK CLEAR C");
    for(const char* bad:{"PICK SET D SRC","PICK SET A 9","PICK SPEED A 3 4","PICK SPEED A 3 90.1","PICK WAIT A 3 2551","PICK WAIT A 3 -1","PICK WAIT A 3 23","PICK SET A SRC extra"}) {
        Serial.output.clear(); send(bad); CHECK(contains("E05"));
    }
    // Saving freezes manual/business events but continues the PC loop.
    servoWrites.clear(); send("PICK SAVE"); CHECK(isPickSaving() && isArmHeld());
    send("GO"); send("BTN 2"); CHECK(isArmHeld() && !isRecording());
    CHECK(!startRecording(simulatedTime)); CHECK(!beginTeach());
    for(int i=0;i<200 && isPickSaving();++i) advance();
    CHECK(!isPickSaving() && contains("PICK SAVED; HELD") && servoWrites.empty());
    uint8_t baseline[1024]; memcpy(baseline,EEPROM.bytes,1024);
    send("PICK CLEAR A"); CHECK(pickStageMask(0)==0); send("PICK LOAD"); CHECK(pickStageMask(0)==255);
    // Real boot has a pending startup frame. Valid data must still restore.
    initializeArm(); CHECK(hasPendingPose()); initializeApplication();
    CHECK(pickStageMask(0)==255 && pickStageSpeed(0,2)==40);
    CHECK(applyPendingPose()); holdArm();
    // Every interrupted write boundary retains the old committed slot.
    for(int cut=0;cut<=156;++cut) {
        memcpy(EEPROM.bytes,baseline,1024); initializePickConfiguration();
        send("PICK CLEAR A"); send("PICK SET B SRC"); CHECK(savePickConfiguration());
        for(int i=0;i<cut;++i) updatePickPersistence();
        initializePickConfiguration();
        CHECK(pickStageMask(0)==(cut==156?0:255));
        CHECK(pickStageMask(1)==(cut==156?2:0));
        CHECK(pickStageMask(2)==0);
    }
    // Corrupt any byte of the sole slot: reject, including version, sequence,
    // CRC, commit marker, masks, angles, speed and timing.
    for(int byte=0;byte<155;++byte) {
        memcpy(EEPROM.bytes,baseline,1024); EEPROM.bytes[byte]^=1;
        initializePickConfiguration(); CHECK(pickStageMask(0)==0 && !loadPickConfiguration());
    }
    memcpy(EEPROM.bytes,baseline,1024); initializePickConfiguration();
    send("PICK SET B SRC"); CHECK(savePickConfiguration());
    while(isPickSaving()) updatePickPersistence();
    uint8_t twoSlots[1024]; memcpy(twoSlots,EEPROM.bytes,1024);
    for(int byte=155;byte<310;++byte) {
        memcpy(EEPROM.bytes,twoSlots,1024); EEPROM.bytes[byte]^=1;
        initializePickConfiguration(); CHECK(pickStageMask(0)==255 && pickStageMask(1)==0);
    }
    memcpy(EEPROM.bytes,baseline,1024); initializePickConfiguration();
    send("GO"); CHECK(startPickPlace(0)); servoWrites.clear();
    send("PICK CLEAR A"); CHECK(pickStageMask(0)==255 && lastControlError()==ControlError::Busy);
    finishTask(); CHECK(!isArmHeld());
    // All phases preflight before first movement, even corrupt runtime edits.
    EEPROM.bytes[310+1+7*6+1]=20; EEPROM.bytes[310+1+7*6+2]=127;
    servoWrites.clear(); CHECK(startPickPlace(0)); finishTask();
    CHECK(isArmHeld() && servoWrites.empty() && lastControlError()==ControlError::Unsafe);
    clearSyntheticPickPrograms(); initializePickConfiguration();
}
void testManualFrameOwnership() {
    fresh(); syntheticCalibration(); initializeArm(); advance(); advance();
    CHECK(setControlMode(ControlMode::Cartesian)); advance();
    Point3D before=solveFK3D(joints[Base].commandedAngle,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    for(uint8_t i=0;i<5;++i) {
        simulatedTime+=20; updateManualControl(simulatedTime,JoystickInput{0,1,0,0}); CHECK(applyPendingPose());
    }
    Point3D after=solveFK3D(joints[Base].commandedAngle,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    CHECK(fabs((after.x-before.x)+0.1f*MANUAL_CART_SLOW)<3.0f); // 5 mm ideal, integer Servo quantization.
    CHECK(fabs(after.y-before.y)<2); // Forward/back must not command lateral motion.
    for(uint8_t i=0;i<5;++i) {
        simulatedTime+=20; updateManualControl(simulatedTime,JoystickInput{1,0,0,0}); CHECK(applyPendingPose());
    }
    Point3D lateral=solveFK3D(joints[Base].commandedAngle,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    CHECK(fabs((lateral.y-after.y)-0.1f*MANUAL_CART_SLOW)<3.0f); // Left/right controls world Y.
    CHECK(fabs(lateral.x-after.x)<2);
    Pose target=commandedPose(); target.angles[Shoulder]=110;
    CHECK(startJointMove(target)); servoWrites.clear();
    updateManualControl(simulatedTime+1000,JoystickInput{2,2,2,2});
    CHECK(!hasPendingPose() && servoWrites.empty()); // Manual cannot steal validation/paused owner.
    CHECK(pauseMotion(simulatedTime)); updateManualControl(simulatedTime+2000,JoystickInput{2,2,2,2});
    CHECK(!hasPendingPose() && servoWrites.empty());
    CHECK(resumeMotion(simulatedTime)); finishTask();
    CHECK(commandedPose().angles[Shoulder]==110 && !isArmHeld());
}
TeachPoint blendPoint(const TeachPoint& a,const TeachPoint& b,float t) { return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t}; }
void testCurveIndependentReference() {
    TeachPoint a={0,0},b={3,4},c={9,1},d={14,6};
    float t0=0,t1=sqrtf(pointDistance(a,b)),t2=t1+sqrtf(pointDistance(b,c)),t3=t2+sqrtf(pointDistance(c,d));
    for(int i=0;i<=100;++i) {
        float u=i/100.0f,t=t1+(t2-t1)*u;
        TeachPoint a1=blendPoint(a,b,(t-t0)/(t1-t0)),a2=blendPoint(b,c,(t-t1)/(t2-t1)),a3=blendPoint(c,d,(t-t2)/(t3-t2));
        TeachPoint b1=blendPoint(a1,a2,(t-t0)/(t2-t0)),b2=blendPoint(a2,a3,(t-t1)/(t3-t1));
        TeachPoint reference=blendPoint(b1,b2,(t-t1)/(t2-t1)),actual;
        CHECK(centripetalSample(a,b,c,d,u,actual)); CHECK(pointDistance(reference,actual)<0.0001f);
    }
    TeachPoint expected,scratch,again;
    CHECK(centripetalSample(a,b,c,d,0.4f,expected));
    CHECK(centripetalSample(d,c,b,a,0.9f,scratch));
    CHECK(centripetalSample(a,b,c,d,0.4f,again));
    CHECK(expected.x==again.x && expected.y==again.y); // No persistent curve scratch.
}
void prepareTeachFive() {
    fresh(); syntheticCalibration(); initializeArm(); advance(); advance();
    hardwareConfig.homeValid=true; hardwareConfig.home=commandedPose();
    send("TEACH START"); CHECK(teachingActive()); send("BTN 1"); CHECK(teachSettings().drawValid);
    Pose p=commandedPose(); p.angles[Shoulder]=123; CHECK(requestPose(p)); CHECK(applyPendingPose());
    send("BTN 2"); CHECK(drawSettingsValid()); send("BTN 3"); CHECK(!markingActive() && contains("T01"));
    send("TEACH TEACHZ"); send("BTN 3"); CHECK(!markingActive()); finishTask(); CHECK(markingActive());
    for(uint8_t i=0;i<5;++i) {
        p=commandedPose(); p.angles[Base]=80+5*i;
        CHECK(requestPose(p)); CHECK(applyPendingPose()); send("BTN 1"); CHECK(teachPointCount()==i+1);
    }
    servoWrites.clear(); Serial.output.clear();
}
void testTeachAndDraw() {
    fresh(); send("TEACH START"); CHECK(contains("E03"));
    prepareTeachFive(); send("ORDER 5 4 3 2 1"); send("BTN 4");
    CHECK(teachPointCount()==4); TeachPoint undone;
    CHECK(getTeachPoint(0,undone)); Point3D surviving=solveFK3D(95,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    CHECK(fabs(undone.x-surviving.x)<0.001f);
    send("MARK"); CHECK(teachPointCount()==5 && getTeachPoint(4,undone));
    Point3D readded=solveFK3D(100,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle); CHECK(fabs(undone.x-readded.x)<0.001f);
    send("DRAW POLY"); send("UNDO"); CHECK(teachPointCount()==5 && contains("E01"));
    finishTask(); send("CLEAR"); send("UNDO"); CHECK(teachPointCount()==0 && contains("E09"));
    prepareTeachFive(); send("MARK"); CHECK(contains("E08") && teachPointCount()==5);
    send("ORDER 1 1 3 4 5"); CHECK(contains("E05"));
    send("ORDER 5 4 3 2 1"); TeachPoint ordered; CHECK(getTeachPoint(0,ordered));
    Point3D expected=solveFK3D(100,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle); CHECK(fabs(ordered.x-expected.x)<0.001f);
    send("ORDER 1 2 3 4 5"); send("BTN 2"); CHECK(drawingTaskActive());
    send("MARK"); CHECK(contains("E01")); send("BTN 1"); CHECK(motionStatus().state==MotionState::Paused);
    size_t count=servoWrites.size(); runFor(2000); CHECK(count==servoWrites.size());
    send("BTN 2"); finishTask(); CHECK(commandedPose().angles[Base]==hardwareConfig.home.angles[Base]);
    CHECK(commandedPose().angles[Shoulder]==hardwareConfig.home.angles[Shoulder]);
    CHECK(!isArmHeld() && servoWrites.size()%4==0);
    for(uint8_t point=0;point<5;++point) {
        TeachPoint target; getTeachPoint(point,target); float closest=10000;
        for(size_t i=0;i+3<servoWrites.size();i+=4) {
            CHECK(isSafePlanarPose(servoWrites[i+1].angle,servoWrites[i+2].angle));
            Point3D actual=solveFK3D(servoWrites[i].angle,servoWrites[i+1].angle,servoWrites[i+2].angle);
            if(fabs(actual.z-teachSettings().drawZ)<2) closest=min(closest,pointDistance({actual.x,actual.y},target));
        }
        CHECK(closest<3); // Integer Servo angles, not a measured paper tolerance.
    }
    send("DRAW CURVE"); CHECK(drawingTaskActive()); finishTask(); CHECK(!isArmHeld());
    send("DRAW POLY"); CHECK(drawingTaskActive());
    for(int i=0;i<1000 && motionStatus().state==MotionState::Validating;++i) loop();
    CHECK(motionStatus().state==MotionState::Running); runFor(200); send("BTN 1"); send("BTN 3");
    CHECK(isMotionBusy() && motionStatus().kind==TaskKind::Home); finishTask();
    CHECK(commandedPose().angles[Shoulder]==hardwareConfig.home.angles[Shoulder]);
    send("CLEAR"); CHECK(teachPointCount()==0 && currentWorkflow()==Workflow::TeachSetup);
    send("TEACH EXIT"); CHECK(!teachingActive());
    CHECK(startDrawLine({86,-5},{88,5})); finishTask(); CHECK(!isArmHeld());
    for(ShapeId shape:{ShapeId::N,ShapeId::Triangle,ShapeId::Z,ShapeId::V}) {
        CHECK(!startBasicShape(shape,{86,-5},{88,5})); CHECK(lastControlError()==ControlError::Geometry);
        CHECK(startBasicShape(shape,{86,-5},{91,5})); finishTask(); CHECK(!isArmHeld());
    }
}
void testFailureAndLatency() {
    fresh(); send(std::string("J1 11")+char(1)+"0"); CHECK(servoWrites.empty() && !isMotionBusy());
    fresh(); send("J1 100");
    while(motionStatus().state==MotionState::Validating) loop();
    Serial.printCostMs=1;
    for(int i=0;isMotionBusy() && i<100;++i) {
        size_t before=servoWrites.size(); send("STATUS"); CHECK(servoWrites.size()<=before+2); advance();
    }
    finishTask(); CHECK(commandedPose().angles[Shoulder]==100); Serial.printCostMs=0; checkWritesSafe();
    fresh(); send("J1 100");
    while(motionStatus().state==MotionState::Validating) loop();
    joints[Shoulder].minAngle=121; // Fault injection, never a production limit change.
    advance(); CHECK(!isMotionBusy() && servoWrites.empty() && isArmHeld());
    prepareTeachFive(); CHECK(startDrawLine({1000,1000},{2000,2000})); finishTask();
    CHECK(servoWrites.empty() && isArmHeld());
    fresh(); CHECK(!setManualSpeedScale(0.5f)); CHECK(!setManualSpeedScale(2));
    float value; CHECK(parseNumber("-.25",value) && value==-0.25f);
    CHECK(!parseNumber("1e2",value)); CHECK(!parseNumber("--1",value));
}
void testFrozenTaskSpeeds() {
    // Draw timing is invariant under global SPEED and joystick H/L.
    uint32_t duration[3];
    for(uint8_t variant=0;variant<3;++variant) {
        prepareTeachFive(); send(variant==0?"SPEED 5":"SPEED 90");
        if(variant) send("H");
        send("FEED 20"); CHECK(contains("E05"));
        send("DRAW POLY");
        while(motionStatus().state==MotionState::Validating) advance();
        uint32_t start=simulatedTime;
        send("FEED 30"); CHECK(contains("E05"));
        finishTask(); duration[variant]=simulatedTime-start;
    }
    CHECK(duration[0]==duration[1] && duration[1]==duration[2]);
    fresh();
    for(const char* cmd:{"FEED", "FEED 0", "FEED 35", "FEED 12.5", "FEED 100.01"}) {
        Serial.output.clear(); send(cmd); CHECK(contains("E05") && !isMotionBusy() && servoWrites.empty());
    }
    Serial.output.clear(); send("HELP"); CHECK(!contains("FEED"));
    Serial.output.clear(); send("STATUS"); CHECK(!contains("feed=") && contains("pickNext=A"));
    // HOME keeps its fixed 60 deg/s timing under SPEED/H/L.
    for(uint8_t variant=0;variant<2;++variant) {
        fresh(); hardwareConfig.homeValid=true; hardwareConfig.home=commandedPose();
        hardwareConfig.home.angles[Shoulder]=100;
        CHECK(setAutomaticSpeed(variant?90:5)); adjustManualSpeedScale(variant);
        CHECK(startHome()); while(motionStatus().state==MotionState::Validating) advance();
        uint32_t start=simulatedTime; finishTask(); duration[variant]=simulatedTime-start;
    }
    CHECK(duration[0]==duration[1]);
    fresh(); send("REC START"); runFor(3000); runFor(1000); send("REC STOP");
    CHECK(hasRecording() && contains("SHORT"));
    send("REC START"); CHECK(contains("REC OVERWRITE") && isRecording());
    clearRecording();
}
void testIntegrationContracts() {
    prepareTeachFive();
    TeachPoint saved[5]; for(uint8_t i=0;i<5;++i) CHECK(getTeachPoint(i,saved[i]));
    send("ORDER 3 1 5 2 4");
    const uint8_t permutation[]={2,0,4,1,3};
    for(uint8_t i=0;i<5;++i) { TeachPoint p; CHECK(getTeachPoint(i,p)); CHECK(p.x==saved[permutation[i]].x && p.y==saved[permutation[i]].y); }
    for(const char* draw:{"DRAW POLY","DRAW CURVE"}) {
        servoWrites.clear(); send(draw); CHECK(drawingTaskActive());
        while(motionStatus().state==MotionState::Validating) advance();
        CHECK(motionStatus().state==MotionState::Running);
        runFor(200); send("BTN 1"); CHECK(motionStatus().state==MotionState::Paused);
        size_t heldWrites=servoWrites.size(); runFor(1200); CHECK(servoWrites.size()==heldWrites);
        send("REC START"); send("A"); send("ORDER 1 2 3 4 5"); CHECK(contains("E01") && !isRecording());
        send("BTN 2"); finishTask(); CHECK(!isArmHeld() && teachPointCount()==5);
        for(uint8_t i=0;i<5;++i) {
            TeachPoint p; CHECK(getTeachPoint(i,p)); CHECK(p.x==saved[permutation[i]].x);
            float nearest=10000;
            for(size_t k=0;k+3<servoWrites.size();k+=4) {
                CHECK(isSafePlanarPose(servoWrites[k+1].angle,servoWrites[k+2].angle));
                Point3D q=solveFK3D(servoWrites[k].angle,servoWrites[k+1].angle,servoWrites[k+2].angle);
                if(fabs(q.z-teachSettings().drawZ)<2) nearest=min(nearest,pointDistance({q.x,q.y},p));
            }
            CHECK(nearest<3); // Simulation command-space tolerance, not measured paper accuracy.
        }
    }
    send("DRAW POLY"); while(motionStatus().state==MotionState::Validating) advance();
    runFor(200); send("BTN 3"); CHECK(motionStatus().kind==TaskKind::Home);
    uint16_t recoveryWaypoints=motionStatus().totalWaypoints;
    send("HOME"); CHECK(motionStatus().totalWaypoints==recoveryWaypoints); // Repeated request cannot drop lift.
    finishTask(); CHECK(!isArmHeld() && teachPointCount()==5);
    send("DRAW LINE 1000 1000 2000 2000"); finishTask();
    CHECK(isArmHeld() && teachPointCount()==5); send("GO"); send("DRAW CURVE"); finishTask(); CHECK(!isArmHeld());
    send("TEACH EXIT"); CHECK(!teachingActive() && teachPointCount()==5);
    send("DRAW POLY"); finishTask(); CHECK(!isArmHeld() && teachPointCount()==5);
}
void testDecimalDiagnostics() {
    fresh();
    for(uint8_t digits=0;digits<=2;++digits) {
        Serial.output.clear(); printDecimal(12.375f,digits);
        CHECK(Serial.output==(digits==0?"12":digits==1?"12.4":"12.38"));
    }
    Serial.output.clear(); printDecimal(-0.125f); CHECK(Serial.output=="-0.13");
    Serial.output.clear(); printDecimal(0); CHECK(Serial.output=="0.00");
    Serial.output.clear(); printDecimal(999.999f); CHECK(Serial.output=="1000.00");
    Serial.output.clear(); printDecimal(std::numeric_limits<float>::quiet_NaN()); CHECK(Serial.output=="nan");
    Serial.output.clear(); printDecimal(std::numeric_limits<float>::infinity()); CHECK(Serial.output=="inf");
    Serial.output.clear(); printDecimal(1e30f); CHECK(Serial.output=="ovf" && servoWrites.empty());
    send("STATUS"); CHECK(contains("manualScale=1.00") && contains("autoSpeed=60.0"));
}
}
int main() {
    for(uint8_t i=0;i<JointCount;++i) original[i]=joints[i];
    testSafety(); testMath(); testLine(); testParser(); testHoldAndJoystick(); test3DAndManual(); testIndependentSpeedAndClock();
    testRecordingAndButtons(); testAdaptiveRecording(); testRecordingInputAndErrors(); testHomeAndPick(); testPackedPickCommands(); testMotionPayloadTransitions(); testManualFrameOwnership();
    testRuntimePickPersistence(); testCurveSampler(); testCurveIndependentReference(); testTeachAndDraw(); testFailureAndLatency(); testFrozenTaskSpeeds(); testIntegrationContracts(); testDecimalDiagnostics();
    std::cout << "PASS: " << assertions << " checks\n";
}
