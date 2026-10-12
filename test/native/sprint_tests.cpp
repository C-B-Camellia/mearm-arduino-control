#include "Arduino.h"
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
#include <cstdlib>
#include <iostream>
#include <limits>

uint32_t simulatedTime = 0;
std::vector<ServoWrite> servoWrites;
std::vector<int> attachedPins;
std::vector<uint32_t> delayCalls;
int analogValues[4] = {506, 519, 515, 519};
SerialStub Serial;
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
    simulatedTime=0; servoWrites.clear(); attachedPins.clear(); delayCalls.clear();
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
    CHECK(!prepareOutputPose(candidate,output)); CHECK(!requestPose(candidate));
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
    }
    CHECK(!solveIK(0,0).valid); CHECK(!solveIK(1000,1000).valid);
    CHECK(!solveIK(std::numeric_limits<float>::infinity(),0).valid);
}
void testLine() {
    fresh(); send("LINE 71.6 42.5"); finishTask(); CHECK(contains("LINE VALID") && servoWrites.empty());
    Serial.output.clear(); send("RUNLINE 71.6 42.5");
    CHECK(isMotionBusy()); send("HOLD"); Pose stopped=commandedPose();
    runFor(1000); CHECK(commandedPose().angles[Shoulder]==stopped.angles[Shoulder]);
    send("GO"); finishTask(); CHECK(contains("RUNLINE DONE")); checkWritesSafe();
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
    send(std::string(80,'A')); CHECK(contains("ERR command too long")); send("STATUS"); CHECK(contains("task="));
}
void testHoldAndJoystick() {
    fresh(); CHECK(!isArmHeld()); CHECK(!setControlMode(ControlMode::Cartesian));
    CHECK(lastControlError()==ControlError::Uncalibrated);
    CHECK(setControlMode(ControlMode::Joint));
    send("HOLD"); CHECK(isArmHeld()); CHECK(!requestPose(commandedPose()));
    send("GO"); CHECK(!isArmHeld() && !hasPendingPose());
    for(int raw=0;raw<=1023;++raw) {
        analogValues[0]=raw; JoystickInput in=readJoysticks();
        if(raw<486) CHECK(in.lx<0);
        else if(raw>526) CHECK(in.lx>0);
        else CHECK(in.lx==0);
    }
    analogValues[0]=0; CHECK(readJoysticks().lx==-2);
    analogValues[0]=1023; CHECK(readJoysticks().lx==2);
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
    fresh(); analogValues[1]=700; runFor(500);
    CHECK(joints[Shoulder].commandedAngle>=99 && joints[Shoulder].commandedAngle<=100);
    CHECK(automaticSpeed()==60 && manualSpeedScale()==1);
    send("H"); CHECK(manualSpeedScale()==1.25f && automaticSpeed()==60);
    send("H"); send("H"); CHECK(manualSpeedScale()==1.5f);
    for(int i=0;i<10;++i) send("L");
    CHECK(manualSpeedScale()==0.75f);
    checkWritesSafe();
    fresh(); send("J1 110"); finishTask(); CHECK(joints[Shoulder].commandedAngle==110);
    CHECK(!isArmHeld()); checkWritesSafe();
    fresh(); send("O"); CHECK(contains("UNCALIBRATED") && servoWrites.empty());
    send("x90,y120,z95"); CHECK(contains("DISABLED") && servoWrites.empty());
    send("x90,y120,x95"); CHECK(contains("INVALID") && servoWrites.empty());
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
    send("REC START"); CHECK(isRecording());
    for(uint32_t t=20;t<=duration;t+=20) { analogValues[1]=(t%2000<1000)?700:300; advance(); }
    analogValues[1]=519; send("REC STOP");
}
void testRecordingAndButtons() {
    fresh(); send("PLAY"); CHECK(contains("NO_DATA"));
    send("BTN 2"); CHECK(isRecording()); runFor(1000); send("BTN 2");
    CHECK(!hasRecording() && contains("TOO_SHORT"));
    fresh(); send("REC START"); runFor(11000); send("REC STOP");
    CHECK(!hasRecording() && contains("NO_MOVEMENT"));
    fresh(); recordMovingSession(11200); CHECK(hasRecording());
    CHECK(recordingStatus().duration>10000 && recordingStatus().count<=RECORD_CAPACITY);
    RecordedFrame first,last; CHECK(getRecordedFrame(0,first));
    CHECK(getRecordedFrame(recordingStatus().count-1,last));
    CHECK(first.timeMs==0 && last.timeMs==11200);
    servoWrites.clear(); send("BTN 3"); CHECK(isMotionBusy());
    send("HOLD"); size_t count=servoWrites.size(); runFor(500); CHECK(count==servoWrites.size());
    send("GO"); finishTask(); CHECK(commandedPose().angles[Shoulder]==last.angles[Shoulder]); checkWritesSafe();
    fresh(); send("REC START"); CHECK(!startJointMove(commandedPose()));
    for(uint32_t t=20;t<=17000;t+=20) { analogValues[1]=(t%2000<1000)?700:300; advance(); }
    CHECK(!isRecording() && hasRecording() && recordingStatus().count==RECORD_CAPACITY);
    fresh(); feedButtonByte('B',simulatedTime); feedButtonByte('2',simulatedTime); feedButtonByte('\n',simulatedTime);
    CHECK(isRecording()); feedButtonByte('B',simulatedTime); feedButtonByte('2',simulatedTime); feedButtonByte('\n',simulatedTime);
    CHECK(isRecording()); feedButtonByte('B',simulatedTime); feedButtonByte('9',simulatedTime); feedButtonByte('\n',simulatedTime);
    CHECK(contains("INVALID")); CHECK(!buttonLinkConfigured());
    send("BTN 4"); CHECK(isRecording() && contains("BUSY"));
}
void testHomeAndPick() {
    fresh(); send("HOME"); CHECK(contains("UNCALIBRATED") && !isMotionBusy() && servoWrites.empty());
    send("GO"); send("A"); CHECK(contains("UNCALIBRATED") && servoWrites.empty());
    syntheticCalibration(); hardwareConfig.gripValid=true; hardwareConfig.gripOpen=80; hardwareConfig.gripClosed=30;
    initializeArm(); advance(); CHECK(attachedPins.size()==6); servoWrites.clear();
    hardwareConfig.homeValid=true; hardwareConfig.home=commandedPose();
    hardwareConfig.home.angles[Shoulder]=110;
    CHECK(startHome()); finishTask(); CHECK(commandedPose().angles[Shoulder]==110);
    Pose poses[PICK_STAGE_COUNT]; for(uint8_t i=0;i<PICK_STAGE_COUNT;++i) { poses[i]=commandedPose(); poses[i].angles[Shoulder]=110+(i%2)*5; }
    setSyntheticPickProgram(0,poses); setSyntheticPickProgram(1,poses); setSyntheticPickProgram(2,poses);
    CHECK(onButtonPressed(1)); CHECK(!onButtonPressed(1)); finishTask();
    CHECK(onButtonPressed(1)); finishTask(); CHECK(onButtonPressed(1)); finishTask();
    CHECK(commandedPose().angles[Shoulder]==115);
    CHECK(cancelAndReturnHome()); finishTask(); CHECK(commandedPose().angles[Shoulder]==110);
}
}
int main() {
    for(uint8_t i=0;i<JointCount;++i) original[i]=joints[i];
    testSafety(); testMath(); testLine(); testParser(); testHoldAndJoystick(); test3DAndManual(); testIndependentSpeedAndClock();
    testRecordingAndButtons(); testHomeAndPick();
    std::cout << "PASS: " << assertions << " checks\n";
}
