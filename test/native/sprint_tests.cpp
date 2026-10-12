#include "Arduino.h"
#include "arm.h"
#include "commands.h"
#include "config.h"
#include "joystick.h"
#include "kinematics.h"
#include "manual_control.h"
#include "motion.h"
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
    analogValues[0]=506; analogValues[1]=519; analogValues[2]=515; analogValues[3]=519;
    simulatedTime=0; servoWrites.clear(); attachedPins.clear(); delayCalls.clear();
    Serial.input.clear(); Serial.output.clear(); setup(); advance();
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
    CHECK(!applyPoseImmediate(candidate)); CHECK(servoWrites.empty() && !hasPendingPose());
    CHECK(commandedPose().angles[Shoulder]==before.angles[Shoulder]);
    candidate.angles[Shoulder]=20; candidate.angles[Elbow]=128;
    CHECK(prepareOutputPose(candidate,output)); CHECK(applyPoseImmediate(candidate)); checkWritesSafe();
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
    fresh(); CHECK(!isArmHeld()); CHECK(setControlMode(ControlMode::Cartesian));
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
}
int main() {
    for(uint8_t i=0;i<JointCount;++i) original[i]=joints[i];
    testSafety(); testMath(); testLine(); testParser(); testHoldAndJoystick();
    std::cout << "PASS: " << assertions << " checks\n";
}
