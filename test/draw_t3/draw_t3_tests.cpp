// Desktop diagnostic only: includes the current executor to observe private
// phase targets/durations. run.ps1 excludes motion.cpp from the other sources.
#include "Arduino.h"
#include "EEPROM.h"
#include "commands.h"
#include "manual_control.h"
#include "control_settings.h"
#include "teaching.h"
#include "app_controller.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
uint32_t simulatedTime=0;
std::vector<ServoWrite> servoWrites;
std::vector<int> attachedPins;
std::vector<uint32_t> delayCalls;
int analogValues[4]={506,519,515,519};
SerialStub Serial;
EEPROMStub EEPROM;
void setup();
void loop();
#include "../../src/motion.cpp"
namespace {
int assertions=0;
HardwareConfig measuredConfig;
JointControl measuredJoints[4];
void require(bool value,const char* expression,int line) {
    ++assertions; if(!value) { std::cerr<<"FAIL "<<line<<": "<<expression<<'\n'; std::exit(1); }
}
#define CHECK(e) require((e),#e,__LINE__)
Point3D fk(const Pose& p) { return solveFK3D(p.angles[0],p.angles[1],p.angles[2]); }
void send(const std::string& text) { Serial.input+=text+'\n'; while(!Serial.input.empty()) pollSerial(); }
void setCommand(const Pose& p) { CHECK(requestPose(p)); CHECK(applyPendingPose()); syncManualTarget(simulatedTime); }
void reset() {
    cancelMotion(); hardwareConfig=measuredConfig;
    for(int i=0;i<4;++i) joints[i]=measuredJoints[i];
    EEPROM.reset(); simulatedTime=0; resetControlSettings();
    Serial.input.clear(); Serial.output.clear(); servoWrites.clear(); setup();
    simulatedTime+=20; loop(); setCommand(hardwareConfig.home);
    CHECK(beginTeach()); CHECK(captureTeachHeight(false));
}
Pose lowerPose(float shoulder) { Pose p=hardwareConfig.home; p.angles[Shoulder]=shoulder; return p; }
const char* phase(uint16_t i,uint16_t stroke) {
    if(i==1) return "HOME"; if(i==2) return "HOME_TO_TRAVEL_Z";
    if(i==3) return "TRAVEL_TO_P1"; if(i==4) return "PEN_DOWN";
    if(i<=4+stroke) return "STROKE"; if(i==5+stroke) return "PEN_UP";
    return "RETURN_HOME";
}
void compareCaptures() {
    reset(); setCommand(lowerPose(118)); send("TEACH DRAWZ");
    float first=teachSettings().drawZ; CHECK(fabs(first-fk(commandedPose()).z)<0.0001f);
    // Capture is commandedPose, even when goal differs and has been discarded.
    CHECK(requestPose(lowerPose(117))); float old=teachSettings().drawZ;
    CHECK(!captureTeachHeight(true)); CHECK(teachSettings().drawZ==old);
    holdArm(); resumeArm(); setCommand(lowerPose(116)); send("TEACH DRAWZ");
    float second=teachSettings().drawZ; CHECK(second<first); CHECK(drawSettingsValid());
    Point3D xy=fk(hardwareConfig.home);
    CHECK(startDrawLine({xy.x,xy.y},{xy.x,xy.y+6}));
    Pose firstGoal=commandedPose(),secondGoal=commandedPose();
    CHECK(drawGoal(4,secondGoal));
    CHECK(fabs(fk(secondGoal).z-second)<0.05f);
    // Observe both captured heights through the real sampler, not a duplicated IK.
    cancelMotion(); resumeArm(); setCommand(lowerPose(118)); CHECK(captureTeachHeight(true));
    CHECK(startDrawLine({xy.x,xy.y},{xy.x,xy.y+6})); CHECK(drawGoal(4,firstGoal));
    CHECK(fabs(fk(firstGoal).z-first)<0.05f); CHECK(fk(secondGoal).z<fk(firstGoal).z);
    Pose qa,qb; CHECK(prepareOutputPose(firstGoal,qa)); CHECK(prepareOutputPose(secondGoal,qb));
    std::cout<<"CAPTURE A z="<<first<<" PEN_DOWN J="<<firstGoal.angles[1]<<','<<firstGoal.angles[2]<<" quantZ="<<fk(qa).z
             <<"; B z="<<second<<" J="<<secondGoal.angles[1]<<','<<secondGoal.angles[2]<<" quantZ="<<fk(qb).z<<'\n';
    CHECK(!captureTeachHeight(true)); CHECK(teachSettings().drawZ==first); // Busy preserves old height.
    cancelMotion(); resumeArm(); setCommand(hardwareConfig.home); CHECK(captureTeachAlignment()); CHECK(beginMarking());
    send("TEACH DRAWZ"); CHECK(lastControlError()==ControlError::TeachState); CHECK(teachSettings().drawZ==first);
    std::cout<<"REJECT marking retained="<<teachSettings().drawZ<<" output="<<Serial.output.substr(Serial.output.size()-13)<<'\n';
}
void quantizationScan() {
    reset(); Point3D center=fk(hardwareConfig.home);
    bool hadPrevious=false; float previousFloat=0,previousQuant=0; int reversals=0,plateaus=0,valid=0;
    float largest=0,previousInput=0; float witnessHigh=0,witnessLow=0,witnessQHigh=0,witnessQLow=0;
    for(int i=0;i<=2500;++i) {
        Point3D p=center; p.z=center.z+5-i*0.01f;
        IKResult3D ik=solveIK3D(p); if(!ik.valid) { hadPrevious=false; continue; }
        Pose pose=commandedPose(),q; pose.angles[0]=ik.j0; pose.angles[1]=ik.j1; pose.angles[2]=ik.j2;
        if(!prepareOutputPose(pose,q)) { hadPrevious=false; continue; }
        float f=fk(pose).z,quant=fk(q).z; CHECK(fabs(f-p.z)<0.05f); ++valid;
        if(hadPrevious) {
            CHECK(f<previousFloat+0.0001f);
            if(fabs(quant-previousQuant)<0.00001f) ++plateaus;
            if(quant>previousQuant+0.0001f) {
                ++reversals; if(quant-previousQuant>largest) { largest=quant-previousQuant; witnessHigh=previousInput; witnessLow=p.z; witnessQHigh=previousQuant; witnessQLow=quant; }
            }
        }
        hadPrevious=true; previousFloat=f; previousQuant=quant; previousInput=p.z;
    }
    CHECK(valid>1000); CHECK(reversals>0); CHECK(plateaus>0);
    std::cout<<"Z_SCAN sameXY="<<center.x<<','<<center.y<<" valid="<<valid<<" plateaus="<<plateaus<<" reversals="<<reversals<<" max_up_step="<<largest
             <<" witness target="<<witnessHigh<<" -> "<<witnessLow<<" quantFK="<<witnessQHigh<<" -> "<<witnessQLow<<'\n';
}
void loggedEvidenceAndRoundTrip() {
    reset();
    const Pose logged[]={{{80,92,63,90}},{{88,85,78,90}},{{88,85,78,85}},{{96,92,115,89}}};
    for(const Pose& p:logged) { Point3D v=fk(p); std::cout<<"LOGGED_STATUS_FK J="<<p.angles[0]<<'/'<<p.angles[1]<<'/'<<p.angles[2]<<'/'<<p.angles[3]<<" XYZ="<<v.x<<'/'<<v.y<<'/'<<v.z<<" (capture time unknown)\n"; }
    Point3D p=fk(hardwareConfig.home); IKResult3D ik=solveIK3D(p); CHECK(ik.valid);
    Pose target=hardwareConfig.home,quant; target.angles[0]=ik.j0; target.angles[1]=ik.j1; target.angles[2]=ik.j2;
    CHECK(prepareOutputPose(target,quant));
    std::cout<<"IDENTITY homeJ="<<hardwareConfig.home.angles[0]<<'/'<<hardwareConfig.home.angles[1]<<'/'<<hardwareConfig.home.angles[2]
             <<" IKJ="<<ik.j0<<'/'<<ik.j1<<'/'<<ik.j2<<" quantJ="<<quant.angles[0]<<'/'<<quant.angles[1]<<'/'<<quant.angles[2]<<" FKz="<<p.z<<" -> "<<fk(quant).z<<'\n';
    float dx=169.62f-144.77f,dy=-6.63f-32.18f;
    float length=sqrtf(dx*dx+dy*dy); uint16_t n=static_cast<uint16_t>(ceilf(length/.75f));
    CHECK(MANUAL_CART_SLOW==25.0f && MANUAL_CART_FAST==100.0f); // Updated authorized manual tiers; old RC0 sampling ceiling is not a current invariant.
    std::cout<<"LOGGED_P1_P2 illustrativeOnly length="<<length<<" samples="<<n<<" nominal15="<<length/15<<" nominal25="<<length/25<<'\n';
}
void trace(float feed,uint32_t gap,bool fastAck) {
    reset(); setCommand(lowerPose(116)); CHECK(captureTeachHeight(true));
    CHECK(feed==kDrawSpeedMmS); Point3D a=fk(hardwareConfig.home); TeachPoint b{a.x,a.y+30};
    CHECK(startDrawLine({a.x,a.y},b)); uint16_t stroke=strokeSteps(0);
    int validations=0;
    while(motionStatus().state==MotionState::Validating) { updateMotion(simulatedTime,false); CHECK(!hasPendingPose()); CHECK(++validations<20000); }
    CHECK(motionStatus().state==MotionState::Running); CHECK(Serial.output.find("PATH VALID")!=std::string::npos);
    CHECK(motionStatus().completedWaypoints==0);
    std::cout<<"TRACE feed="<<feed<<" tickGap="<<gap<<" fastAck="<<fastAck<<" sampleStroke="<<stroke<<" validateCalls="<<validations<<" drawZ="<<teachSettings().drawZ<<" travelZ="<<teachSettings().travelZ<<'\n';
    uint32_t start=simulatedTime,strokeStart=0,strokeEnd=0,sameFrames=0; Pose previousOutput=commandedPose();
    for(int count=0;isMotionBusy() && count<100000;++count) {
        uint16_t before=motionStatus().completedWaypoints;
        simulatedTime+=gap; updateMotion(simulatedTime,true);
        bool pending=hasPendingPose(); CHECK(applyPendingPose());
        if(pending) {
            Pose now=commandedPose(); bool same=true; for(int j=0;j<4;++j) if(now.angles[j]!=previousOutput.angles[j]) same=false;
            if(same) ++sameFrames; previousOutput=now;
        }
        if(fastAck) updateMotion(simulatedTime,false);
        uint16_t after=motionStatus().completedWaypoints;
        if(before<4 && after>=4) strokeStart=simulatedTime;
        if(before<4+stroke && after>=4+stroke) strokeEnd=simulatedTime;
    }
    CHECK(!isMotionBusy()); CHECK(Serial.output.find("DONE DRAW LINE")!=std::string::npos);
    std::cout<<"TOTAL ms="<<simulatedTime-start<<" strokeMs="<<strokeEnd-strokeStart<<" repeatedOutputFrames="<<sameFrames<<'\n';
}
}
int main() {
    measuredConfig=hardwareConfig; for(int i=0;i<4;++i) measuredJoints[i]=joints[i];
    std::cout<<std::fixed<<std::setprecision(5);
    CHECK(std::string(phase(4,40))=="PEN_DOWN");
    compareCaptures(); quantizationScan(); loggedEvidenceAndRoundTrip(); trace(kDrawSpeedMmS,20,true); trace(kDrawSpeedMmS,20,false); trace(kDrawSpeedMmS,100,true);
    std::cout<<"PASS DRAW T3 diagnostic: "<<assertions<<" checks\n";
}