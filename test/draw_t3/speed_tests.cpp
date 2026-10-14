// Desktop diagnostic only: includes the unmodified executor to observe private
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

struct RunResult { uint32_t total,stroke; uint16_t samples; uint32_t frames; };
void prepareLine(float feed) {
    reset(); setCommand(lowerPose(116)); CHECK(captureTeachHeight(true));
    CHECK(setDrawingFeed(feed)); Point3D a=fk(hardwareConfig.home);
    CHECK(startDrawLine({a.x,a.y},{a.x,a.y+30}));
}
void prepareFive(float feed,bool curve) {
    reset(); setCommand(lowerPose(116)); CHECK(captureTeachHeight(true));
    setCommand(lowerPose(118)); CHECK(captureTeachAlignment()); CHECK(beginMarking());
    for(int i=0;i<5;++i) { Pose p=commandedPose(); p.angles[Base]=80+i*5; setCommand(p); CHECK(onButtonPressed(1)); }
    CHECK(setDrawingFeed(feed)); CHECK(curve?startCurve():startPolyline());
}
void preflight() {
    size_t before=servoWrites.size(); int calls=0;
    while(motionStatus().state==MotionState::Validating) { updateMotion(simulatedTime,false); CHECK(++calls<100000); }
    CHECK(motionStatus().state==MotionState::Running); CHECK(servoWrites.size()==before);
}
RunResult run(bool pause=false,uint32_t gap=20) {
    preflight();
    uint16_t count=motionStatus().totalWaypoints;
    uint16_t penUp=count-hardwareConfig.homeViaCount-1;
    uint32_t start=simulatedTime,strokeStart=0,strokeEnd=0,pausedMs=0,frames=0;
    Pose prior=goalPose(); bool didPause=false,checkpoint=false;
    for(int loops=0;isMotionBusy() && loops<100000;++loops) {
        simulatedTime+=gap; uint32_t interval=gap;
        uint16_t before=motionStatus().completedWaypoints;
        updateMotion(simulatedTime,true);
        if(hasPendingPose()) {
            Pose goal=goalPose(); float movement=maxDifference(prior,goal);
            CHECK(movement<=DEFAULT_MOVE_SPEED*min(interval,MANUAL_MAX_DT_MS)*.001f+.001f);
            Pose beforeOutput=commandedPose(); Pose expected; CHECK(prepareOutputPose(goal,expected));
            size_t writes=servoWrites.size(); CHECK(applyPendingPose());
            bool changed=maxDifference(beforeOutput,expected)>0;
            CHECK(servoWrites.size()==writes+(changed?4:0));
            CHECK(maxDifference(commandedPose(),expected)==0);
            if(changed) ++frames; prior=goal;
        }
        // Every mandatory checkpoint must reach the output, not just the cursor.
        if(task.endpointQueued) { CHECK(maxDifference(goalPose(),task.segmentTarget)<.001f); checkpoint=true; }
        updateMotion(simulatedTime,false);
        uint16_t after=motionStatus().completedWaypoints;
        if(before<4 && after>=4) strokeStart=simulatedTime;
        if(before<penUp-1 && after>=penUp-1) strokeEnd=simulatedTime;
        if(pause && !didPause && after>4 && after<penUp-1) {
            CHECK(pauseMotion(simulatedTime)); size_t n=servoWrites.size();
            simulatedTime+=5000; updateMotion(simulatedTime,true); CHECK(applyPendingPose());
            CHECK(servoWrites.size()==n); CHECK(resumeMotion(simulatedTime));
            pausedMs=5000; didPause=true;
        }
        CHECK(after>=before); CHECK(after-before<=16);
    }
    CHECK(!isMotionBusy()); CHECK(!isArmHeld()); CHECK(checkpoint);
    CHECK(strokeEnd>strokeStart); if(pause) CHECK(didPause);
    return {simulatedTime-start-pausedMs,strokeEnd-strokeStart-pausedMs,static_cast<uint16_t>(count-6),frames};
}
void speedAndTravel() {
    RunResult r[4]; const float feeds[]={15,25,80,100};
    for(int i=0;i<4;++i) {
        prepareLine(feeds[i]); r[i]=run();
        std::cout<<"LINE feed="<<feeds[i]<<" totalMs="<<r[i].total<<" strokeMs="<<r[i].stroke<<" effective="<<30000.f/r[i].stroke<<" frames="<<r[i].frames<<'\n';
    }
    CHECK(r[0].stroke<=2040); CHECK(r[1].stroke<=1240);
    CHECK(r[2].stroke<600); CHECK(r[3].stroke<=r[2].stroke);
    // Air travel and HOME duration no longer depend on paper feed.
    uint32_t homes[2];
    for(int i=0;i<2;++i) {
        prepareLine(i?100:15); preflight(); simulatedTime+=20; updateMotion(simulatedTime,true);
        homes[i]=task.duration; CHECK(applyPendingPose());
    }
    CHECK(homes[0]==homes[1]);
    prepareLine(80); RunResult paused=run(true); CHECK(paused.stroke==r[2].stroke);
    prepareLine(80); simulatedTime=UINT32_MAX-100; RunResult wrapped=run(); CHECK(wrapped.stroke==r[2].stroke);
    reset(); send("FEED 50"); CHECK(drawingFeed()==50); send("FEED 100"); CHECK(drawingFeed()==100);
    send("FEED 100.01"); CHECK(drawingFeed()==100);
    send("STATUS"); CHECK(Serial.output.find("Z=")!=std::string::npos);
}
void verticesAndCurve() {
    for(int curve=0;curve<2;++curve) {
        prepareFive(80,curve); servoWrites.clear(); RunResult result=run(true);
        std::cout<<(curve?"CURVE":"POLY")<<" feed=80 totalMs="<<result.total<<" strokeMs="<<result.stroke<<" samples="<<result.samples<<" frames="<<result.frames<<'\n';
        for(int i=0;i<5;++i) {
            TeachPoint p; CHECK(getTeachPoint(i,p)); float closest=10000;
            for(size_t k=0;k+3<servoWrites.size();k+=4) {
                Point3D q=solveFK3D(servoWrites[k].angle,servoWrites[k+1].angle,servoWrites[k+2].angle);
                if(fabs(q.z-teachSettings().drawZ)<3) closest=min(closest,pointDistance({q.x,q.y},p));
                CHECK(isSafePlanarPose(servoWrites[k+1].angle,servoWrites[k+2].angle));
            }
            CHECK(closest<3);
        }
    }
    prepareFive(100,true); RunResult loaded=run(false,100);
    std::cout<<"CURVE late-tick totalMs="<<loaded.total<<" strokeMs="<<loaded.stroke<<'\n';
    prepareLine(100); preflight();
    simulatedTime+=20; updateMotion(simulatedTime,true); CHECK(applyPendingPose()); updateMotion(simulatedTime,false);
    CHECK(cancelAndReturnHome());
    while(isMotionBusy()) { simulatedTime+=20; updateMotion(simulatedTime,true); CHECK(applyPendingPose()); updateMotion(simulatedTime,false); }
    CHECK(!isArmHeld()); CHECK(commandedPose().angles[Gripper]==85);
    // Runtime safety is rechecked even for intermediate samples crossed in one tick.
    prepareLine(100); preflight(); joints[Shoulder].minAngle=124;
    servoWrites.clear(); simulatedTime+=20; updateMotion(simulatedTime,true);
    CHECK(!isMotionBusy() && isArmHeld() && servoWrites.empty());
}
}
int main() {
    measuredConfig=hardwareConfig; for(int i=0;i<4;++i) measuredJoints[i]=joints[i];
    speedAndTravel(); verticesAndCurve();
    std::cout<<"PASS DRAW speed: "<<assertions<<" checks\n";
    return 0;
}
