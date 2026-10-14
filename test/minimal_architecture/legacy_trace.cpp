#include "Arduino.h"
#include "arm.h"
#include "motion.h"
#include <iostream>
uint32_t simulatedTime=0;
std::vector<ServoWrite> servoWrites;
std::vector<int> attachedPins;
std::vector<uint32_t> delayCalls;
int analogValues[4]={506,519,515,519};
SerialStub Serial;
unsigned cost=0;
IKResult timedIK(float x,float z) { IKResult r=solveIK(x,z); simulatedTime+=cost; return r; }
#define solveIK timedIK
#include "motion.cpp"
#undef solveIK
int main(int argc,char**) {
    cost=argc>1?5:0;
    joints[Shoulder].commandedAngle=joints[Shoulder].goalAngle=120;
    joints[Elbow].commandedAngle=joints[Elbow].goalAngle=60;
    initializeArm(); applyPendingPose();
    Point2D start=solveFK(120,60).position;
    if(!startLine({start.x+20,start.z},TaskKind::LineExecution)) return 1;
    while(motionStatus().state==MotionState::Validating) updateMotion(simulatedTime,false);
    uint32_t begin=simulatedTime,next=begin+20;
    size_t previous=servoWrites.size();
    for(int guard=0;isMotionBusy() && guard<1000;++guard) {
        simulatedTime=next; next+=20; updateMotion(simulatedTime,true);
        if(servoWrites.size()!=previous) {
            Pose p=commandedPose();
            std::cout<<"OLD,"<<cost<<','<<simulatedTime-begin<<','<<p.angles[1]<<','<<p.angles[2]<<','
                     <<servoWrites[servoWrites.size()-2].angle<<','<<servoWrites.back().angle<<'\n';
            previous=servoWrites.size();
        }
    }
    std::cout<<"OLD total="<<simulatedTime-begin<<" cost="<<cost<<'\n';
    return isMotionBusy()?2:0;
}
