#include "teaching.h"
static const TeachPoint fieldPoints[5]={{106.56f,26.37f},{147.29f,22.12f},{159.81f,-17.72f},{129.64f,-29.16f},{107.31f,-11.54f}};
static bool fieldPoint(uint8_t i,TeachPoint& p) { if(i>=5) return false; p=fieldPoints[i];return true; }
static uint8_t fieldCount() { return 5; }
#define getTeachPoint fieldPoint
#define teachPointCount fieldCount
#define main speedMain
#include "speed_tests.cpp"
#undef main
#undef getTeachPoint
#undef teachPointCount
void heights() { auto& s=const_cast<TeachSettings&>(teachSettings());s={82.53f,94.29f,86.71f,true,true,true}; }
void diagnose(bool curve) {
 reset(); heights(); setCommand(hardwareConfig.home); CHECK(curve?startCurve():startPolyline());
 unsigned calls=0;
 while(task.status.state==MotionState::Validating) {
  auto waypoint=task.status.validatedWaypoints; auto sub=task.validationSubstep;
  updateMotion(simulatedTime,false); ++calls;
  if(isArmHeld()) {
   Pose p=interpolate(task.segmentStart,task.segmentTarget,float(task.validationSubstep)/task.validationSubsteps);Pose q=p;
   for(float& x:q.angles)x=int(x);
   Point3D xyz=fk(task.segmentTarget);
   std::cout<<(curve?"CURVE":"POLY")<<" FAIL wp="<<waypoint+1<<" sub="<<sub<<" calls="<<calls<<" error="<<int(lastControlError())<<" goalXYZ="<<xyz.x<<','<<xyz.y<<','<<xyz.z<<" candidate=";
   for(float x:p.angles)std::cout<<x<<',';
   std::cout<<" floatSafe="<<isSafePose(p)<<" quantSafe="<<isSafePose(q)<<" sum="<<p.angles[1]+p.angles[2]<<'\n';return;
  }
 }
 std::cout<<(curve?"CURVE":"POLY")<<" VALID samples="<<task.status.totalWaypoints<<" calls="<<calls<<'\n';
}
int main(){ measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];
 std::cout<<std::setprecision(9);diagnose(false);diagnose(true);
 reset();setCommand({{74,42,106,58}});holdArm();CHECK(cancelAndReturnHome());
 while(isMotionBusy()){simulatedTime+=20;updateMotion(simulatedTime,true);applyPendingPose();updateMotion(simulatedTime,false);}
 std::cout<<"HOME held="<<isArmHeld()<<" error="<<int(lastControlError())<<" J="<<commandedPose().angles[1]<<','<<commandedPose().angles[2]<<'\n';
 reset();Pose a={{74,42,106,58}},b={{75,43,105,58}},q;unsigned fails=0;
 for(int i=0;i<=100;++i){Pose p=interpolate(a,b,i/100.f);if(!prepareOutputPose(p,q))++fails;}
 std::cout<<"BOUNDARY interpolate failures="<<fails<<"/101\n";
}
