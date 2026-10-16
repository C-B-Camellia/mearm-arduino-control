// Test the actual executor at the requested production parameters.
#define FIELD_LIBRARY
#include "field_tests.cpp"
#include <limits>
void tick80(){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);}
void poly80(){reset();heights();setCommand(hardwareConfig.home);servoWrites.clear();Serial.output.clear();CHECK(startPolyline());preflight();}
void waitCorner(unsigned corner){
 unsigned seen=0;
 for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){auto old=task.dwellRemaining;tick80();if(!old&&task.dwellRemaining&&task.status.completedWaypoints>hardwareConfig.homeViaCount+4+task.recoveryLift&&++seen==corner)return;}
 CHECK(false);
}
void finish80(){for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard)tick80();CHECK(!isMotionBusy()&&!isArmHeld());}
void pauseCorner(unsigned corner){
 poly80();waitCorner(corner);CHECK(task.dwellRemaining==500);auto waypoint=task.status.completedWaypoints;uint32_t start=simulatedTime;
 for(unsigned i=0;i<3;++i)tick80();CHECK(task.dwellRemaining==440);
 send("HOLD");size_t writes=servoWrites.size();Pose held=commandedPose();
 for(unsigned i=0;i<100;++i){tick80();CHECK(task.status.state==MotionState::Paused&&task.dwellRemaining==440);CHECK(task.status.completedWaypoints==waypoint&&servoWrites.size()==writes);CHECK(maxDifference(commandedPose(),held)==0);}
 send("GO");while(task.dwellRemaining)tick80();CHECK(simulatedTime-start==2500);finish80();
 std::cout<<"HOLD corner="<<corner<<" active=500 pause=2000 total=2500 PASS\n";
}
void cancelCorner(unsigned corner,bool paused){
 poly80();waitCorner(corner);if(paused)send("HOLD");send(paused?"CANCEL":"CANCEL");CHECK(!drawingTaskActive()&&task.status.kind==TaskKind::Home);finish80();CHECK(maxDifference(commandedPose(),hardwareConfig.home)==0);
 size_t writes=servoWrites.size();send("GO");for(unsigned i=0;i<50;++i)tick80();CHECK(servoWrites.size()==writes&&!drawingTaskActive());CHECK(Serial.output.find("DONE DRAW POLY")==std::string::npos);
}
void geometryAndSpeed(){
 for(bool curve:{false,true}){
  reset();heights();setCommand(hardwareConfig.home);CHECK(curve?startCurve():startPolyline());preflight();
  unsigned holds=0;uint8_t knots=0;unsigned loaded=0;uint16_t knotIndex=hardwareConfig.homeViaCount+4;
  for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){
   uint16_t before=task.status.completedWaypoints;bool wasLoaded=task.segmentLoaded;uint16_t oldHold=task.dwellRemaining;simulatedTime+=20;updateMotion(simulatedTime,true);
   if(!wasLoaded&&task.segmentLoaded&&before>=4&&before<task.status.totalWaypoints-2){
    CHECK(task.duration==max(1UL,static_cast<unsigned long>(ceilf(task.segmentDistance*1000.f/80.f))));++loaded;
   }
   CHECK(applyPendingPose());
   if(task.endpointQueued){
    uint16_t at=task.status.completedWaypoints+1;knotIndex=hardwareConfig.homeViaCount+4;
    for(uint8_t i=0;i<5;++i){if(i)knotIndex+=strokeSteps(i-1);if(at==knotIndex){TeachPoint expected;CHECK(fieldPoint(i,expected));Point3D q=fk(task.segmentTarget);CHECK(pointDistance(expected,{q.x,q.y})<.05f);CHECK(fabsf(q.z-teachSettings().drawZ)<.05f);knots|=1<<i;}}
   }
   updateMotion(simulatedTime,false);if(!oldHold&&task.dwellRemaining){CHECK(task.dwellRemaining==500);++holds;}
  }
  CHECK(!isMotionBusy()&&!isArmHeld()&&knots==31&&loaded>0);CHECK(holds==(curve?1:4));
  std::cout<<(curve?"CURVE":"POLY")<<" knots=31 holds="<<holds<<" spaceSpeed=80 PASS\n";
 }
 // POLY generation is still a straight affine XY interpolation per segment.
 poly80();for(uint8_t segment=0;segment<4;++segment){TeachPoint a,b;CHECK(fieldPoint(segment,a)&&fieldPoint(segment+1,b));uint16_t index=4;for(uint8_t i=0;i<segment;++i)index+=strokeSteps(i);uint16_t n=strokeSteps(segment);for(uint16_t i=1;i<=n;++i){Pose p;CHECK(sampleGoal(index+i,p));auto q=fk(p);float t=float(i)/n;CHECK(pointDistance({q.x,q.y},{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t})<.05f);}}
 cancelMotion();resumeArm();
}
void tangentGain(){
 TeachPoint a={-1,0},b={0,0},c={1,0},d={1,1},q,nearStart,nearEnd;
 CHECK(centripetalSample(a,b,c,d,0,q)&&q.x==b.x&&q.y==b.y);
 CHECK(centripetalSample(a,b,c,d,1,q)&&q.x==c.x&&q.y==c.y);
 CHECK(centripetalSample(a,b,c,d,.5f,q));CHECK(fabsf(q.x-.575f)<1e-6f&&fabsf(q.y+.075f)<1e-6f);
 CHECK(centripetalSample(a,b,c,d,.001f,nearStart));CHECK(centripetalSample(a,b,c,d,.999f,nearEnd));
 CHECK(fabsf(nearStart.x/.001f-1.2f)<.003f&&fabsf(nearStart.y/.001f)<.003f);
 CHECK(fabsf((1-nearEnd.x)/.001f-.6f)<.003f&&fabsf(-nearEnd.y/.001f-.6f)<.003f);
 CHECK(!centripetalSample(a,b,b,d,.5f,q));CHECK(!centripetalSample(a,b,c,d,-.1f,q));CHECK(!centripetalSample(a,b,c,d,std::numeric_limits<float>::quiet_NaN(),q));
 std::cout<<"Hermite gain=1.20 endpoint/midpoint/tangents/guards PASS\n";
}
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];
 CHECK(kDrawSpeedMmS==80&&kFivePointDrawSpeedMmS==80&&kPenTransitionSpeedMmS==25);CHECK(DEFAULT_MOVE_SPEED==60&&MANUAL_JOINT_SLOW==25&&MANUAL_JOINT_FAST==90&&MANUAL_CART_SLOW==25&&MANUAL_CART_FAST==100);CHECK(MEARM_POLY_HOLD_MS==500);
 tangentGain();geometryAndSpeed();for(unsigned i=1;i<=3;++i){pauseCorner(i);cancelCorner(i,false);cancelCorner(i,true);}std::cout<<"PASS parameter80 "<<assertions<<'\n';
}
