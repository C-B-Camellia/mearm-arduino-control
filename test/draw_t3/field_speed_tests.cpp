#define FIELD_LIBRARY
#include "field_tests.cpp"
void measure(int kind,float feed,bool pause=false) {
 reset();heights();setCommand(hardwareConfig.home);CHECK(feed==kDrawSpeedMmS);
 TeachPoint a=fieldPoints[0],b=fieldPoints[1];float length=pointDistance(a,b);b={a.x+(b.x-a.x)*20/length,a.y+(b.y-a.y)*20/length};
 CHECK(kind==0?startDrawLine(a,b):kind==1?startPolyline():startCurve());preflight();
 uint16_t down=4,up=task.status.totalWaypoints-1;uint32_t start=simulatedTime,strokeStart=0,strokeEnd=0;
 uint32_t firstEnd=0;unsigned physical=0,holds=0;float maximumDelta=0;bool paused=false;
 Pose last=commandedPose();uint16_t firstCorner=down+strokeSteps(0);
 std::cout<<"BEGIN "<<kind<<' '<<feed<<'\n';
 for(unsigned guard=0;isMotionBusy() && guard<10000;++guard){
  simulatedTime+=20;auto before=task.status.completedWaypoints;size_t writes=servoWrites.size();updateMotion(simulatedTime,true);
  bool had=hasPendingPose();Pose goal=goalPose();CHECK(applyPendingPose());Pose actual=commandedPose();
  if(servoWrites.size()!=writes){++physical;maximumDelta=max(maximumDelta,maxDifference(last,actual));}last=actual;
  if(had){std::cout<<"SEQ "<<kind<<' '<<feed<<' '<<simulatedTime-start<<' '<<before;for(float x:actual.angles)std::cout<<' '<<x;std::cout<<'\n';}
  if(task.endpointQueued) CHECK(maxDifference(goal,task.segmentTarget)<.001f);
  updateMotion(simulatedTime,false);auto after=task.status.completedWaypoints;
  if(before<down && after>=down)strokeStart=simulatedTime;
  if(before<up-1 && after>=up-1)strokeEnd=simulatedTime;
  if(before<firstCorner && after>=firstCorner)firstEnd=simulatedTime;
  if(!task.dwellRemaining && before>=down && before<up-1){}
  if(task.dwellRemaining && after!=before){++holds;
#ifdef MEARM_FIVE_FAST
   CHECK(kind==1 && task.dwellRemaining==MEARM_POLY_HOLD_MS);
#else
   CHECK(false);
#endif
  }
  bool pauseHere=before>down;
#ifdef MEARM_FIVE_FAST
  if(kind==1 && MEARM_POLY_HOLD_MS)pauseHere=task.dwellRemaining>0;
#endif
  if(pause && !paused && pauseHere){CHECK(pauseMotion(simulatedTime));size_t n=servoWrites.size();simulatedTime+=1000;updateMotion(simulatedTime,true);CHECK(applyPendingPose());CHECK(servoWrites.size()==n);CHECK(resumeMotion(simulatedTime));paused=true;}
 }
 CHECK(!isMotionBusy()&&!isArmHeld());
#ifdef MEARM_FIVE_FAST
 CHECK(holds==(kind==1 && MEARM_POLY_HOLD_MS?3:0));
#else
 CHECK(holds==0);
#endif
 std::cout<<"METRIC kind="<<kind<<" feed="<<feed<<" total="<<simulatedTime-start<<" stroke="<<strokeEnd-strokeStart<<" first="<<firstEnd-strokeStart<<" writes="<<physical<<" maxIntDelta="<<maximumDelta<<" holds="<<holds<<'\n';
}
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];
 for(int kind=0;kind<3;++kind)measure(kind,kDrawSpeedMmS);
 measure(1,kDrawSpeedMmS,true);measure(2,kDrawSpeedMmS,true);
 std::cout<<"PASS field speed "<<assertions<<'\n';}
