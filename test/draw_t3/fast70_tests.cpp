#define FIELD_LIBRARY
#include "field_tests.cpp"
void prepare(int kind){
 reset();heights();setCommand(hardwareConfig.home);
 TeachPoint a=fieldPoints[0],b=fieldPoints[1];float length=pointDistance(a,b);b={a.x+(b.x-a.x)*20/length,a.y+(b.y-a.y)*20/length};
 CHECK(kind==0?startDrawLine(a,b):kind==1?startBasicShape(ShapeId::N,{106.56f,-29.16f},{159.81f,26.37f}):kind==2?startPolyline():startCurve());preflight();
}
void measure70(int kind){
 prepare(kind);uint16_t down=4,up=task.status.totalWaypoints-1;
 uint16_t knots[5]={down};if(kind>=2)for(int i=1;i<5;++i)knots[i]=knots[i-1]+strokeSteps(i-1);
 unsigned seen=0,frames=0,holds=0;float maximum=0,strokeMaximum=0;uint32_t start=simulatedTime,strokeStart=0,strokeEnd=0,holdStart=0;Pose previous=commandedPose();
 for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){
  simulatedTime+=20;uint16_t before=task.status.completedWaypoints;size_t n=servoWrites.size();uint32_t oldDwell=task.dwellRemaining;
  updateMotion(simulatedTime,true);CHECK(applyPendingPose());Pose actual=commandedPose();
  CHECK(isSafePose(actual));CHECK(servoWrites.size()==n||servoWrites.size()==n+4);
  if(servoWrites.size()!=n){++frames;float diff=maxDifference(previous,actual);maximum=max(maximum,diff);if(before>=down && before<up-1)strokeMaximum=max(strokeMaximum,diff);
   std::cout<<"OUT "<<kind<<' '<<simulatedTime-start<<' '<<before;for(float j:actual.angles)std::cout<<' '<<j;std::cout<<'\n';}
  previous=actual;
  if(task.endpointQueued){CHECK(maxDifference(goalPose(),task.segmentTarget)<.001f);
   if(kind>=2)for(int i=0;i<5;++i)if(task.status.completedWaypoints+1==knots[i]){
    Pose expected;CHECK(prepareOutputPose(task.segmentTarget,expected));CHECK(maxDifference(actual,expected)==0);
    Point3D xyz=fk(task.segmentTarget);CHECK(pointDistance({xyz.x,xyz.y},fieldPoints[i])<.05f);CHECK(fabsf(xyz.z-teachSettings().drawZ)<.05f);
    seen|=1<<i;std::cout<<"KNOT "<<kind<<' '<<i+1;for(float j:actual.angles)std::cout<<' '<<j;std::cout<<'\n';
   }
  }
  updateMotion(simulatedTime,false);uint16_t after=task.status.completedWaypoints;
  if(before<down && after>=down)strokeStart=simulatedTime;
  if(before<up-1 && after>=up-1)strokeEnd=simulatedTime;
  if(!oldDwell && task.dwellRemaining){++holds;holdStart=simulatedTime;CHECK(task.dwellRemaining==(holds==1?500:kind==2?MEARM_POLY_HOLD_MS:1000));}
  if(oldDwell && !task.dwellRemaining){CHECK(simulatedTime-holdStart==(holds==1?500:kind==2?MEARM_POLY_HOLD_MS:1000));std::cout<<"HOLD "<<kind<<" ms="<<simulatedTime-holdStart<<'\n';}
 }
 CHECK(!isMotionBusy()&&!isArmHeld());CHECK(holds==(kind==0?2:kind==1?4:kind==2?4:1));if(kind>=2)CHECK(seen==31);
 std::cout<<"METRIC kind="<<kind<<" total="<<simulatedTime-start<<" stroke="<<strokeEnd-strokeStart<<" frames="<<frames<<" maxDelta="<<maximum<<" strokeMaxDelta="<<strokeMaximum<<" holds="<<holds<<" knots="<<seen<<'\n';
}
void interruption(int kind,bool cancel){
 prepare(kind);bool done=false;size_t frozen=0;
 for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){
  simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);
  bool ready=task.status.completedWaypoints>6;
  if(kind==2 && !cancel)ready=task.dwellRemaining>0;
  if(!done&&ready){
   CHECK(dispatchAction({AppAction::Hold,0}));frozen=servoWrites.size();simulatedTime+=1000;updateMotion(simulatedTime,true);CHECK(applyPendingPose());CHECK(servoWrites.size()==frozen);
   if(cancel)CHECK(dispatchAction({AppAction::Cancel,0}));else CHECK(dispatchAction({AppAction::Go,0}));done=true;
  }
 }
 CHECK(done&&!isMotionBusy()&&!isArmHeld());CHECK(maxDifference(commandedPose(),hardwareConfig.home)==0);
 std::cout<<"INTERRUPT kind="<<kind<<" cancel="<<cancel<<" PASS\n";
}
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];
 for(int kind=0;kind<4;++kind)measure70(kind);
 for(int kind=2;kind<4;++kind){interruption(kind,false);interruption(kind,true);}
 std::cout<<"PASS fast35/70 comparison "<<assertions<<'\n';}
