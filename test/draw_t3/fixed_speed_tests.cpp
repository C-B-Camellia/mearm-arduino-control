#define FIELD_LIBRARY
#include "field_tests.cpp"
void timing(int kind){
 reset();heights();setCommand(hardwareConfig.home);
 CHECK(kind==0?startDrawLine(fieldPoints[0],fieldPoints[1]):kind==1?startBasicShape(ShapeId::N,{106.56f,-29.16f},{159.81f,26.37f}):kind==2?startPolyline():startCurve());preflight();
 const uint16_t down=hardwareConfig.homeViaCount+4,up=task.status.totalWaypoints-hardwareConfig.homeViaCount-1;
 unsigned penChecks=0,strokeChecks=0,airChecks=0;uint16_t inspected=0;uint32_t start=simulatedTime;
 for(unsigned guard=0;isMotionBusy()&&guard<60000;++guard){
  simulatedTime+=1;updateMotion(simulatedTime,true);
  if(task.segmentLoaded && task.status.completedWaypoints+1!=inspected){
   uint16_t index=task.status.completedWaypoints+1;inspected=index;bool stroke=index>down&&index<up;
   uint32_t expected=stroke?1:WAYPOINT_DELAY_MS;
   bool bypass=false;
#ifdef MEARM_FIVE_FAST
   bypass=stroke && kind>=2;
#endif
   float deg=maxDifference(task.segmentStart,task.segmentTarget);
   if(!bypass)expected=max(expected,uint32_t(ceilf(deg*1000/DEFAULT_MOVE_SPEED)));
   if(index>=down && index<=up){
    float speed=stroke?(kind>=2?kFivePointDrawSpeedMmS:kDrawSpeedMmS):(index==down?37.5f:25.0f);
    expected=max(expected,uint32_t(ceilf(task.segmentDistance*1000/speed)));
    if(stroke)++strokeChecks;else ++penChecks;
   }else ++airChecks;
   CHECK(task.duration==expected);
   std::cout<<"PLAN kind="<<kind<<" index="<<index<<" stroke="<<stroke<<" dist="<<task.segmentDistance<<" degrees="<<deg<<" ms="<<task.duration<<'\n';
  }
  CHECK(applyPendingPose());updateMotion(simulatedTime,false);
 }
 CHECK(!isMotionBusy()&&!isArmHeld());CHECK(penChecks==2);CHECK(strokeChecks>0&&airChecks>=4);
 std::cout<<"PLAN_DONE kind="<<kind<<" speed="<<(kind>=2?kFivePointDrawSpeedMmS:kDrawSpeedMmS)<<" totalVirtualMs="<<simulatedTime-start<<'\n';
}
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];
 CHECK(kPenTransitionSpeedMmS==25.0f);
 for(int i=0;i<4;++i)timing(i);
 std::cout<<"PASS fixed drawing timing "<<assertions<<'\n';}
