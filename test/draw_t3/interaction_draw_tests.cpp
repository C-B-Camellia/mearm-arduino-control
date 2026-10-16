#define FIELD_LIBRARY
#include "field_tests.cpp"
struct Output { uint32_t ms; uint16_t waypoint,hold; Pose p; };
std::vector<Output> trace(int kind,float grip) {
 reset();heights();Pose initial=hardwareConfig.home;initial.angles[3]=grip;setCommand(initial);
 const char* commands[]={"DRAW LINE","DRAW N","DRAW POLY","DRAW CURVE"};
 Serial.output.clear();servoWrites.clear();send(commands[kind]);CHECK(Serial.output.find(std::string("OK ")+commands[kind]+"\r\n")!=std::string::npos);CHECK(drawingTaskActive());
 bool prep=grip!=85;CHECK(task.recoveryLift==prep);preflight();CHECK(servoWrites.empty());
 std::vector<Output> result;uint32_t start=simulatedTime;bool prepared=!prep;unsigned holds=0;
 for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){
  simulatedTime+=20;auto before=task.status.completedWaypoints;auto oldHold=task.dwellRemaining;
  updateMotion(simulatedTime,true);CHECK(applyPendingPose());Pose actual=commandedPose();CHECK(isSafePose(actual));
  if(prep&&before==0)for(int j=0;j<3;++j)CHECK(actual.angles[j]==initial.angles[j]);
  updateMotion(simulatedTime,false);
  if(!prepared&&task.status.completedWaypoints==1){CHECK(actual.angles[3]==85);prepared=true;start=simulatedTime;continue;}
  if(!prepared)continue;
  if(!oldHold&&task.dwellRemaining){++holds;CHECK(task.dwellRemaining==(holds==1?500:kind==2?MEARM_POLY_HOLD_MS:1000));}
  result.push_back({simulatedTime-start,static_cast<uint16_t>(task.status.completedWaypoints-prep),task.dwellRemaining,actual});
 }
 CHECK(!isMotionBusy()&&!isArmHeld());CHECK(maxDifference(commandedPose(),hardwareConfig.home)==0);CHECK(holds==(kind==0?2:kind==1?4:kind==2?4:1));
 const char* done[]={"DONE DRAW LINE","DONE DRAW N","DONE DRAW POLY","DONE DRAW CURVE"};CHECK(Serial.output.find(done[kind])!=std::string::npos);
 return result;
}
void prepStart(){reset();heights();Pose p=hardwareConfig.home;p.angles[3]=40;setCommand(p);CHECK(startPolyline());preflight();}
void tick(){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);}
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];
 for(int kind=0;kind<4;++kind){auto normal=trace(kind,85),prepared=trace(kind,40);CHECK(normal.size()==prepared.size());for(size_t i=0;i<normal.size();++i){CHECK(normal[i].ms==prepared[i].ms);CHECK(normal[i].waypoint==prepared[i].waypoint);CHECK(normal[i].hold==prepared[i].hold);CHECK(maxDifference(normal[i].p,prepared[i].p)==0);}std::cout<<"PREP identical subsequent sequence kind="<<kind<<" ticks="<<normal.size()<<'\n';}
 prepStart();tick();CHECK(task.status.completedWaypoints==0&&commandedPose().angles[3]>40&&commandedPose().angles[3]<85);send("HOLD");size_t n=servoWrites.size();for(int i=0;i<20;++i)tick();CHECK(servoWrites.size()==n&&task.status.state==MotionState::Paused);send("GO");for(int i=0;isMotionBusy()&&i<10000;++i)tick();CHECK(!isMotionBusy()&&!isArmHeld());
 for(const char* cancel:{"CANCEL","CANCEL"}){prepStart();tick();send(cancel);CHECK(!drawingTaskActive()&&task.status.kind==TaskKind::Home);for(int i=0;isMotionBusy()&&i<10000;++i)tick();CHECK(!isMotionBusy()&&!isArmHeld());CHECK(maxDifference(commandedPose(),hardwareConfig.home)==0);n=servoWrites.size();send("GO");for(int i=0;i<100;++i)tick();CHECK(servoWrites.size()==n);}
 // Whole-path failure must not even execute the otherwise valid grip prep.
 reset();heights();Pose p=hardwareConfig.home;p.angles[3]=40;setCommand(p);servoWrites.clear();fieldShift=300;CHECK(startCurve());while(task.status.state==MotionState::Validating)updateMotion(simulatedTime,false);CHECK(isArmHeld()&&servoWrites.empty()&&lastControlError()==ControlError::Unreachable);fieldShift=0;
 reset();heights();hardwareConfig.home.angles[3]=84;CHECK(!startPolyline()&&lastControlError()==ControlError::DrawGrip);CHECK(!isMotionBusy());
 // A valid start request whose grip limit becomes invalid before preflight is held.
 prepStart();cancelMotion();resumeArm();p=commandedPose();p.angles[3]=40;setCommand(p);CHECK(startPolyline());joints[Gripper].maxAngle=84;servoWrites.clear();while(task.status.state==MotionState::Validating)updateMotion(simulatedTime,false);CHECK(lastControlError()==ControlError::DrawGrip&&isArmHeld()&&servoWrites.empty());
 std::cout<<"PASS DRAW prepare "<<assertions<<'\n';
}
