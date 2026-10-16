// Exercise actual endpoint/output-acknowledgement and existing dwell owner.
#define FIELD_LIBRARY
#include "field_tests.cpp"
void tickVertex(){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);}
void beginVertex(int kind,float grip=85,uint8_t vias=0){
 reset();heights();hardwareConfig.homeViaCount=vias;for(uint8_t i=0;i<vias;++i)hardwareConfig.homeVia[i]=hardwareConfig.home;
 Pose p=hardwareConfig.home;p.angles[3]=grip;setCommand(p);servoWrites.clear();Serial.output.clear();
 CHECK(kind==0?startDrawLine(fieldPoints[0],fieldPoints[1]):startBasicShape(ShapeId::N,{106.56f,-29.16f},{159.81f,26.37f}));preflight();
}
std::vector<uint16_t> vertexIndices(){std::vector<uint16_t> result;result.push_back(hardwareConfig.homeViaCount+4+task.recoveryLift);for(uint8_t i=0;i<drawCount()-1;++i)result.push_back(result.back()+strokeSteps(i));return result;}
void checkAll(int kind,float grip,uint8_t vias){
 beginVertex(kind,grip,vias);auto expected=vertexIndices();size_t seen=0;uint32_t heldAt=0;uint16_t expectedHold=0;Pose held{};
 for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){
  simulatedTime+=20;uint16_t old=task.dwellRemaining,complete=task.status.completedWaypoints;updateMotion(simulatedTime,true);
  // A new endpoint request cannot start a dwell before arm accepts it.
  bool queued=task.endpointQueued&&hasPendingPose();if(queued)CHECK(task.dwellRemaining==0);
  CHECK(applyPendingPose());Pose accepted=commandedPose();updateMotion(simulatedTime,false);
  if(!old&&task.dwellRemaining){
   CHECK(seen<expected.size()&&task.status.completedWaypoints==expected[seen]);expectedHold=seen==0?500:1000;CHECK(queued&&task.dwellRemaining==expectedHold);CHECK(task.status.completedWaypoints==complete+1);
   Pose target;CHECK(prepareOutputPose(task.segmentTarget,target));CHECK(maxDifference(target,accepted)==0);
   Point3D q=fk(task.segmentTarget);TeachPoint v;CHECK(drawVertex(seen,v));CHECK(pointDistance(v,{q.x,q.y})<.05f&&fabsf(q.z-teachSettings().drawZ)<.05f);
   held=accepted;heldAt=simulatedTime;++seen;
  }else if(old){CHECK(task.status.completedWaypoints==complete);CHECK(maxDifference(commandedPose(),held)==0);if(!task.dwellRemaining)CHECK(simulatedTime-heldAt==expectedHold);}
 }
 CHECK(!isMotionBusy()&&!isArmHeld()&&seen==(kind?4:2));CHECK(maxDifference(commandedPose(),hardwareConfig.home)==0);
 std::cout<<"VERTICES kind="<<kind<<" grip="<<grip<<" vias="<<int(vias)<<" holds="<<seen<<" first=500 others=1000 PASS\n";
}
void waitVertex(unsigned vertex){unsigned seen=0;for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){uint16_t old=task.dwellRemaining;tickVertex();if(!old&&task.dwellRemaining&&++seen==vertex)return;}CHECK(false);}
void finishVertex(){for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard)tickVertex();CHECK(!isMotionBusy()&&!isArmHeld());}
void pauseVertex(int kind,unsigned vertex){
 beginVertex(kind);waitVertex(vertex);uint16_t complete=task.status.completedWaypoints;uint16_t remaining=task.dwellRemaining-140;for(unsigned i=0;i<7;++i)tickVertex();CHECK(task.dwellRemaining==remaining);
 send("HOLD");size_t writes=servoWrites.size();for(unsigned i=0;i<100;++i){tickVertex();CHECK(task.dwellRemaining==remaining&&task.status.state==MotionState::Paused);CHECK(task.status.completedWaypoints==complete&&servoWrites.size()==writes);}
 send("GO");uint32_t start=simulatedTime;for(unsigned i=0;task.dwellRemaining&&i<100;++i)tickVertex();CHECK(!task.dwellRemaining&&simulatedTime-start==remaining);finishVertex();
}
void cancelVertex(int kind,unsigned vertex,bool paused){
 beginVertex(kind);waitVertex(vertex);if(paused)send("HOLD");send(paused?"CANCEL":"CANCEL");CHECK(task.status.kind==TaskKind::Home&&!drawingTaskActive());finishVertex();CHECK(maxDifference(commandedPose(),hardwareConfig.home)==0);
 size_t writes=servoWrites.size();send("GO");for(unsigned i=0;i<60;++i)tickVertex();CHECK(servoWrites.size()==writes&&!isMotionBusy());CHECK(Serial.output.find("DONE DRAW ")==std::string::npos);
}
void pendingAck(int kind){
 beginVertex(kind);auto at=vertexIndices();for(unsigned guard=0;guard<10000;++guard){simulatedTime+=20;updateMotion(simulatedTime,true);if(task.endpointQueued&&task.status.completedWaypoints+1==at[0]){CHECK(hasPendingPose()&&!task.dwellRemaining);for(unsigned i=0;i<3;++i){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(task.endpointQueued&&hasPendingPose()&&!task.dwellRemaining);}CHECK(applyPendingPose());updateMotion(simulatedTime,false);CHECK(task.dwellRemaining==500);finishVertex();return;}CHECK(applyPendingPose());updateMotion(simulatedTime,false);}CHECK(false);
}
void dwellClock(){
 beginVertex(0);waitVertex(1);uint16_t remaining=task.dwellRemaining;simulatedTime+=5000;updateMotion(simulatedTime,true);CHECK(task.dwellRemaining==remaining-40);CHECK(applyPendingPose());updateMotion(simulatedTime,false);finishVertex();
 beginVertex(0);while(task.status.state==MotionState::Validating)updateMotion(simulatedTime,false);simulatedTime=UINT32_MAX-100;task.lastUpdate=simulatedTime;waitVertex(1);for(unsigned i=0;i<25;++i)tickVertex();CHECK(task.dwellRemaining==0);finishVertex();
}
void otherShapes(){
 for(ShapeId shape:{ShapeId::Triangle,ShapeId::Z,ShapeId::V}){reset();heights();setCommand(hardwareConfig.home);CHECK(!startBasicShape(shape,{106.56f,-29.16f},{159.81f,26.37f}));CHECK(lastControlError()==ControlError::Invalid&&!isMotionBusy());}
 std::cout<<"Retired SHAPE kinds reject without motion PASS\n";
}
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];CHECK(MEARM_POLY_HOLD_MS==500);CHECK(kDrawSpeedMmS==80&&kFivePointDrawSpeedMmS==80);
 for(int kind=0;kind<2;++kind){for(float grip:{85.f,40.f})for(uint8_t vias=0;vias<3;++vias)checkAll(kind,grip,vias);pendingAck(kind);for(unsigned i=1;i<=unsigned(kind?4:2);++i){pauseVertex(kind,i);cancelVertex(kind,i,false);cancelVertex(kind,i,true);}}
 dwellClock();otherShapes();std::cout<<"PASS vertex dwell "<<assertions<<'\n';
}
