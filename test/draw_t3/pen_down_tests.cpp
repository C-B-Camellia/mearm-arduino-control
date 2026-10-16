#define FIELD_LIBRARY
#include "field_tests.cpp"
#include <cstring>
struct DownTiming{uint32_t oldDuration,newDuration,jointDuration,oldSpace,newSpace;};
DownTiming runDown(const char* label){
 const uint16_t down=hardwareConfig.homeViaCount+4+task.recoveryLift,up=task.status.totalWaypoints-hardwareConfig.homeViaCount-1;
 uint16_t inspected=0;DownTiming result{};bool accepted=false;unsigned holds=0;Pose previous=goalPose();
 for(unsigned guard=0;isMotionBusy()&&guard<10000;++guard){
  simulatedTime+=20;uint16_t oldHold=task.dwellRemaining;updateMotion(simulatedTime,true);uint16_t index=task.status.completedWaypoints+1;
  if(task.segmentLoaded&&index!=inspected){
   inspected=index;bool stroke=index>down&&index<up;bool bypass=stroke&&(task.status.kind==TaskKind::Polyline||task.status.kind==TaskKind::Curve);
   uint32_t joint=ceilf(maxDifference(task.segmentStart,task.segmentTarget)*1000/60.f),expected=stroke?1:10;
   if(!bypass)expected=max(expected,joint);
   if(index>=down&&index<=up){float speed=stroke?80.f:index==down?37.5f:25.f;expected=max(expected,uint32_t(ceilf(task.segmentDistance*1000/speed)));}
   CHECK(task.duration==expected);
   if(index==down){result.jointDuration=joint;result.oldSpace=ceilf(task.segmentDistance*1000/25.f);result.newSpace=ceilf(task.segmentDistance*1000/37.5f);result.oldDuration=max(uint32_t(10),max(joint,result.oldSpace));result.newDuration=task.duration;CHECK(task.duration>=joint&&task.duration<=result.oldDuration);}
  }
  if(task.status.completedWaypoints==down-1&&hasPendingPose())CHECK(maxDifference(previous,goalPose())<=60.f*.020f+.001f);
  if(task.endpointQueued&&index==down)CHECK(hasPendingPose()&&task.dwellRemaining==0);
  previous=goalPose();CHECK(applyPendingPose());updateMotion(simulatedTime,false);
  if(!oldHold&&task.dwellRemaining){++holds;if(task.status.completedWaypoints==down){CHECK(task.dwellRemaining==500);accepted=true;}}
 }
 CHECK(!isMotionBusy()&&!isArmHeld()&&accepted&&result.newDuration>0);CHECK(maxDifference(commandedPose(),hardwareConfig.home)==0);
 std::cout<<"DOWN "<<label<<" old="<<result.oldDuration<<" new="<<result.newDuration<<" joint="<<result.jointDuration<<" space25="<<result.oldSpace<<" space37.5="<<result.newSpace<<" ratio="<<float(result.oldDuration)/result.newDuration<<" holds="<<holds<<'\n';return result;
}
void originalN(){
 const uint8_t codes[4]={0,4,2,6};const TeachPoint wanted[4]={{115,-40},{115,10},{155,-40},{155,10}};
 for(uint8_t i=0;i<4;++i){TeachPoint q;CHECK(shapeVertex(ShapeId::N,i,{115,-40},{155,10},q));CHECK(std::memcmp(&q,&wanted[i],sizeof(q))==0);}
 for(int r=0;r<40;++r){TeachPoint lo={r*.37f-100,r*.23f-70},hi={lo.x+10+r*.17f,lo.y+5+r*.39f};for(uint8_t i=0;i<4;++i){TeachPoint q;CHECK(shapeVertex(ShapeId::N,i,lo,hi,q));float x=(codes[i]&3)*.5f,y=codes[i]>>2;TeachPoint expected={lo.x+x*(hi.x-lo.x),lo.y+y*(hi.y-lo.y)};CHECK(std::memcmp(&q,&expected,sizeof(q))==0);}}
 std::cout<<"N (115,-40)->(115,10)->(155,-40)->(155,10), original arithmetic bit-exact PASS\n";
}
void retired(){
 reset();heights();servoWrites.clear();for(const char* name:{"TRIANGLE","Z","V"}){Serial.output.clear();send(std::string("DRAW SHAPE ")+name+" 115 -40 155 10");CHECK(Serial.output=="ERR E05\r\n"&&!isMotionBusy()&&servoWrites.empty());}
 for(ShapeId shape:{ShapeId::Triangle,ShapeId::Z,ShapeId::V}){TeachPoint p;CHECK(shapeVertexCount(shape)==0&&!shapeVertex(shape,0,{115,-40},{155,10},p));}
}
void jointLimited(){
 unsigned valid=0,limited=0;float maxFraction=0;
 for(int shoulder=25;shoulder<=120;shoulder+=5)for(int elbow=40;elbow<=150;elbow+=5)for(float dz:{.5f,1.f,2.f,4.f}){
  Pose pose={{90,float(shoulder),float(elbow),85}},quantized;if(!prepareOutputPose(pose,quantized))continue;Point3D a=fk(pose),b=a;b.z-=dz;
  IKResult3D ia=solveIK3D(a),ib=solveIK3D(b);if(!ia.valid||!ib.valid)continue;
  Pose pa={{ia.j0,ia.j1,ia.j2,85}},pb={{ib.j0,ib.j1,ib.j2,85}};
  if(!prepareOutputPose(pa,quantized)||!prepareOutputPose(pb,quantized))continue;
  uint32_t joint=ceilf(maxDifference(pa,pb)*1000/60.f),space=ceilf(dz*1000/37.5f);++valid;maxFraction=max(maxFraction,float(joint)/space);if(joint<=space||joint<=10)continue;++limited;
  reset();auto& t=const_cast<TeachSettings&>(teachSettings());t={b.z,a.z,(b.z+a.z)/2,true,true,true};setCommand(hardwareConfig.home);if(!startDrawLine({a.x,a.y},{a.x-.5f,a.y}))continue;
  for(unsigned n=0;task.status.state==MotionState::Validating&&n<10000;++n)updateMotion(simulatedTime,false);
  if(task.status.state!=MotionState::Running){cancelMotion();resumeArm();continue;}
  DownTiming d=runDown("joint-limited");CHECK(d.newDuration==max(uint32_t(10),d.jointDuration)&&d.newDuration>d.newSpace);CHECK(float(d.oldDuration)/d.newDuration<1.5f);return;
 }
 std::cout<<"Current-calibration vertical grid valid="<<valid<<" jointLimitedCandidates="<<limited<<" maxJointToNewSpace="<<maxFraction<<"; cap remains in every checked duration\n";
 CHECK(valid>0);
}
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];CHECK(kPenDownSpeedScale==1.50f&&DEFAULT_MOVE_SPEED==60);originalN();retired();
 for(int kind=0;kind<4;++kind){reset();heights();setCommand(hardwareConfig.home);CHECK(kind==0?startDrawLine(fieldPoints[0],fieldPoints[1]):kind==1?startBasicShape(ShapeId::N,{106.56f,-29.16f},{159.81f,26.37f}):kind==2?startPolyline():startCurve());preflight();const char* names[]={"LINE","N","POLY","CURVE"};runDown(names[kind]);}
 reset();heights();setCommand(hardwareConfig.home);send("DRAW N");CHECK(drawingTaskActive());preflight();runDown("requested-N");
 jointLimited();std::cout<<"PASS pen down "<<assertions<<'\n';
}
