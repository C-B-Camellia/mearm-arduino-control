#define main axisRegressionMain
#include "teach_axis_tests.cpp"
#undef main
#include "app_controller.h"
#include "hardware_config.h"
void motionTick(){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);}
void prepared(){
 reset({{90,110,70,85}});CHECK(beginTeach());
 float z=fk(commandedPose()).z;
 auto& t=const_cast<TeachSettings&>(teachSettings());t={z-10,z+10,z+5,true,true,true};
 Serial.output.clear();servoWrites.clear();
}
void five(){
 prepared();CHECK(beginMarking());finish();
 for(int i=0;i<5;++i){Pose p=commandedPose();p.angles[Base]=80+i*5;command(p);CHECK(markTeachPoint());}
 CHECK(beginTeach());Serial.output.clear();servoWrites.clear();
}
int main(){
 prepared();Point3D start=fk(commandedPose());float grip=commandedPose().angles[3];analogValues[3]=619;
 send("TEACH MARKING");CHECK(Serial.output.find("OK TEACH MARKING\r\n")!=std::string::npos);CHECK(motionStatus().state==MotionState::Waiting&&!markingActive());
 for(int i=0;i<20;++i){motionTick();updateManualControl(simulatedTime);CHECK(!hasPendingPose());}CHECK(servoWrites.empty());
 send("MARK");CHECK(lastControlError()==ControlError::Busy);send("MODE JOINT");CHECK(lastControlError()==ControlError::Busy);
 center();finish();CHECK(markingActive() && Serial.output.find("TEACH MOVING")!=std::string::npos && Serial.output.find("TEACH READY")!=std::string::npos);
 Point3D goal=fk(goalPose());CHECK(fabsf(goal.x-start.x)<.05f&&fabsf(goal.y-start.y)<.05f&&fabsf(goal.z-teachSettings().teachZ)<.05f);CHECK(commandedPose().angles[3]==grip);
 send("MARK");CHECK(teachPointCount()==1 && Serial.output.find("OK MARK\r\n")!=std::string::npos);
 for(const char* abort:{"TEACH EXIT","CANCEL","BTN 4"}){
  five();TeachPoint saved;CHECK(getTeachPoint(0,saved));analogValues[3]=619;send("TEACH MARKING");CHECK(isMotionBusy());send(abort);CHECK(!isMotionBusy());center();send("GO");
  size_t writes=servoWrites.size();for(int i=0;i<50;++i)motionTick();CHECK(servoWrites.size()==writes&&!markingActive());TeachPoint retained;CHECK(teachPointCount()==5&&getTeachPoint(0,retained)&&saved.x==retained.x&&saved.y==retained.y);
 }
 prepared();analogValues[3]=619;send("TEACH MARKING");send("HOLD");center();for(int i=0;i<30;++i)motionTick();CHECK(servoWrites.empty()&&motionStatus().state==MotionState::Paused);send("GO");finish();CHECK(markingActive());
 for(const char* abort:{"TEACH EXIT","CANCEL"}){
  prepared();send("TEACH MARKING");while(motionStatus().state!=MotionState::Running)motionTick();motionTick();send(abort);CHECK(!isMotionBusy()&&!markingActive());size_t writes=servoWrites.size();for(int i=0;i<50;++i)motionTick();CHECK(servoWrites.size()==writes);
 }
 five();auto& t=const_cast<TeachSettings&>(teachSettings());t.teachZ=1000;t.travelZ=1010;send("TEACH MARKING");
 for(int i=0;isMotionBusy()&&i<10000;++i){motionTick();}
 CHECK(!isMotionBusy()&&isArmHeld()&&!markingActive()&&teachPointCount()==5&&servoWrites.empty());CHECK(lastControlError()==ControlError::Unreachable);
 prepared();auto& same=const_cast<TeachSettings&>(teachSettings());same.teachZ=same.drawZ;send("TEACH MARKING");CHECK(Serial.output.find("ERR T01\r\n")!=std::string::npos&&!isMotionBusy());
 prepared();send("MARK");CHECK(Serial.output.find("ERR T03\r\n")!=std::string::npos);send("TEACH EXIT");Serial.output.clear();send("TEACH MARKING");CHECK(Serial.output=="ERR T02\r\n");
 prepared();Serial.output.clear();send("S");CHECK(Serial.output=="OK S\r\n");finish();CHECK(commandedPose().angles[3]==85);send("O");finish();CHECK(commandedPose().angles[3]==hardwareConfig.gripOpen);
 send("HELP");CHECK(lastControlError()==ControlError::Invalid);send("FEED 70");CHECK(lastControlError()==ControlError::Invalid);
 std::cout<<"PASS interaction "<<assertions<<'\n';
}
