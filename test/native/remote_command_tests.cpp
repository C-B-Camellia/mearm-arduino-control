#define main oldAxisMain
#include "teach_axis_tests.cpp"
#undef main
#include "app_controller.h"
#include "hardware_config.h"
#include "pick_place.h"
#include "recording.h"
#include "button_link.h"
#include "button_protocol.h"
#include <fstream>
#include <sstream>
void initializePinUart() {}
int readPinUart() { return -1; }
bool pinUartOverflow() { return false; }
HardwareConfig hw;
JointControl jc[4];
uint8_t seq=0;
void wire(uint8_t type,uint8_t value,uint8_t sequence) {
 uint8_t b[8];ButtonWire::encode(b,type,77,sequence,value);for(auto c:b)feedButtonByte(c,simulatedTime);
}
void remote(AppAction c){wire(ButtonWire::Command,uint8_t(c),++seq);}
void freshRemote(){
 hardwareConfig=hw;for(int i=0;i<4;++i)joints[i]=jc[i];EEPROM.reset();
 reset(hw.home);Serial.output.clear();servoWrites.clear();clearControlError();seq=0;wire(ButtonWire::Hello,0,0);
}
void paper(){auto& t=const_cast<TeachSettings&>(teachSettings());t={82.53f,94.29f,86.71f,true,true,true};}
void actualFive(int count=5){
 CHECK(beginTeach());command({{90,116,60,85}});CHECK(captureTeachHeight(true));
 command({{90,118,60,85}});CHECK(captureTeachAlignment());command(hw.home);CHECK(captureTeachHeight(false));
 CHECK(beginMarking());finish();
 for(int i=0;i<count;++i){Pose p=commandedPose();p.angles[0]=80+5*i;command(p);CHECK(markTeachPoint());}
}
void arrange(uint8_t id){
 freshRemote();
 if(id==1){Pose p[8];for(auto& v:p)v=hw.home;for(int i=0;i<3;++i)setSyntheticPickProgram(i,p);}
 if(id>=6&&id<=10){CHECK(beginTeach());paper();}
 if(id==11)actualFive(4);
 if(id==12||id==13||id==16||id==17||id==20)actualFive();
 if(id==14||id==15)paper();
 if(id==18||id==19){Pose p=hw.home;p.angles[0]+=5;CHECK(startJointMove(p));if(id==19)CHECK(pauseMotion(simulatedTime));}
 if(id==20)CHECK(startPolyline());
 Serial.output.clear();servoWrites.clear();clearControlError();
}
std::string snapshot(){
 std::ostringstream o;auto s=motionStatus();auto t=teachSettings();
 o<<int(s.kind)<<','<<int(s.state)<<','<<s.totalWaypoints<<','<<isArmHeld()<<','<<int(lastControlError())<<','<<int(currentWorkflow())<<','<<int(teachPointCount())<<','<<int(recordingStatus().state)<<','<<t.drawZ<<','<<t.teachZ<<','<<t.travelZ;
 for(int i=0;i<4;++i)o<<','<<commandedPose().angles[i];
 return o.str();
}
int main(int argc,char** argv){
 hw=hardwareConfig;for(int i=0;i<4;++i)jc[i]=joints[i];
 const char* text[]={"PICK NEXT","REC","PLAY","HOME","TEACH START","TEACH DRAWZ","TEACH TEACHZ","TEACH TRAVELZ","TEACH MARKING","TEACH EXIT","MARK","UNDO","CLEAR","DRAW LINE","DRAW N","DRAW POLY","DRAW CURVE","HOLD","GO","CANCEL"};
 for(uint8_t id=1;id<=20;++id){
  arrange(id);send(text[id-1]);std::string a=snapshot();
  arrange(id);remote(static_cast<AppAction>(id));CHECK(snapshot()==a);
 }
 // Fixed presets: both transports enter the original complete preflight/executor.
 for(uint8_t id:{14,15})for(bool usb:{false,true}){
  freshRemote();paper();if(usb)send(text[id-1]);else remote(static_cast<AppAction>(id));
  CHECK(motionStatus().state==MotionState::Validating);finish();CHECK(!isArmHeld());
 }
 freshRemote();paper();
 for(const char* old:{"DRAW LINE 125 30 125 100","DRAW SHAPE N 115 -40 155 10","DRAW SHAPE N","BTN 1"}){
  Serial.output.clear();send(old);CHECK(lastControlError()==ControlError::Invalid&&!isMotionBusy()&&servoWrites.empty());
 }
 // Successful admission advances PICK NEXT; busy and duplicate delivery do not.
 arrange(1);remote(AppAction::PickNext);CHECK(isMotionBusy());send("STATUS");CHECK(Serial.output.find("pickNext=B")!=std::string::npos);
 Serial.output.clear();wire(ButtonWire::Command,1,seq);CHECK(Serial.output.empty());remote(AppAction::PickNext);CHECK(lastControlError()==ControlError::Busy);
 finish();Serial.output.clear();send("STATUS");CHECK(Serial.output.find("pickNext=B")!=std::string::npos);
 remote(AppAction::PickNext);finish();Serial.output.clear();send("STATUS");CHECK(Serial.output.find("pickNext=C")!=std::string::npos);
 remote(AppAction::PickNext);finish();Serial.output.clear();send("STATUS");CHECK(Serial.output.find("pickNext=A")!=std::string::npos);
 // Old key frames never acquire new command meaning, even with valid CRC.
 freshRemote();wire(1,2,1);CHECK(!isRecording());wire(ButtonWire::Command,2,1);CHECK(!isRecording());
 wire(ButtonWire::Hello,0,1);wire(ButtonWire::Command,2,2);CHECK(isRecording());
 // Faulted, missing-session and duplicate frames cannot replay HOME/CLEAR/PICK.
 for(uint8_t id:{1,4,13}){
  arrange(id);std::string before=snapshot();size_t writes=servoWrites.size();
  initializeButtonLink();wire(ButtonWire::Command,id,1);CHECK(snapshot()==before && servoWrites.size()==writes);
  wire(ButtonWire::Hello,0,0);uint8_t bad[8];ButtonWire::encode(bad,ButtonWire::Command,77,1,id);bad[6]^=0x40;
  for(auto b:bad)feedButtonByte(b,simulatedTime);
  wire(ButtonWire::Command,id,1);CHECK(snapshot()==before && servoWrites.size()==writes);
  wire(ButtonWire::Hello,0,0);wire(ButtonWire::Command,id,1);std::string accepted=snapshot();
  Serial.output.clear();wire(ButtonWire::Command,id,1);CHECK(snapshot()==accepted && Serial.output.empty());
  if(id==13){CHECK(teachPointCount()==0);CHECK(beginMarking());finish();CHECK(markTeachPoint());wire(ButtonWire::Command,id,1);CHECK(teachPointCount()==1);}
 }
 // Partial USB input stays untouched while the second UART executes a business action.
 freshRemote();Serial.input="J3 8";pollSerial();remote(AppAction::Go);Serial.input="5\n";pollSerial();
 CHECK(Serial.output.find("OK J3 85")!=std::string::npos);finish();CHECK(commandedPose().angles[3]==85);
 // Mismatched local mode cannot capture/clear or change teaching while drawing owns motion.
 freshRemote();paper();remote(AppAction::DrawLine);CHECK(isMotionBusy());
 for(auto a:{AppAction::TeachStart,AppAction::Marking,AppAction::Clear,AppAction::Mark,AppAction::PickNext}){
  remote(a);CHECK(lastControlError()==ControlError::Busy || lastControlError()==ControlError::TeachState);
  CHECK(drawingTaskActive());
 }
 remote(AppAction::Hold);CHECK(motionStatus().state==MotionState::Paused);remote(AppAction::Go);CHECK(motionStatus().state==MotionState::Validating);
 remote(AppAction::Cancel);finish();
 // Consume exact frames emitted by the real panel gesture loop (no handwritten encoder).
 CHECK(argc==2);std::ifstream in(argv[1],std::ios::binary);CHECK(bool(in));
 freshRemote();std::vector<uint8_t> received;uint8_t bytes[8];
 while(in.read(reinterpret_cast<char*>(bytes),8)){
  CHECK(ButtonWire::valid(bytes));uint8_t id=bytes[5];
  if(bytes[1]==ButtonWire::Command){
   received.push_back(id);
   if(id==6)command({{90,116,60,85}});
   if(id==7)command({{90,118,60,85}});
   if(id==8)command(hw.home);
   if(id==11){Pose p=commandedPose();p.angles[0]=80+5*teachPointCount();command(p);}
   if(id==9)analogValues[3]=619;
   clearControlError();
  }
  for(auto b:bytes)feedButtonByte(b,simulatedTime);
  if(bytes[1]!=ButtonWire::Command)continue;
  CHECK(lastControlError()==ControlError::None);
  if(id==9){CHECK(motionStatus().state==MotionState::Waiting);for(int i=0;i<5;++i){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(!markingActive());}center();finish();CHECK(markingActive());}
  if(id==16){CHECK(teachPointCount()==5);while(motionStatus().state==MotionState::Validating)updateMotion(simulatedTime,false);CHECK(motionStatus().state==MotionState::Running);}
  if(id==18)CHECK(motionStatus().state==MotionState::Paused);
  if(id==19)CHECK(motionStatus().state==MotionState::Running);
  if(id==20){finish();CHECK(!isArmHeld());}
  if(id==13)CHECK(teachPointCount()==0);
  if(id==10)CHECK(!teachingActive());
 }
 CHECK(received==std::vector<uint8_t>({5,6,7,8,9,11,11,11,11,11,12,11,16,18,19,20,13,10}));
 std::cout<<"PASS business transport parity + real panel bytes: "<<assertions<<" checks\n";return 0;
}
