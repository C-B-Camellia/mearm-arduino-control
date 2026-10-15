#include "Arduino.h"
#include "EEPROM.h"
#include "arm.h"
#include "commands.h"
#include "config.h"
#include "joystick.h"
#include "kinematics.h"
#include "manual_control.h"
#include "motion.h"
#include "control_settings.h"
#include "control_error.h"
#include "teaching.h"
#include <iostream>
#include <cstdlib>
uint32_t simulatedTime=0;
std::vector<ServoWrite> servoWrites;
std::vector<int> attachedPins;
std::vector<uint32_t> delayCalls;
int analogValues[4]={506,519,515,519};
SerialStub Serial; EEPROMStub EEPROM;
void setup();
int assertions=0;
#define CHECK(e) do {++assertions;if(!(e)){std::cerr<<"FAIL "<<__LINE__<<": "<<#e<<'\n';std::exit(1);}}while(0)
Point3D fk(const Pose& p){return solveFK3D(p.angles[0],p.angles[1],p.angles[2]);}
void center(){analogValues[0]=506;analogValues[1]=519;analogValues[2]=515;analogValues[3]=519;}
void tick(){simulatedTime+=20;updateManualControl(simulatedTime);CHECK(applyPendingPose());}
void command(const Pose& p){CHECK(requestPose(p));CHECK(applyPendingPose());syncManualTarget(simulatedTime);}
void reset(const Pose& p){cancelMotion();resumeArm();center();resetControlSettings();setup();command(p);CHECK(setControlMode(ControlMode::Cartesian));tick();}
void send(const char* s){Serial.input+=std::string(s)+'\n';while(!Serial.input.empty())pollSerial();}
void finish(){unsigned n=0;while(isMotionBusy()&&++n<10000){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);}CHECK(!isMotionBusy()&&!isArmHeld());}
struct Result{Pose goal,accepted;Point3D desired,output;};
Result move(const Pose& p,bool teach,int pin,int delta){
 reset(p);if(teach){CHECK(beginTeach());tick();}
 Point3D initial=fk(commandedPose());float grip=commandedPose().angles[3];
 analogValues[pin-A0]+=delta;
 JoystickInput in=readJoysticks();
 std::cout<<"INPUT pin="<<pin<<" delta="<<delta<<" axes="<<int(in.lx)<<','<<int(in.ly)<<','<<int(in.rx)<<','<<int(in.ry)<<'\n';
 for(int i=0;i<20;++i)tick();
 Result r{goalPose(),commandedPose(),fk(goalPose()),fk(commandedPose())};
 if(pin==A3){CHECK(std::fabs(r.desired.x-initial.x)<0.01f);CHECK(std::fabs(r.desired.y-initial.y)<0.01f);CHECK((r.desired.z-initial.z)*delta>0);CHECK(std::fabs(r.desired.z-initial.z)>9.9f);}
 if(teach)CHECK(commandedPose().angles[3]==grip);
 std::cout<<"MOVE teach="<<teach<<" initial="<<initial.x<<','<<initial.y<<','<<initial.z<<" goal="<<r.desired.x<<','<<r.desired.y<<','<<r.desired.z<<" integerFK="<<r.output.x<<','<<r.output.y<<','<<r.output.z<<'\n';
 center();return r;
}
int main(){setup();
 // Logical LX/LY/RY/RX are A0/A1/A3/A2. Struct field order differs.
 for(int pin=A0;pin<=A3;++pin){center();analogValues[pin-A0]+=100;auto q=readJoysticks();CHECK(q.lx==(pin==A0?-1:0));CHECK(q.ly==(pin==A1?1:0));CHECK(q.rx==(pin==A2?-1:0));CHECK(q.ry==(pin==A3?-1:0));center();analogValues[pin-A0]+=20;CHECK(joysticksCentered(readJoysticks()));}
 const Pose poses[]={{{90,110,70,85}},{{74,64,95,85}}};
 for(auto p:poses)for(int sign:{-1,1}){
  auto a=move(p,false,A3,sign*100),b=move(p,true,A3,sign*100);
  for(int j=0;j<4;++j){CHECK(a.goal.angles[j]==b.goal.angles[j]);CHECK(a.accepted.angles[j]==b.accepted.angles[j]);}
  CHECK(captureTeachHeight(true));CHECK(std::fabs(teachSettings().drawZ-b.output.z)<0.001f);
 }
 // Gripper input remains a gripper axis in normal CART, and is ignored in teaching.
 Pose p=poses[0];auto a=move(p,false,A2,-100),b=move(p,true,A2,-100);CHECK(a.accepted.angles[3]>85);CHECK(b.accepted.angles[3]==85);
 send("J3 84");finish();CHECK(commandedPose().angles[3]==84);tick();analogValues[2]=415;for(int i=0;i<10;++i)tick();CHECK(commandedPose().angles[3]==84);center();CHECK(exitTeach());tick();analogValues[2]=415;for(int i=0;i<10;++i)tick();CHECK(commandedPose().angles[3]>84);center();
 // After mode/ownership change, center all axes once before movement is armed.
 reset(p);CHECK(beginTeach());analogValues[3]=619;for(int i=0;i<10;++i)tick();CHECK(commandedPose().angles[1]==p.angles[1]);center();tick();analogValues[3]=619;for(int i=0;i<10;++i)tick();CHECK(fk(goalPose()).z>fk(p).z+4.9f);center();
 // Capture three actual accepted Zs with joystick movement, then enter MARKING.
 reset(p);CHECK(beginTeach());tick();CHECK(captureTeachHeight(true));float z0=teachSettings().drawZ;
 analogValues[3]=619;for(int i=0;i<10;++i)tick();center();CHECK(captureTeachAlignment());float z1=teachSettings().teachZ;Pose teachPose=commandedPose();
 analogValues[3]=619;for(int i=0;i<10;++i)tick();center();CHECK(captureTeachHeight(false));float z2=teachSettings().travelZ;
 CHECK(z0<z1&&z1<z2);command(teachPose);CHECK(teachSettings().teachZ==z1);CHECK(beginMarking());finish();CHECK(markingActive());
 std::cout<<"HEIGHTS "<<z0<<' '<<z1<<' '<<z2<<" MARKING accepted\n";
 reset(p);CHECK(beginTeach());CHECK(captureTeachHeight(true));CHECK(captureTeachAlignment());CHECK(captureTeachHeight(false));CHECK(!beginMarking());CHECK(lastControlError()==ControlError::TeachHeights);
 // Pending/motion ownership must reject height capture rather than reuse a new goal.
 reset(p);CHECK(beginTeach());CHECK(captureTeachHeight(true));float saved=teachSettings().drawZ;Pose next=p;next.angles[1]+=1;CHECK(requestPose(next));CHECK(!captureTeachHeight(true));CHECK(teachSettings().drawZ==saved);CHECK(applyPendingPose());
 CHECK(startJointMove(p));CHECK(!captureTeachHeight(true));finish();
 send("STATUS");CHECK(Serial.output.find("Z=")!=std::string::npos);
 std::cout<<"PASS TEACH ADC/XYZ/capture "<<assertions<<'\n';return 0;
}
