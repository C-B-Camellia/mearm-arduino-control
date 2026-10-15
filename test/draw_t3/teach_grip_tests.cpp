#define main previousMain
#include "speed_tests.cpp"
#undef main
void rightStick() { simulatedTime+=20;updateManualControl(simulatedTime,{});for(int i=0;i<20;++i){simulatedTime+=20;updateManualControl(simulatedTime,{0,0,2,0});CHECK(applyPendingPose());} }
int main(){measuredConfig=hardwareConfig;for(int i=0;i<4;++i)measuredJoints[i]=joints[i];reset();
 CHECK(teachingActive() && !markingActive());float grip=commandedPose().angles[3];rightStick();CHECK(commandedPose().angles[3]==grip);
 setCommand(lowerPose(116));CHECK(captureTeachHeight(true));setCommand(hardwareConfig.home);CHECK(captureTeachAlignment());CHECK(beginMarking()); while(isMotionBusy()){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);} CHECK(markingActive());rightStick();CHECK(commandedPose().angles[3]==grip);
 CHECK(beginTeach());send("J3 84");while(isMotionBusy()){simulatedTime+=20;updateMotion(simulatedTime,true);CHECK(applyPendingPose());updateMotion(simulatedTime,false);}CHECK(commandedPose().angles[3]==84);rightStick();CHECK(commandedPose().angles[3]==84);
 CHECK(exitTeach());rightStick();CHECK(commandedPose().angles[3]>84);
 std::cout<<"PASS teach grip "<<assertions<<'\n';}
