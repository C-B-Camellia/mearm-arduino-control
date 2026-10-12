#include "manual_control.h"
#include "arm.h"
#include "motion.h"
#include "config.h"
#include "joystick.h"
#include "kinematics.h"
#include "hardware_config.h"
#include "control_settings.h"
#include "control_error.h"
#include <math.h>

namespace {

    ControlMode currentMode = ControlMode::Joint;
    Pose manualTarget{};
    Point3D cartTarget{};
    uint32_t lastManualTime=0;
    bool armed=false;
    float axisVelocity(int8_t axis,float slow,float fast) {
        return (axis<0?-1.0f:1.0f)*(abs(axis)==2?fast:slow)*manualSpeedScale();
    }
}

ControlMode getControlMode() {
    return currentMode;
}

bool setControlMode(ControlMode newMode) {
    if(newMode!=ControlMode::Joint && newMode!=ControlMode::Cartesian) return rejectControl(ControlError::Invalid);
    if(newMode==currentMode) return true;
    if(isMotionBusy() || hasPendingPose() || !joysticksCentered(readJoysticks())) return rejectControl(ControlError::Busy);
    if(isArmHeld()) return rejectControl(ControlError::Held);
    if(newMode==ControlMode::Cartesian && !cartesianCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    currentMode=newMode; syncManualTarget(millis()); clearControlError(); return true;
}

void updateManualControl(uint32_t now) {
    updateManualControl(now,readJoysticks());
}

void syncManualTarget(uint32_t now) {
    manualTarget=commandedPose();
    cartTarget=solveFK3D(manualTarget.angles[Base],manualTarget.angles[Shoulder],manualTarget.angles[Elbow]);
    lastManualTime=now; armed=false;
}
void initializeManualControl(uint32_t now) { currentMode=ControlMode::Joint; syncManualTarget(now); }
void updateManualControl(uint32_t now,const JoystickInput& in) {
    if(isMotionBusy() || isArmHeld() || hasPendingPose()) { syncManualTarget(now); return; }
    uint32_t elapsed=now-lastManualTime; lastManualTime=now;
    if(!armed) { if(joysticksCentered(in)) { syncManualTarget(now); armed=true; } return; }
    float dt=min(elapsed,MANUAL_MAX_DT_MS)*0.001f;
    if(joysticksCentered(in) || dt==0) return;
    Pose candidate=manualTarget;
    if(currentMode==ControlMode::Joint) {
        const int8_t axes[]={in.lx,in.ly,in.ry,in.rx};
        for(uint8_t i=0;i<JointCount;++i) {
            if(axes[i] && joints[i].outputEnabled)
                candidate.angles[i]=constrain(candidate.angles[i]+axisVelocity(axes[i],MANUAL_JOINT_SLOW,MANUAL_JOINT_FAST)*dt,joints[i].minAngle,joints[i].maxAngle);
        }
    } else {
        if(!cartesianCalibrationValid()) return;
        Point3D point=cartTarget;
        if(in.lx) point.x+=axisVelocity(in.lx,MANUAL_CART_SLOW,MANUAL_CART_FAST)*dt;
        if(in.ly) point.y+=axisVelocity(in.ly,MANUAL_CART_SLOW,MANUAL_CART_FAST)*dt;
        if(in.ry) point.z+=axisVelocity(in.ry,MANUAL_CART_SLOW,MANUAL_CART_FAST)*dt;
        IKResult3D ik=solveIK3D(point); if(!ik.valid) return;
        candidate.angles[Base]=ik.j0; candidate.angles[Shoulder]=ik.j1; candidate.angles[Elbow]=ik.j2;
        if(in.rx && joints[Gripper].outputEnabled)
            candidate.angles[Gripper]=constrain(candidate.angles[Gripper]+axisVelocity(in.rx,MANUAL_JOINT_SLOW,MANUAL_JOINT_FAST)*dt,joints[Gripper].minAngle,joints[Gripper].maxAngle);
        Pose output; if(!prepareOutputPose(candidate,output)) return;
        cartTarget=point;
    }
    if(requestPose(candidate)) manualTarget=candidate;
}
