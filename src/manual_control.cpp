#include "manual_control.h"
#include "arm.h"
#include "motion.h"
#include "config.h"
#include "joystick.h"
#include "kinematics.h"
#include "hardware_config.h"
#include "control_settings.h"
#include "control_error.h"
#include "recording.h"
#include "teaching.h"
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
    if(teachingActive()) return rejectControl(ControlError::Busy);
    if(isMotionBusy() || isRecording() || hasPendingPose() || !joysticksCentered(readJoysticks())) return rejectControl(ControlError::Busy);
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
// Manual IK scratch must not occupy the persistent loop frame while serial
// events execute drawing cancellation/recovery through their own IK path.
__attribute__((noinline)) void updateManualControl(uint32_t now,const JoystickInput& in) {
    // Re-arm from accepted output only after ownership returns and sticks center.
    // Repeated FK while blocked cannot affect an output or the re-arm pose.
    if(isMotionBusy() || isArmHeld() || hasPendingPose()) { lastManualTime=now; armed=false; return; }
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
        // 左摇杆前后 → X
        if (in.ly)
            point.x -= axisVelocity(in.ly, MANUAL_CART_SLOW, MANUAL_CART_FAST) * dt;

        // 左摇杆左右 → Y
        if (in.lx)
            point.y += axisVelocity(in.lx, MANUAL_CART_SLOW, MANUAL_CART_FAST) * dt;

        // 右摇杆前后 → Z
        if (markingActive())
            point.z = teachSettings().teachZ;
        else if (in.ry)
            point.z -= axisVelocity(in.ry, MANUAL_CART_SLOW, MANUAL_CART_FAST) * dt;
        IKResult3D ik=solveIK3D(point); if(!ik.valid) return;
        candidate.angles[Base]=ik.j0; candidate.angles[Shoulder]=ik.j1; candidate.angles[Elbow]=ik.j2;
        if(!teachingActive() && in.rx && joints[Gripper].outputEnabled)
            candidate.angles[Gripper]=constrain(candidate.angles[Gripper]+axisVelocity(in.rx,MANUAL_JOINT_SLOW,MANUAL_JOINT_FAST)*dt,joints[Gripper].minAngle,joints[Gripper].maxAngle);
        Pose output; if(!prepareOutputPose(candidate,output)) return;
        cartTarget=point;
    }
    if(requestPose(candidate)) manualTarget=candidate;
}
