#include "motion.h"
#include "arm.h"
#include "config.h"
#include "control_settings.h"
#include "control_error.h"
#include "manual_control.h"
#include "hardware_config.h"
#include "recording.h"
#include "pick_place.h"
#include <math.h>

namespace {
struct MotionTask {
    MotionStatus status;
    MotionState resumeState;
    Pose origin, target, segmentStart, segmentTarget;
    Point3D cartStart,cartEnd;
    uint32_t elapsed,lastUpdate,duration;
    uint16_t validationSubstep,validationSubsteps;
    uint16_t dwellRemaining;
    uint8_t objectIndex;
    bool segmentLoaded,endpointQueued;
};
MotionTask task{};
float maxDifference(const Pose& a,const Pose& b) {
    float d=0; for(uint8_t i=0;i<JointCount;++i) d=max(d,fabsf(a.angles[i]-b.angles[i])); return d;
}
Pose interpolate(const Pose& a,const Pose& b,float t) {
    Pose p; for(uint8_t i=0;i<JointCount;++i) p.angles[i]=a.angles[i]+(b.angles[i]-a.angles[i])*t; return p;
}
bool sampleGoal(uint16_t index,Pose& pose) {
    pose=task.origin;
    float t=static_cast<float>(index)/task.status.totalWaypoints;
    if(task.status.kind==TaskKind::JointMove) { pose=task.target; return true; }
    if(task.status.kind==TaskKind::Home) {
        pose=index<=hardwareConfig.homeViaCount?hardwareConfig.homeVia[index-1]:hardwareConfig.home; return true;
    }
    if(task.status.kind==TaskKind::PickPlace) {
        uint16_t dwell; return getPickStage(task.objectIndex,index-1,pose,dwell) || rejectControl(ControlError::Uncalibrated);
    }
    if(task.status.kind==TaskKind::Playback) {
        RecordedFrame frame; if(!getRecordedFrame(index-1,frame)) return rejectControl(ControlError::NoData);
        for(uint8_t i=0;i<JointCount;++i) pose.angles[i]=frame.angles[i];
        return true;
    }
    Point3D p={task.cartStart.x+(task.cartEnd.x-task.cartStart.x)*t,
               task.cartStart.y+(task.cartEnd.y-task.cartStart.y)*t,
               task.cartStart.z+(task.cartEnd.z-task.cartStart.z)*t};
    if(task.status.kind==TaskKind::LinePreview || task.status.kind==TaskKind::LineExecution) {
        IKResult ik=solveIK(p.x,p.z); if(!ik.valid) return rejectControl(ControlError::Unreachable);
        pose.angles[Shoulder]=ik.j1; pose.angles[Elbow]=ik.j2;
    } else {
        IKResult3D ik=solveIK3D(p); if(!ik.valid) return rejectControl(ControlError::Unreachable);
        pose.angles[Base]=ik.j0; pose.angles[Shoulder]=ik.j1; pose.angles[Elbow]=ik.j2;
    }
    return true;
}
void finish(bool failed=false) {
    TaskKind kind=task.status.kind;
    task.status.state=MotionState::Idle;
    task.segmentLoaded=false; task.endpointQueued=false;
    if(failed) { holdArm(); printControlError(); }
    else {
        if(kind==TaskKind::LinePreview) Serial.println(F("LINE VALID"));
        else if(kind==TaskKind::LineExecution) Serial.println(F("RUNLINE DONE"));
        else Serial.println(F("TASK DONE (commanded)"));
    }
    syncManualTarget(millis());
}
bool begin(TaskKind kind,uint16_t count) {
    if(isMotionBusy() || hasPendingPose() || isRecording()) return rejectControl(ControlError::Busy);
    if(isArmHeld()) return rejectControl(ControlError::Held);
    task=MotionTask{}; task.status.kind=kind; task.status.state=MotionState::Validating;
    task.status.totalWaypoints=count; task.origin=commandedPose(); task.segmentStart=task.origin;
    clearControlError(); return true;
}
void validateOne() {
    if(!task.segmentLoaded) {
        if(!sampleGoal(task.status.validatedWaypoints+1,task.segmentTarget)) { finish(true); return; }
        task.validationSubsteps=max(1,static_cast<int>(ceilf(maxDifference(task.segmentStart,task.segmentTarget))));
        task.validationSubstep=0; task.segmentLoaded=true;
    }
    ++task.validationSubstep;
    Pose p=interpolate(task.segmentStart,task.segmentTarget,static_cast<float>(task.validationSubstep)/task.validationSubsteps), out;
    if(!prepareOutputPose(p,out)) {
        rejectControl(ControlError::Unsafe);
        if(task.status.kind==TaskKind::LineExecution) Serial.println(F("RUNLINE ABORTED before motion"));
        finish(true); return;
    }
    if(task.validationSubstep==task.validationSubsteps) {
        task.segmentStart=task.segmentTarget; task.segmentLoaded=false;
        ++task.status.validatedWaypoints;
        if(task.status.validatedWaypoints==task.status.totalWaypoints) {
            if(task.status.kind==TaskKind::LinePreview) finish();
            else { task.status.state=MotionState::Running; task.lastUpdate=millis(); Serial.println(F("PATH VALID")); }
        }
    }
}
void execute(uint32_t now) {
    if(task.endpointQueued) return;
    if(!task.segmentLoaded) {
        task.segmentStart=commandedPose();
        if(!sampleGoal(task.status.completedWaypoints+1,task.segmentTarget)) { finish(true); return; }
        task.elapsed=0; task.segmentLoaded=true;
        task.duration=max(static_cast<uint32_t>(WAYPOINT_DELAY_MS),static_cast<uint32_t>(ceilf(maxDifference(task.segmentStart,task.segmentTarget)*1000.0f/automaticSpeed())));
        if(task.status.kind==TaskKind::Playback && task.status.completedWaypoints>0) {
            RecordedFrame a,b;
            getRecordedFrame(task.status.completedWaypoints-1,a); getRecordedFrame(task.status.completedWaypoints,b);
            task.duration=max(task.duration,static_cast<uint32_t>(b.timeMs-a.timeMs));
        }
    }
    uint32_t dt=min(static_cast<uint32_t>(now-task.lastUpdate),MANUAL_MAX_DT_MS); task.lastUpdate=now;
    task.elapsed=min(task.duration,task.elapsed+dt);
    float t=static_cast<float>(task.elapsed)/task.duration;
    Pose p=interpolate(task.segmentStart,task.segmentTarget,t);
    if(!requestPose(p)) { rejectControl(ControlError::Unsafe); finish(true); return; }
    task.endpointQueued=task.elapsed==task.duration;
}
}
bool isMotionBusy() { return task.status.state!=MotionState::Idle; }
MotionStatus motionStatus() { return task.status; }
bool startJointMove(const Pose& pose) {
    Pose out;
    if(!prepareOutputPose(pose,out)) return rejectControl(ControlError::Unsafe);
    if(!begin(TaskKind::JointMove,1)) return false;
    task.target=pose; return true;
}
bool startLine(Point2D target,TaskKind kind) {
    if(kind!=TaskKind::LinePreview && kind!=TaskKind::LineExecution) return rejectControl(ControlError::Invalid);
    Point2D start=solveFK(joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle).position;
    float dx=target.x-start.x,dz=target.z-start.z;
    float n=ceilf(sqrtf(dx*dx+dz*dz)/LINE_STEP_MM);
    if(!isfinite(n) || n>32767 || !isfinite(target.x) || !isfinite(target.z)) return rejectControl(ControlError::Invalid);
    if(!begin(kind,max(1,static_cast<int>(n)))) return false;
    task.cartStart={start.x,0,start.z}; task.cartEnd={target.x,0,target.z}; return true;
}
bool startCartesianMove(const Point3D& target) {
    if(!cartesianCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    Point3D start=solveFK3D(joints[Base].commandedAngle,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    float dx=target.x-start.x,dy=target.y-start.y,dz=target.z-start.z;
    float n=ceilf(sqrtf(dx*dx+dy*dy+dz*dz)/2.0f);
    if(!isfinite(n) || n>32767) return rejectControl(ControlError::Invalid);
    if(!begin(TaskKind::CartesianMove,max(1,static_cast<int>(n)))) return false;
    task.cartStart=start; task.cartEnd=target; return true;
}
void updateMotion(uint32_t now,bool controlDue) {
    if(task.status.state==MotionState::Validating) { validateOne(); return; }
    if(task.status.state!=MotionState::Running) return;
    if(task.dwellRemaining) {
        if(controlDue) {
            uint32_t dt=min(static_cast<uint32_t>(now-task.lastUpdate),MANUAL_MAX_DT_MS); task.lastUpdate=now;
            task.dwellRemaining=dt>=task.dwellRemaining?0:task.dwellRemaining-dt;
        }
        if(task.dwellRemaining) return;
        if(task.status.completedWaypoints==task.status.totalWaypoints) { finish(); return; }
    }
    if(task.endpointQueued && !hasPendingPose()) {
        ++task.status.completedWaypoints; task.endpointQueued=false; task.segmentLoaded=false;
        if(task.status.kind==TaskKind::PickPlace) {
            Pose unused; getPickStage(task.objectIndex,task.status.completedWaypoints-1,unused,task.dwellRemaining);
        }
        if(task.status.completedWaypoints==task.status.totalWaypoints && !task.dwellRemaining) { finish(); return; }
        if(task.dwellRemaining) { task.lastUpdate=now; return; }
    }
    if(controlDue) execute(now);
}
bool pauseMotion(uint32_t now) {
    if(!isMotionBusy()) return false;
    if(task.status.state==MotionState::Paused) return true;
    task.resumeState=task.status.state; task.status.state=MotionState::Paused;
    task.lastUpdate=now; holdArm(); return true;
}
bool resumeMotion(uint32_t now) {
    if(!isMotionBusy()) return false;
    if(task.status.state==MotionState::Paused) { task.status.state=task.resumeState; task.lastUpdate=now; resumeArm(); }
    return true;
}
void cancelMotion() { task=MotionTask{}; holdArm(); syncManualTarget(millis()); }
bool startHome() {
    if(!homeCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    return begin(TaskKind::Home,hardwareConfig.homeViaCount+1);
}
bool startPickPlace(uint8_t object) {
    if(!pickCalibrationValid(object)) return rejectControl(ControlError::Uncalibrated);
    if(!begin(TaskKind::PickPlace,PICK_STAGE_COUNT)) return false;
    task.objectIndex=object; return true;
}
bool startPlayback() {
    if(!hasRecording()) return rejectControl(ControlError::NoData);
    return begin(TaskKind::Playback,recordingStatus().count);
}
bool cancelAndReturnHome() {
    cancelMotion();
    if(!homeCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    resumeArm();
    if(!startHome()) { holdArm(); return false; }
    return true;
}
