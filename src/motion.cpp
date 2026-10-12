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
    ShapeId shape;
    bool recoveryLift;
    bool segmentLoaded,endpointQueued;
};
MotionTask task{};
bool isDrawingKind(TaskKind kind) {
    return kind==TaskKind::DrawLine || kind==TaskKind::DrawShape || kind==TaskKind::Polyline || kind==TaskKind::Curve;
}
uint8_t drawCount() {
    if(task.status.kind==TaskKind::DrawLine) return 2;
    if(task.status.kind==TaskKind::DrawShape) return shapeVertexCount(task.shape);
    return TEACH_POINT_COUNT;
}
bool drawVertex(uint8_t index,TeachPoint& p) {
    if(task.status.kind==TaskKind::DrawLine) {
        if(index>1) return false;
        p=index==0?TeachPoint{task.cartStart.x,task.cartStart.y}:TeachPoint{task.cartEnd.x,task.cartEnd.y}; return true;
    }
    if(task.status.kind==TaskKind::DrawShape)
        return shapeVertex(task.shape,index,{task.cartStart.x,task.cartStart.y},{task.cartEnd.x,task.cartEnd.y},p);
    return getTeachPoint(index,p);
}
bool curveKnots(uint8_t segment,TeachPoint& a,TeachPoint& b,TeachPoint& c,TeachPoint& d) {
    if(!drawVertex(segment,b) || !drawVertex(segment+1,c)) return false;
    if(segment==0) a={2*b.x-c.x,2*b.y-c.y}; else if(!drawVertex(segment-1,a)) return false;
    if(segment+2==drawCount()) d={2*c.x-b.x,2*c.y-b.y}; else if(!drawVertex(segment+2,d)) return false;
    return true;
}
uint16_t strokeSteps(uint8_t segment) {
    TeachPoint a,b,c,d;
    float length;
    if(task.status.kind==TaskKind::Curve) {
        if(!curveKnots(segment,a,b,c,d)) return 0;
        length=2*(pointDistance(a,b)+pointDistance(b,c)+pointDistance(c,d));
    } else {
        if(!drawVertex(segment,a) || !drawVertex(segment+1,b)) return 0;
        length=pointDistance(a,b);
    }
    float steps=ceilf(length/0.75f);
    if(!isfinite(steps) || length<0.1f || steps>4096) return 0;
    return max(1,static_cast<int>(steps));
}
bool pointPose(const Point3D& point,Pose& pose) {
    IKResult3D ik=solveIK3D(point);
    if(!ik.valid) return rejectControl(ControlError::Unreachable);
    pose.angles[Base]=ik.j0; pose.angles[Shoulder]=ik.j1; pose.angles[Elbow]=ik.j2; return true;
}
bool drawGoal(uint16_t index,Pose& pose) {
    uint8_t homeCount=hardwareConfig.homeViaCount+1;
    if(index<=homeCount) { pose=index<homeCount?hardwareConfig.homeVia[index-1]:hardwareConfig.home; return true; }
    index-=homeCount;
    TeachPoint point;
    if(index==1) {
        Point3D home=solveFK3D(hardwareConfig.home.angles[Base],hardwareConfig.home.angles[Shoulder],hardwareConfig.home.angles[Elbow]);
        home.z=teachSettings().travelZ; return pointPose(home,pose);
    }
    if(index<=3) {
        if(!drawVertex(0,point)) return rejectControl(ControlError::Invalid);
        return pointPose({point.x,point.y,index==3?teachSettings().drawZ:teachSettings().travelZ},pose);
    }
    index-=3;
    for(uint8_t segment=0;segment<drawCount()-1;++segment) {
        uint16_t steps=strokeSteps(segment);
        if(!steps) return rejectControl(ControlError::Invalid);
        if(index<=steps) {
            float t=static_cast<float>(index)/steps;
            TeachPoint a,b,c,d;
            if(task.status.kind==TaskKind::Curve) {
                if(!curveKnots(segment,a,b,c,d) || !centripetalSample(a,b,c,d,t,point)) return rejectControl(ControlError::Invalid);
            } else {
                if(!drawVertex(segment,a) || !drawVertex(segment+1,b)) return rejectControl(ControlError::Invalid);
                point={a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};
            }
            return pointPose({point.x,point.y,teachSettings().drawZ},pose);
        }
        index-=steps;
    }
    if(index==1) {
        if(!drawVertex(drawCount()-1,point)) return rejectControl(ControlError::Invalid);
        return pointPose({point.x,point.y,teachSettings().travelZ},pose);
    }
    --index;
    if(index>homeCount) return rejectControl(ControlError::Invalid);
    pose=index<homeCount?hardwareConfig.homeVia[index-1]:hardwareConfig.home; return true;
}
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
        if(task.recoveryLift) { if(index==1) { pose=task.target; return true; } --index; }
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
    if(isDrawingKind(task.status.kind)) return drawGoal(index,pose);
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
        if(isDrawingKind(task.status.kind)) {
            Point3D a=solveFK3D(task.segmentStart.angles[Base],task.segmentStart.angles[Shoulder],task.segmentStart.angles[Elbow]);
            Point3D b=solveFK3D(task.segmentTarget.angles[Base],task.segmentTarget.angles[Shoulder],task.segmentTarget.angles[Elbow]);
            float dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z;
            float feed=10.0f*automaticSpeed()/DEFAULT_MOVE_SPEED;
            task.duration=max(task.duration,static_cast<uint32_t>(ceilf(sqrtf(dx*dx+dy*dy+dz*dz)*1000.0f/feed)));
        }
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
    bool lift=drawingTaskActive();
    Pose lifted=commandedPose();
    if(lift) {
        Point3D point=solveFK3D(lifted.angles[Base],lifted.angles[Shoulder],lifted.angles[Elbow]);
        point.z=teachSettings().travelZ;
        if(!drawSettingsValid() || !pointPose(point,lifted)) { cancelMotion(); return rejectControl(ControlError::Unreachable); }
    }
    cancelMotion();
    if(!homeCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    resumeArm();
    if(!startHome()) { holdArm(); return false; }
    if(lift) { task.recoveryLift=true; task.target=lifted; ++task.status.totalWaypoints; }
    return true;
}
bool drawingTaskActive() { return isMotionBusy() && isDrawingKind(task.status.kind); }
namespace {
bool startDrawing(TaskKind kind,ShapeId shape,const TeachPoint& a,const TeachPoint& b) {
    if(!drawSettingsValid() || !homeCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    // The configured HOME/via route must keep the pencil clamped.
    float grip=joints[Gripper].commandedAngle;
    if(hardwareConfig.home.angles[Gripper]!=grip) return rejectControl(ControlError::Unsafe);
    for(uint8_t i=0;i<hardwareConfig.homeViaCount;++i) if(hardwareConfig.homeVia[i].angles[Gripper]!=grip) return rejectControl(ControlError::Unsafe);
    if(!begin(kind,1)) return false;
    task.shape=shape; task.cartStart={a.x,a.y,0}; task.cartEnd={b.x,b.y,0};
    uint32_t count=2*(hardwareConfig.homeViaCount+1)+4;
    for(uint8_t i=0;i<drawCount()-1;++i) {
        uint16_t n=strokeSteps(i);
        if(!n) { task.status.state=MotionState::Idle; return rejectControl(ControlError::Invalid); }
        count+=n;
    }
    if(count>32767) { task.status.state=MotionState::Idle; return rejectControl(ControlError::Full); }
    task.status.totalWaypoints=count; return true;
}
}
bool startDrawLine(const TeachPoint& a,const TeachPoint& b) { return startDrawing(TaskKind::DrawLine,ShapeId::N,a,b); }
bool startBasicShape(ShapeId shape,const TeachPoint& lo,const TeachPoint& hi) { return startDrawing(TaskKind::DrawShape,shape,lo,hi); }
bool startPolyline() {
    if(teachPointCount()!=TEACH_POINT_COUNT) return rejectControl(ControlError::NoData);
    return startDrawing(TaskKind::Polyline,ShapeId::N,{},{});
}
bool startCurve() {
    if(teachPointCount()!=TEACH_POINT_COUNT) return rejectControl(ControlError::NoData);
    return startDrawing(TaskKind::Curve,ShapeId::N,{},{});
}
