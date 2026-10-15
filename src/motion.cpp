#include "motion.h"
#include "command_text.h"
#include "arm.h"
#include "config.h"
#include "control_settings.h"
#include "control_error.h"
#include "manual_control.h"
#include "hardware_config.h"
#include "recording.h"
#include "pick_place.h"
#include "serial_format.h"
#include <math.h>

namespace {
struct MotionTask {
    MotionStatus status;
    MotionState resumeState;
    Pose origin,segmentStart,segmentTarget;
    // TaskKind is the discriminator: JointMove/Home use target; Cartesian,
    // line/drawing tasks use cart; Playback uses cart.start.x as time scale.
    // No task reads both payloads.
    union {
        Pose target;
        struct { Point3D start,end; } cart;
    };
    uint32_t elapsed,lastUpdate,duration;
    // Validation scratch and execution distance are never live together.
    union { struct { uint16_t validationSubstep,validationSubsteps; }; float segmentDistance; };
    uint16_t dwellRemaining;
    // PickPlace owns objectIndex; drawing owns the mandatory endpoint flag.
    union { uint8_t objectIndex; bool drawCheckpoint; };
    ShapeId shape;
    bool recoveryLift;
    bool segmentLoaded,endpointQueued;
};
MotionTask task{};
#ifdef __AVR__
static_assert(sizeof(MotionTask)==104,"motion payload grew beyond audited AVR budget");
#endif
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
        p=index==0?TeachPoint{task.cart.start.x,task.cart.start.y}:TeachPoint{task.cart.end.x,task.cart.end.y}; return true;
    }
    if(task.status.kind==TaskKind::DrawShape)
        return shapeVertex(task.shape,index,{task.cart.start.x,task.cart.start.y},{task.cart.end.x,task.cart.end.y},p);
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
    // Geometric knots are independent of the 20 ms output clock.
    // Retain every vertex; validate the same joint chords that will execute.
    float steps=ceilf(length/5.0f);
    if(!isfinite(steps) || length<0.1f || steps>4096) return 0;
    return max(1,static_cast<int>(steps));
}
bool pointPose(const Point3D& point,Pose& pose) {
    IKResult3D ik=solveIK3D(point);
    if(!ik.valid) return rejectControl(ControlError::Unreachable);
    pose.angles[Base]=ik.j0; pose.angles[Shoulder]=ik.j1; pose.angles[Elbow]=ik.j2; return true;
}
bool drawGoal(uint16_t index,Pose& pose) {
    task.drawCheckpoint=true;
    task.segmentDistance=0;
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
        if(index==3) task.segmentDistance=teachSettings().travelZ-teachSettings().drawZ;
        return pointPose({point.x,point.y,index==3?teachSettings().drawZ:teachSettings().travelZ},pose);
    }
    index-=3;
    for(uint8_t segment=0;segment<drawCount()-1;++segment) {
        uint16_t steps=strokeSteps(segment);
        if(!steps) return rejectControl(ControlError::Invalid);
        if(index<=steps) {
            task.drawCheckpoint=index==steps; // Keep all taught vertices/corners.
            float t=static_cast<float>(index)/steps;
            TeachPoint a,b,c,d;
            if(task.status.kind==TaskKind::Curve) {
                if(!curveKnots(segment,a,b,c,d) || !centripetalSample(a,b,c,d,t,point)) return rejectControl(ControlError::Invalid);
                TeachPoint previous;
                if(!centripetalSample(a,b,c,d,static_cast<float>(index-1)/steps,previous)) return rejectControl(ControlError::Invalid);
                task.segmentDistance=pointDistance(previous,point);
            } else {
                if(!drawVertex(segment,a) || !drawVertex(segment+1,b)) return rejectControl(ControlError::Invalid);
                point={a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};
                task.segmentDistance=pointDistance(a,b)/steps;
            }
            return pointPose({point.x,point.y,teachSettings().drawZ},pose);
        }
        index-=steps;
    }
    if(index==1) {
        if(!drawVertex(drawCount()-1,point)) return rejectControl(ControlError::Invalid);
        task.segmentDistance=teachSettings().travelZ-teachSettings().drawZ;
        return pointPose({point.x,point.y,teachSettings().travelZ},pose);
    }
    --index;
    if(index>homeCount) return rejectControl(ControlError::Invalid);
    pose=index<homeCount?hardwareConfig.homeVia[index-1]:hardwareConfig.home; return true;
}
__attribute__((noinline)) float maxDifference(const Pose& a,const Pose& b) {
    float d=0; for(uint8_t i=0;i<JointCount;++i) d=max(d,fabsf(a.angles[i]-b.angles[i])); return d;
}
__attribute__((noinline)) Pose interpolate(const Pose& a,const Pose& b,float t) {
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
        uint8_t homeCount=hardwareConfig.homeViaCount+1;
        if(index<=homeCount) { pose=index<homeCount?hardwareConfig.homeVia[index-1]:hardwareConfig.home; return true; }
        uint16_t dwell; return getPickStage(task.objectIndex,index-homeCount-1,pose,dwell) || rejectControl(ControlError::Uncalibrated);
    }
    if(task.status.kind==TaskKind::Playback) {
        RecordedFrame frame; if(!getRecordedFrame(index-1,frame)) return rejectControl(ControlError::NoData);
        for(uint8_t i=0;i<JointCount;++i) pose.angles[i]=frame.angles[i];
        return true;
    }
    if(isDrawingKind(task.status.kind)) return drawGoal(index,pose);
    Point3D p={task.cart.start.x+(task.cart.end.x-task.cart.start.x)*t,
               task.cart.start.y+(task.cart.end.y-task.cart.start.y)*t,
               task.cart.start.z+(task.cart.end.z-task.cart.start.z)*t};
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
    if(failed && task.status.state==MotionState::Validating) {
        printText(F("PATH ERR ")); printUnsigned(task.status.validatedWaypoints+1);
        printChar('/'); printlnUnsigned(task.validationSubstep);
    }
    task.status.state=MotionState::Idle;
    task.segmentLoaded=false; task.endpointQueued=false;
    if(failed) { holdArm(); printControlError(); }
    else {
        printText(F("DONE "));
        printCommandName(commandNames("NONE\0LINE\0RUNLINE\0MOVE\0XYZ\0HOME\0PICK\0PLAY\0DRAW LINE\0DRAW SHAPE\0DRAW POLY\0DRAW CURVE\0"),static_cast<uint8_t>(kind));
    }
    syncManualTarget(millis());
}
bool begin(TaskKind kind,uint16_t count) {
    if(isMotionBusy() || hasPendingPose() || isRecording() || isPickSaving()) return rejectControl(ControlError::Busy);
    if(isArmHeld()) return rejectControl(ControlError::Held);
    task=MotionTask{}; task.status.kind=kind; task.status.state=MotionState::Validating;
    task.status.totalWaypoints=count; task.origin=commandedPose(); task.segmentStart=task.origin;
    clearControlError(); return true;
}
__attribute__((noinline)) void validateOne() {
    if(!task.segmentLoaded) {
        task.validationSubstep=0;
        if(!sampleGoal(task.status.validatedWaypoints+1,task.segmentTarget)) { finish(true); return; }
        task.validationSubsteps=max(1,static_cast<int>(ceilf(maxDifference(task.segmentStart,task.segmentTarget))));
        task.validationSubstep=0; task.segmentLoaded=true;
        if(task.status.kind==TaskKind::Playback && task.status.validatedWaypoints>0) {
            RecordedFrame a,b;
            getRecordedFrame(task.status.validatedWaypoints-1,a); getRecordedFrame(task.status.validatedWaypoints,b);
            // A uniform, reported time scale, independent of H/L and SPEED.
            task.cart.start.x=max(task.cart.start.x,maxDifference(task.segmentStart,task.segmentTarget)*1000.0f/(120.0f*(b.timeMs-a.timeMs)));
        }
    }
    ++task.validationSubstep;
    Pose p=interpolate(task.segmentStart,task.segmentTarget,static_cast<float>(task.validationSubstep)/task.validationSubsteps), out;
    if(!prepareOutputPose(p,out)) {
        rejectControl(ControlError::Unsafe);
        if(task.status.kind==TaskKind::LineExecution) printLine(F("RUNLINE ABORTED (preflight)"));
        finish(true); return;
    }
    if(task.validationSubstep==task.validationSubsteps) {
        task.segmentStart=task.segmentTarget; task.segmentLoaded=false;
        ++task.status.validatedWaypoints;
        if(task.status.validatedWaypoints==task.status.totalWaypoints) {
            if(task.status.kind==TaskKind::LinePreview) finish();
            else {
                task.status.state=MotionState::Running; task.segmentTarget=task.origin; task.lastUpdate=millis(); printLine(F("PATH VALID"));
                if(task.status.kind==TaskKind::Playback && task.cart.start.x>1) {
                    printText(F("PLAY timeScale=")); printDecimal(task.cart.start.x); printNewline();
                }
            }
        }
    }
}
__attribute__((noinline)) void executePlayback(uint32_t now) {
    // First frame has already been approached and acknowledged. Carry clock
    // overshoot across keyframes instead of rounding every segment to a tick.
    task.elapsed+=now-task.lastUpdate; task.lastUpdate=now;
    float time=task.elapsed/task.cart.start.x;
    RecordedFrame a,b;
    uint8_t index=task.status.completedWaypoints-1;
    getRecordedFrame(index,a);
    while(static_cast<uint16_t>(index+1)<task.status.totalWaypoints) {
        getRecordedFrame(index+1,b);
        if(time<b.timeMs) break;
        a=b; ++index;
    }
    task.status.completedWaypoints=index+1;
    bool ended=static_cast<uint16_t>(index+1)==task.status.totalWaypoints;
    float t=ended?0:(time-a.timeMs)/(b.timeMs-a.timeMs);
    for(uint8_t j=0;j<JointCount;++j) {
        task.segmentTarget.angles[j]=ended?a.angles[j]:a.angles[j]+(static_cast<int>(b.angles[j])-a.angles[j])*t;
    }
    if(!requestPose(task.segmentTarget)) { rejectControl(ControlError::Unsafe); finish(true); return; }
    // Reuse normal endpoint acknowledgement without double-counting the end.
    if(ended) { --task.status.completedWaypoints; task.endpointQueued=true; }
}
__attribute__((noinline)) void execute(uint32_t now) {
    if(task.endpointQueued) return;
    if(task.status.kind==TaskKind::Playback && task.status.completedWaypoints>0) { executePlayback(now); return; }
    bool drawing=isDrawingKind(task.status.kind);
    uint32_t dt=min(static_cast<uint32_t>(now-task.lastUpdate),MANUAL_MAX_DT_MS); task.lastUpdate=now;
    // Bounded work, one output frame. Fine geometric samples may share a tick;
    // mandatory HOME/pen/vertex endpoints still wait for output acknowledgement.
    for(uint8_t advanced=0;advanced<16;++advanced) {
        if(!task.segmentLoaded) {
            // Preserve the continuous drawing trajectory across integer outputs.
            task.segmentStart=drawing?task.segmentTarget:commandedPose();
            uint16_t index=task.status.completedWaypoints+1;
            if(!sampleGoal(index,task.segmentTarget)) { finish(true); return; }
            if(!drawing) task.elapsed=0;
            task.segmentLoaded=true;
            uint16_t penDown=hardwareConfig.homeViaCount+4;
            uint16_t penUp=task.status.totalWaypoints-hardwareConfig.homeViaCount-1;
            bool stroke=drawing && index>penDown && index<penUp;
            float speed=task.status.kind==TaskKind::Playback?DEFAULT_MOVE_SPEED:automaticSpeed();
            if(task.status.kind==TaskKind::Home || task.status.kind==TaskKind::PickPlace || drawing) speed=DEFAULT_MOVE_SPEED;
            if(task.status.kind==TaskKind::PickPlace && task.status.completedWaypoints>hardwareConfig.homeViaCount)
                speed=pickStageSpeed(task.objectIndex,task.status.completedWaypoints-hardwareConfig.homeViaCount-1);
            task.duration=stroke?1:WAYPOINT_DELAY_MS;
#ifdef MEARM_FIVE_FAST
            // Experimental only: preserve every position/IK/output check.
            if(!(stroke && (task.status.kind==TaskKind::Polyline || task.status.kind==TaskKind::Curve)))
#endif
                task.duration=max(task.duration,static_cast<uint32_t>(ceilf(maxDifference(task.segmentStart,task.segmentTarget)*1000.0f/speed)));
            // Fixed stroke speed; preserve the existing pen-down/up timing.
            // HOME, air travel and return keep their joint-speed timing.
            if(drawing && index>=penDown && index<=penUp) {
                // The sampler already knows the Cartesian chord. Reconstructing
                // both endpoints through FK adds six trigonometric evaluations
                // per endpoint and introduces IK round-trip tolerance into timing.
                const float paperSpeed=stroke
                    ? ((task.status.kind==TaskKind::Polyline || task.status.kind==TaskKind::Curve) ? kFivePointDrawSpeedMmS : kDrawSpeedMmS)
                    : kPenTransitionSpeedMmS;
                task.duration=max(task.duration,static_cast<uint32_t>(ceilf(task.segmentDistance*1000.0f/paperSpeed)));
            }
        }
        task.elapsed+=dt; dt=0;
        bool reached=task.elapsed>=task.duration;
        Pose p=interpolate(task.segmentStart,task.segmentTarget,reached?1.0f:static_cast<float>(task.elapsed)/task.duration);
        // Validate even skipped geometric endpoints; Servo is applied once by main.
        if(!requestPose(p)) { rejectControl(ControlError::Unsafe); finish(true); return; }
        if(!reached) return;
        if(!drawing || task.drawCheckpoint) {
            task.elapsed=task.duration; task.endpointQueued=true; return;
        }
        task.elapsed-=task.duration; ++task.status.completedWaypoints; task.segmentLoaded=false;
    }
    // Saturation slows the path instead of accumulating a future catch-up jump.
    task.elapsed=0;
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
    task.cart.start={start.x,0,start.z}; task.cart.end={target.x,0,target.z}; return true;
}
bool startCartesianMove(const Point3D& target) {
    if(!cartesianCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    Point3D start=solveFK3D(joints[Base].commandedAngle,joints[Shoulder].commandedAngle,joints[Elbow].commandedAngle);
    float dx=target.x-start.x,dy=target.y-start.y,dz=target.z-start.z;
    float n=ceilf(sqrtf(dx*dx+dy*dy+dz*dz)/2.0f);
    if(!isfinite(n) || n>32767) return rejectControl(ControlError::Invalid);
    if(!begin(TaskKind::CartesianMove,max(1,static_cast<int>(n)))) return false;
    task.cart.start=start; task.cart.end=target; return true;
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
        if(isDrawingKind(task.status.kind)) task.elapsed=0;
        if(task.status.kind==TaskKind::Playback && task.status.completedWaypoints==1) {
            task.elapsed=0; task.lastUpdate=now; printLine(F("PLAY START"));
        }
        if(task.status.kind==TaskKind::PickPlace && task.status.completedWaypoints>static_cast<uint16_t>(hardwareConfig.homeViaCount+1))
            task.dwellRemaining=pickStageDwell(task.objectIndex,task.status.completedWaypoints-hardwareConfig.homeViaCount-2);
#ifdef MEARM_FIVE_FAST
        // Only the three interior POLY vertices hold; CURVE remains continuous.
        if(task.status.kind==TaskKind::Polyline && task.status.completedWaypoints>hardwareConfig.homeViaCount+4 &&
           task.status.completedWaypoints<task.status.totalWaypoints-hardwareConfig.homeViaCount-2)
            task.dwellRemaining=MEARM_POLY_HOLD_MS;
#endif
        if(task.status.completedWaypoints==task.status.totalWaypoints && !task.dwellRemaining) { finish(); return; }
        if(task.dwellRemaining) { task.lastUpdate=now; return; }
    }
    if(controlDue) execute(now);
}
bool pauseMotion(uint32_t now) {
    if(!isMotionBusy()) return false;
    if(task.status.state==MotionState::Paused) return true;
    if(task.status.state==MotionState::Running && task.status.kind==TaskKind::Playback && task.status.completedWaypoints>0) task.elapsed+=now-task.lastUpdate;
    task.resumeState=task.status.state; task.status.state=MotionState::Paused;
    task.lastUpdate=now;
    // HOLD discards pending output. It must not acknowledge an unwritten end.
    if(hasPendingPose()) task.endpointQueued=false;
    holdArm(); return true;
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
    if(!pickCalibrationValid(object) || !homeCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    if(!begin(TaskKind::PickPlace,PICK_STAGE_COUNT+hardwareConfig.homeViaCount+1)) return false;
    task.objectIndex=object; return true;
}
bool startPlayback() {
    if(!hasRecording()) return rejectControl(ControlError::NoData);
    if(!begin(TaskKind::Playback,recordingStatus().count)) return false;
    task.cart.start.x=1; return true;
}
bool cancelAndReturnHome() {
    // A repeated HOME during draw recovery must retain the required lift.
    bool lift=drawingTaskActive() || (isMotionBusy() && task.recoveryLift);
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
    task.shape=shape; task.cart.start={a.x,a.y,0}; task.cart.end={b.x,b.y,0};
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
bool startBasicShape(ShapeId shape,const TeachPoint& lo,const TeachPoint& hi) {
    // Reject thin rectangles instead of reporting a near-line as a 2D shape.
    if(hi.x-lo.x<5.0f || hi.y-lo.y<5.0f) return rejectControl(ControlError::Geometry);
    return startDrawing(TaskKind::DrawShape,shape,lo,hi);
}
bool startPolyline() {
    if(teachPointCount()!=TEACH_POINT_COUNT) return rejectControl(ControlError::NoData);
    return startDrawing(TaskKind::Polyline,ShapeId::N,{},{});
}
bool startCurve() {
    if(teachPointCount()!=TEACH_POINT_COUNT) return rejectControl(ControlError::NoData);
    return startDrawing(TaskKind::Curve,ShapeId::N,{},{});
}
