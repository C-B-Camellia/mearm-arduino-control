#pragma once

#include <Arduino.h>
#include "kinematics.h"
#include "arm.h"
#include "path_planner.h"

enum class TaskKind : uint8_t {
    None,
    LinePreview,
    LineExecution,
    JointMove,
    CartesianMove,
    Home,
    PickPlace,
    Playback,
    DrawLine,
    DrawShape,
    Polyline,
    Curve,
    TeachPosition
};

enum class MotionState : uint8_t {
    Idle,
    Validating,
    Running,
    Paused,
    Waiting
};

struct MotionStatus {
    TaskKind kind;
    MotionState state;
    uint16_t validatedWaypoints;
    uint16_t completedWaypoints;
    uint16_t totalWaypoints;
};

bool isMotionBusy();
MotionStatus motionStatus();
bool startLine(Point2D target, TaskKind kind);
bool startJointMove(const Pose& target);
bool startCartesianMove(const Point3D& target,TaskKind kind=TaskKind::CartesianMove);
bool startTeachPosition();
bool startHome();
bool startPickPlace(uint8_t object);
bool startPlayback();
bool cancelAndReturnHome();
bool startDrawLine(const TeachPoint& from,const TeachPoint& to);
bool startBasicShape(ShapeId shape,const TeachPoint& low,const TeachPoint& high);
bool startPolyline();
bool startCurve();
bool drawingTaskActive();
void updateMotion(uint32_t now, bool controlDue);
bool pauseMotion(uint32_t now);
bool resumeMotion(uint32_t now);
void cancelMotion();

// Every automatic behavior shares this owner, including paused/validating tasks.
