#pragma once

#include <Arduino.h>
#include "kinematics.h"

enum class TaskKind : uint8_t {
    None,
    LinePreview,
    LineExecution
};

enum class MotionState : uint8_t {
    Idle,
    Validating,
    Running,
    Paused
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
void updateMotion(uint32_t now, bool controlDue);
bool pauseMotion(uint32_t now);
bool resumeMotion(uint32_t now);
void cancelMotion();

// Future multi-segment drawings and pick/place actions will advance one step
// here and share pause/resume/cancel. Recording will observe commanded poses;
// its bounded storage is added only when the recording feature is implemented.
