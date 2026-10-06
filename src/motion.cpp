#include "motion.h"
#include "arm.h"
#include "config.h"

#include <math.h>

namespace {
struct LinePath {
    Point2D start;
    float dx;
    float dz;
    uint16_t steps;
};

struct LineTask {
    MotionStatus status;
    LinePath path;
    MotionState resumeState;
    uint32_t waitStartedAt;
    uint32_t remainingWait;
};

LineTask task{};

Point2D lineWaypoint(uint16_t index)
{
    float t = static_cast<float>(index) / task.path.steps;
    Point2D point;
    point.x = task.path.start.x + task.path.dx * t;
    point.z = task.path.start.z + task.path.dz * t;
    return point;
}

void printWaypointPose(const Point2D& point, const IKResult& ik)
{
    Serial.print(point.x, 1);
    Serial.print(' ');
    Serial.print(point.z, 1);
    Serial.print(F(" -> J1="));
    Serial.print(ik.j1, 1);
    Serial.print(F(" J2="));
    Serial.println(ik.j2, 1);
}

void validateNextWaypoint()
{
    uint16_t index = task.status.validatedWaypoints + 1;
    Point2D point = lineWaypoint(index);
    IKResult ik = solveIK(point.x, point.z);
    if (!ik.valid) {
        Serial.print(F("ERR waypoint "));
        Serial.print(index);
        Serial.print(F(" invalid at "));
        Serial.print(point.x, 1);
        Serial.print(' ');
        Serial.println(point.z, 1);
        if (task.status.kind == TaskKind::LineExecution) {
            Serial.println(F("RUNLINE ABORTED before motion"));
        }
        cancelMotion();
        return;
    }

    task.status.validatedWaypoints = index;
    if (task.status.kind == TaskKind::LinePreview) {
        Serial.print(index);
        Serial.print(F(": XY="));
        printWaypointPose(point, ik);
    }

    if (index == task.path.steps) {
        if (task.status.kind == TaskKind::LinePreview) {
            Serial.println(F("LINE VALID"));
            cancelMotion();
        }
        else {
            Serial.println(F("PATH VALID"));
            task.status.state = MotionState::Running;
            task.remainingWait = 0;
        }
    }
}

void executeNextWaypoint(uint32_t now)
{
    if (static_cast<uint32_t>(now - task.waitStartedAt) < task.remainingWait) {
        return;
    }

    // Preserve the complete delay after the final waypoint too.
    if (task.status.completedWaypoints == task.path.steps) {
        Serial.println(F("RUNLINE DONE"));
        cancelMotion();
        return;
    }

    uint16_t index = task.status.completedWaypoints + 1;
    Point2D point = lineWaypoint(index);
    IKResult ik = solveIK(point.x, point.z);
    if (!ik.valid) {
        Serial.println(F("ERR unexpected IK failure"));
        cancelMotion();
        return;
    }

    Serial.print(F("MOVE "));
    Serial.print(index);
    Serial.print('/');
    Serial.print(task.path.steps);
    Serial.print(F(" XY="));
    printWaypointPose(point, ik);

    Pose pose = commandedPose();
    pose.angles[Shoulder] = ik.j1;
    pose.angles[Elbow] = ik.j2;
    if (!applyPoseImmediate(pose)) {
        Serial.println(F("ERR execution safety failure"));
        cancelMotion();
        return;
    }

    task.status.completedWaypoints = index;
    task.waitStartedAt = millis();
    task.remainingWait = WAYPOINT_DELAY_MS;
}
}

bool isMotionBusy()
{
    return task.status.state != MotionState::Idle;
}

MotionStatus motionStatus()
{
    return task.status;
}

bool startLine(Point2D target, TaskKind kind)
{
    if (isMotionBusy() || hasPendingPose()) {
        Serial.println(F("ERR busy"));
        return false;
    }
    if (kind != TaskKind::LinePreview && kind != TaskKind::LineExecution) {
        return false;
    }

    LinePath path;
    path.start = solveFK(joints[Shoulder].commandedAngle,
                         joints[Elbow].commandedAngle).position;
    path.dx = target.x - path.start.x;
    path.dz = target.z - path.start.z;
    float distance = sqrt(path.dx * path.dx + path.dz * path.dz);
    float stepCount = ceil(distance / LINE_STEP_MM);

    // Check before converting to an integer, including AVR's 16-bit range.
    if (!isfinite(target.x) || !isfinite(target.z) ||
        !isfinite(stepCount) || stepCount > 32767.0f) {
        Serial.println(F("ERR line: target too large or non-finite"));
        return false;
    }
    path.steps = max(1, static_cast<int>(stepCount));

    task = LineTask{};
    task.path = path;
    task.status.kind = kind;
    task.status.state = MotionState::Validating;
    task.status.totalWaypoints = path.steps;

    Serial.print(kind == TaskKind::LinePreview ? F("LINE start=(") : F("RUNLINE start=("));
    Serial.print(path.start.x, 1);
    Serial.print(',');
    Serial.print(path.start.z, 1);
    Serial.print(F(") target=("));
    Serial.print(target.x, 1);
    Serial.print(',');
    Serial.print(target.z, 1);
    Serial.print(F(") steps="));
    Serial.println(path.steps);
    return true;
}

void updateMotion(uint32_t now, bool controlDue)
{
    if (task.status.state == MotionState::Validating) {
        validateNextWaypoint();
    }
    else if (task.status.state == MotionState::Running && controlDue) {
        executeNextWaypoint(now);
    }
}

bool pauseMotion(uint32_t now)
{
    if (!isMotionBusy()) {
        return false;
    }
    if (task.status.state == MotionState::Paused) {
        return true;
    }

    task.resumeState = task.status.state;
    if (task.status.state == MotionState::Running) {
        uint32_t elapsed = now - task.waitStartedAt;
        task.remainingWait = elapsed >= task.remainingWait ? 0 : task.remainingWait - elapsed;
    }
    task.status.state = MotionState::Paused;
    return true;
}

bool resumeMotion(uint32_t now)
{
    if (!isMotionBusy()) {
        return false;
    }
    if (task.status.state == MotionState::Paused) {
        task.status.state = task.resumeState;
        task.waitStartedAt = now;
    }
    return true;
}

void cancelMotion()
{
    task = LineTask{};
    holdArm();
}
