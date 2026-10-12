#include "arm.h"

#include <math.h>

// Original master mapping, with this branch's calibrated elbow startup (95).
// Fields: joystick pin, servo pin, center, commanded/goal angles, limits,
//         direction sign, output enabled.
JointControl joints[JointCount] = {
    {A0, 9, 506,  90.0f,  90.0f,  0.0f, 180.0f,  1, false}, // Base
    {A1, 8, 519, 120.0f, 120.0f, 20.0f, 125.0f, -1, true},  // Shoulder / J1
    {A3, 7, 519,  95.0f,  95.0f, 40.0f, 130.0f,  1, true},  // Elbow / J2
    {A2, 6, 515,  50.0f,  50.0f,  0.0f, 180.0f,  1, false}  // Gripper
};
Servo servos[JointCount];

namespace {
bool held = false;
bool posePending = true;

bool withinLimits(float angle, const JointControl& joint)
{
    return isfinite(angle) && angle >= joint.minAngle && angle <= joint.maxAngle;
}

bool canApplyPose(const Pose& pose)
{
    if (!isSafePose(pose)) {
        return false;
    }

    // Never report a changed command angle for a servo that cannot receive it.
    for (uint8_t i = 0; i < JointCount; ++i) {
        if (!joints[i].outputEnabled && pose.angles[i] != joints[i].commandedAngle) {
            return false;
        }
    }
    return true;
}
}

void initializeArm()
{
    Pose output;
    if (!prepareOutputPose(commandedPose(), output)) {
        held = true;
        posePending = false;
        return;
    }
    for (uint8_t i = 0; i < JointCount; ++i) {
        joints[i].commandedAngle = output.angles[i];
        if (joints[i].outputEnabled) {
            servos[i].attach(joints[i].servoPin);
            servos[i].write(static_cast<int>(joints[i].commandedAngle));
        }
    }
    held = false;
    posePending = true;
}

Pose commandedPose()
{
    Pose pose;
    for (uint8_t i = 0; i < JointCount; ++i) {
        pose.angles[i] = joints[i].commandedAngle;
    }
    return pose;
}

Pose goalPose()
{
    Pose pose;
    for (uint8_t i = 0; i < JointCount; ++i) {
        pose.angles[i] = joints[i].goalAngle;
    }
    return pose;
}

bool isSafePlanarPose(float shoulderAngle, float elbowAngle)
{
    return withinLimits(shoulderAngle, joints[Shoulder]) &&
           withinLimits(elbowAngle, joints[Elbow]) &&
           shoulderAngle + elbowAngle >= J1_J2_MIN_SUM;
}

bool isSafePose(const Pose& pose)
{
    for (uint8_t i = 0; i < JointCount; ++i) {
        if (!withinLimits(pose.angles[i], joints[i])) {
            return false;
        }
    }
    return isSafePlanarPose(pose.angles[Shoulder], pose.angles[Elbow]);
}

bool requestPose(const Pose& pose)
{
    Pose output;
    if (held || !prepareOutputPose(pose, output)) {
        return false;
    }
    for (uint8_t i = 0; i < JointCount; ++i) {
        joints[i].goalAngle = pose.angles[i];
    }
    posePending = true;
    return true;
}

bool applyPoseImmediate(const Pose& pose)
{
    Pose output;
    if (!prepareOutputPose(pose, output)) {
        return false;
    }

    // Update both software angles before sending either servo command.
    for (uint8_t i = 0; i < JointCount; ++i) {
        joints[i].commandedAngle = output.angles[i];
        joints[i].goalAngle = pose.angles[i];
    }
    for (uint8_t i = 0; i < JointCount; ++i) {
        if (joints[i].outputEnabled) {
            servos[i].write(static_cast<int>(output.angles[i]));
        }
    }
    posePending = false;
    return true;
}

bool prepareOutputPose(const Pose& candidate, Pose& output)
{
    if (!canApplyPose(candidate)) return false;
    Pose quantized;
    for (uint8_t i = 0; i < JointCount; ++i) {
        quantized.angles[i] = static_cast<int>(candidate.angles[i]);
    }
    if (!canApplyPose(quantized)) return false;
    output = quantized;
    return true;
}

bool applyPendingPose()
{
    if (held || !posePending) {
        return true;
    }
    if (!applyPoseImmediate(goalPose())) {
        return false;
    }
    return true;
}

bool hasPendingPose()
{
    return posePending;
}

bool isArmHeld()
{
    return held;
}

void holdArm()
{
    held = true;
    posePending = false;
    for (uint8_t i = 0; i < JointCount; ++i) {
        joints[i].goalAngle = joints[i].commandedAngle;
    }
}

void resumeArm()
{
    held = false;
}
