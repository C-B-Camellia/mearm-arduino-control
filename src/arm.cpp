#include "arm.h"
#include "hardware_config.h"

#include <math.h>

// Servo targets/limits only. ADC centers and directions live in joystick.cpp.
// Fields: servo pin, commanded/goal angles, limits, output enabled.
JointControl joints[JointCount] = {
    {9,  90.0f,  90.0f,  0.0f, 180.0f, false}, // Base
    {8, 120.0f, 120.0f, 20.0f, 125.0f, true},  // Shoulder / J1
    {7,  60.0f,  60.0f, 40.0f, 150.0f, true},  // Elbow / J2
    {6,  50.0f,  50.0f,  0.0f, 180.0f, false}  // Gripper
};
namespace {
Servo servos[JointCount];
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
    configureCalibratedJoints();
    Pose output;
    if (!prepareOutputPose(commandedPose(), output)) {
        held = true;
        posePending = false;
        return;
    }
    for (uint8_t i = 0; i < JointCount; ++i) {
        joints[i].commandedAngle = output.angles[i];
        if (joints[i].outputEnabled) {
            // Seed the pulse target before enabling the Timer1 channel.
            servos[i].write(static_cast<int>(joints[i].commandedAngle));
            servos[i].attach(joints[i].servoPin);
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
           shoulderAngle + elbowAngle >= J1_J2_MIN_SUM &&
               shoulderAngle + elbowAngle <= J1_J2_MAX_SUM;
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

static bool applyPoseImmediate(const Pose& pose)
{
    Pose output;
    if (!prepareOutputPose(pose, output)) {
        return false;
    }

    // Timer1 keeps emitting the previous pulse without another Servo.write.
    // Still accept the continuous float goal and acknowledge an unchanged frame.
    bool changed=false;
    // Update both software angles before sending either servo command.
    for (uint8_t i = 0; i < JointCount; ++i) {
        if(output.angles[i]!=joints[i].commandedAngle) changed=true;
        joints[i].commandedAngle = output.angles[i];
        joints[i].goalAngle = pose.angles[i];
    }
    for (uint8_t i = 0; i < JointCount; ++i) {
        if (changed && joints[i].outputEnabled) {
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
    // Two independent floors can turn a legal 148.x sum into 147.
    // Restore the integer lower sum, then recheck every physical limit.
    if(quantized.angles[Shoulder]+quantized.angles[Elbow]<J1_J2_MIN_SUM)
        quantized.angles[Elbow]=J1_J2_MIN_SUM-quantized.angles[Shoulder];
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
