#pragma once

#include <Arduino.h>
#include <Servo.h>

enum JointId : uint8_t {
    Base,
    Shoulder,
    Elbow,
    Gripper,
    JointCount
};

struct JointControl {
    int servoPin;

    // Software command state, not measured position feedback.
    float commandedAngle;
    float goalAngle;
    float minAngle;
    float maxAngle;

    bool outputEnabled;
};

struct Pose {
    float angles[JointCount];
};

#ifndef MEARM_J1_J2_MIN_SUM
#define MEARM_J1_J2_MIN_SUM 148.0f
#endif
#ifndef MEARM_J1_J2_MAX_SUM
#define MEARM_J1_J2_MAX_SUM 250.0f
#endif
constexpr float J1_J2_MIN_SUM = MEARM_J1_J2_MIN_SUM;
constexpr float J1_J2_MAX_SUM = MEARM_J1_J2_MAX_SUM; // 临时上界，非碰撞实测值

extern JointControl joints[JointCount];

void initializeArm();
Pose commandedPose();
Pose goalPose();
bool isSafePlanarPose(float shoulderAngle, float elbowAngle);
bool isSafePose(const Pose& pose);
bool prepareOutputPose(const Pose& candidate, Pose& output);

// Queue ordinary motion, or send a validated trajectory waypoint immediately.
bool requestPose(const Pose& pose);
bool applyPendingPose();
bool hasPendingPose();
bool isArmHeld();
void holdArm();
void resumeArm();

// All sources submit complete poses; only arm owns Servo output. Base/gripper
// require explicitly valid hardware calibration before channels are attached.
