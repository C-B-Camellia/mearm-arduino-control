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
    int joystickPin;
    int servoPin;
    int center;

    // Software command state, not measured position feedback.
    float commandedAngle;
    float goalAngle;
    float minAngle;
    float maxAngle;

    int directionSign;
    bool outputEnabled;
};

struct Pose {
    float angles[JointCount];
};

constexpr float J1_J2_MIN_SUM = 148.0f;

extern JointControl joints[JointCount];
extern Servo servos[JointCount];

void initializeArm();
Pose commandedPose();
Pose goalPose();
bool isSafePlanarPose(float shoulderAngle, float elbowAngle);
bool isSafePose(const Pose& pose);

// Queue ordinary motion, or send a validated trajectory waypoint immediately.
bool requestPose(const Pose& pose);
bool applyPoseImmediate(const Pose& pose);
bool applyPendingPose();
bool hasPendingPose();
bool isArmHeld();
void holdArm();
void resumeArm();

// Future joystick control will use joystickPin/center/directionSign, build a
// complete Pose, and submit it here. Gripper/base outputs remain disabled until
// their behavior is implemented; do not bypass the pose checks or Servo writes.
