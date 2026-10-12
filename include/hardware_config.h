#pragma once
#include "arm.h"
// Measured values only. All new output/motion features are disabled by default.
enum class ToolModel : uint8_t { Uncalibrated, FixedPlanar, DistalLink };
struct JointCalibration { bool valid; float minimum, maximum, startup; };
struct CartesianCalibration {
    bool valid;
    ToolModel toolModel;
    int8_t yawSign;
    float yawZero, shoulderRadial, shoulderHeight;
    float toolRadial, toolTangential, toolVertical;
};
struct HardwareConfig {
    JointCalibration base, gripper;
    CartesianCalibration cartesian;
    bool gripValid;
    float gripOpen, gripClosed;
    bool homeValid;
    Pose home;
    uint8_t homeViaCount;
    Pose homeVia[2];
};
extern HardwareConfig hardwareConfig;
void configureCalibratedJoints();
bool cartesianCalibrationValid();
bool gripperCalibrationValid();
bool homeCalibrationValid();
