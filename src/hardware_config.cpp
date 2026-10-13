#include "hardware_config.h"
#include <math.h>
// Enter independently verified hardware measurements here; do not copy nominal
// 0..180 limits, startup angles or guessed offsets into valid configurations.
#ifndef MEARM_NATIVE_TEST
const
#endif
HardwareConfig hardwareConfig{
    // J0 Base：首次测试暂时限制在 80~100°
    {true, 0.0f, 180.0f, 80.0f},

    // J3 Gripper：仍未标定
    {true, 0.0f, 180.0f, 50.0f},

    // 三维几何：已填实测值，但暂不启用 CART
    {
        true,
        ToolModel::FixedPlanar,
        +1,          // yawSign：待测
        79.67f,       // yawZero：待测
        14.75f,     // shoulderRadial
        63.0f,      // shoulderHeight
        53.0f,      // toolRadial
        -4.65f,     // toolTangential
        -6.0f       // toolVertical
    },

    true,                          // gripValid
    40.0f,                         // gripOpen
    90.0f,                         // gripClosed

    true,                          // homeValid
    {{90.0f, 120.0f, 60.0f, 85.0f}}, // HOME: J0 J1 J2 J3

    0,                             // homeViaCount
    {}
};
namespace {
bool validJoint(const JointCalibration& c) {
    return c.valid && isfinite(c.minimum) && isfinite(c.maximum) && isfinite(c.startup) &&
        c.minimum>=0 && c.maximum<=180 && c.minimum<=c.startup && c.startup<=c.maximum && c.minimum<c.maximum;
}
}
void configureCalibratedJoints() {
    const JointCalibration* configs[]={&hardwareConfig.base,&hardwareConfig.gripper};
    const JointId ids[]={Base,Gripper};
    for(uint8_t i=0;i<2;++i) {
        JointControl& joint=joints[ids[i]]; joint.outputEnabled=validJoint(*configs[i]);
        if(joint.outputEnabled) {
            joint.minAngle=configs[i]->minimum; joint.maxAngle=configs[i]->maximum;
            joint.commandedAngle=joint.goalAngle=configs[i]->startup;
        }
    }
}
bool cartesianCalibrationValid() {
    const CartesianCalibration& c=hardwareConfig.cartesian;
    return validJoint(hardwareConfig.base) && c.valid &&
        (c.toolModel==ToolModel::FixedPlanar || c.toolModel==ToolModel::DistalLink) &&
        (c.yawSign==1 || c.yawSign==-1) && isfinite(c.yawZero) && isfinite(c.shoulderRadial) &&
        isfinite(c.shoulderHeight) && isfinite(c.toolRadial) && isfinite(c.toolTangential) && isfinite(c.toolVertical);
}
bool gripperCalibrationValid() {
    return validJoint(hardwareConfig.gripper) && hardwareConfig.gripValid &&
        isfinite(hardwareConfig.gripOpen) && isfinite(hardwareConfig.gripClosed) &&
        hardwareConfig.gripOpen>=hardwareConfig.gripper.minimum && hardwareConfig.gripOpen<=hardwareConfig.gripper.maximum &&
        hardwareConfig.gripClosed>=hardwareConfig.gripper.minimum && hardwareConfig.gripClosed<=hardwareConfig.gripper.maximum;
}
bool homeCalibrationValid() {
    Pose output;
    if(!hardwareConfig.homeValid || hardwareConfig.homeViaCount>2 || !prepareOutputPose(hardwareConfig.home,output)) return false;
    for(uint8_t i=0;i<hardwareConfig.homeViaCount;++i) if(!prepareOutputPose(hardwareConfig.homeVia[i],output)) return false;
    return true;
}
