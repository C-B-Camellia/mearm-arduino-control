#include "hardware_config.h"
#include <math.h>
// Enter independently verified hardware measurements here; do not copy nominal
// 0..180 limits, startup angles or guessed offsets into valid configurations.
#ifndef MEARM_NATIVE_TEST
const
#endif
HardwareConfig hardwareConfig{};
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
