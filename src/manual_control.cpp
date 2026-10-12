#include "manual_control.h"
#include "arm.h"
#include "motion.h"
#include "config.h"

namespace {

    ControlMode currentMode = ControlMode::Joint;

}

ControlMode getControlMode() {
    return currentMode;
}

bool setControlMode(ControlMode newMode) {
    if ((newMode == ControlMode::Cartesian || newMode == ControlMode::Joint) && (newMode == currentMode || (!isMotionBusy() && !hasPendingPose() && !isArmHeld()))) {
        currentMode = newMode;
        return true;
    }
    return false;
}

void updateManualControl(uint32_t now) {
    (void)now;
}
