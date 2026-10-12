#include "control_settings.h"
#include "config.h"
#include <math.h>
namespace { float manualScale=1.0f; float autoSpeed=DEFAULT_MOVE_SPEED; }
float manualSpeedScale() { return manualScale; }
bool setManualSpeedScale(float s) {
    if(!isfinite(s) || s<0.75f || s>1.5f) return false;
    manualScale=s; return true;
}
void adjustManualSpeedScale(bool faster) {
    manualScale=constrain(manualScale+(faster?0.25f:-0.25f),0.75f,1.5f);
}
float automaticSpeed() { return autoSpeed; }
bool setAutomaticSpeed(float s) {
    if(!isfinite(s) || s<MIN_MOVE_SPEED || s>MAX_MOVE_SPEED) return false;
    autoSpeed=s; return true;
}
void resetControlSettings() { manualScale=1; autoSpeed=DEFAULT_MOVE_SPEED; }
