#include "teaching.h"
#include "arm.h"
#include "hardware_config.h"
#include "manual_control.h"
#include "motion.h"
#include "recording.h"
#include "control_error.h"
#include "pick_place.h"
#include <math.h>
namespace {
TeachPoint points[TEACH_POINT_COUNT];
uint8_t order[TEACH_POINT_COUNT]={0,1,2,3,4},count=0;
TeachSettings settings{};
bool active=false,marking=false;
bool mutableNow() {
    if(isMotionBusy() || hasPendingPose() || isRecording() || isPickSaving()) return rejectControl(ControlError::Busy);
    return true;
}
Point3D currentPoint() {
    Pose p=commandedPose(); return solveFK3D(p.angles[Base],p.angles[Shoulder],p.angles[Elbow]);
}
}
bool teachingActive() { return active; }
bool markingActive() { return active && marking; }
uint8_t teachPointCount() { return count; }
const TeachSettings& teachSettings() { return settings; }
bool drawSettingsValid() {
    return cartesianCalibrationValid() && settings.drawValid && settings.travelValid &&
        isfinite(settings.drawZ) && isfinite(settings.travelZ) && settings.travelZ>settings.drawZ;
}
void initializeTeaching() { active=marking=false; settings=TeachSettings{}; count=0; for(uint8_t i=0;i<5;++i) order[i]=i; }
bool beginTeach() {
    if(!mutableNow()) return false;
    if(!cartesianCalibrationValid()) return rejectControl(ControlError::Uncalibrated);
    if(!setControlMode(ControlMode::Cartesian)) return false;
    active=true; marking=false; syncManualTarget(millis()); return true;
}
static bool captureHeight(float& z,bool& valid) {
    if(!mutableNow()) return false;
    if(!active || marking) return rejectControl(ControlError::TeachState);
    Point3D p=currentPoint(); if(!isfinite(p.z)) return rejectControl(ControlError::Uncalibrated);
    z=p.z; valid=true;
    return true;
}
bool captureTeachHeight(bool drawing) { return drawing?captureHeight(settings.drawZ,settings.drawValid):captureHeight(settings.travelZ,settings.travelValid); }
bool captureTeachAlignment() { return captureHeight(settings.teachZ,settings.teachValid); }
bool beginMarking() {
    if(!mutableNow()) return false;
    if(!active) return rejectControl(ControlError::TeachState);
    if(!drawSettingsValid() || !settings.teachValid || !isfinite(settings.teachZ) || settings.teachZ<=settings.drawZ || settings.teachZ>settings.travelZ) return rejectControl(ControlError::TeachHeights);
    Point3D p=currentPoint();
    // Do not silently move vertically when entering marking. Capture travelZ at
    // this pose, then explicitly capture TEACHZ and enter centered.
    if(fabsf(p.z-settings.teachZ)>0.05f || !joysticksCentered(readJoysticks())) return rejectControl(ControlError::Invalid);
    marking=true; syncManualTarget(millis()); return true;
}
bool markTeachPoint() {
    if(!mutableNow()) return false;
    if(!markingActive()) return rejectControl(ControlError::MarkState);
    if(count==TEACH_POINT_COUNT) return rejectControl(ControlError::Full);
    Point3D p=currentPoint(); if(!isfinite(p.x) || !isfinite(p.y)) return rejectControl(ControlError::Uncalibrated);
    for(uint8_t i=0;i<count;++i) {
        float dx=p.x-points[i].x,dy=p.y-points[i].y;
        if(dx*dx+dy*dy<0.01f) return rejectControl(ControlError::Invalid);
    }
    points[count]={p.x,p.y}; order[count]=count; ++count; return true;
}
bool clearTeachPoints() {
    if(!mutableNow()) return false;
    count=0; marking=false; for(uint8_t i=0;i<5;++i) order[i]=i; return true;
}
bool undoTeachPoint() {
    if(!mutableNow()) return false;
    if(!markingActive() || !count) return rejectControl(ControlError::NoData);
    --count;
    uint8_t out=0;
    for(uint8_t i=0;i<=count;++i) if(order[i]!=count) order[out++]=order[i];
    order[count]=count;
    return true;
}
bool exitTeach() {
    if(!mutableNow()) return false;
    active=marking=false; syncManualTarget(millis()); return true;
}
bool setTeachOrder(const uint8_t (&sequence)[TEACH_POINT_COUNT]) {
    if(!mutableNow()) return false;
    if(count!=TEACH_POINT_COUNT) return rejectControl(ControlError::NoData);
    uint8_t seen=0;
    for(uint8_t i=0;i<5;++i) {
        if(sequence[i]<1 || sequence[i]>5 || (seen&(1<<(sequence[i]-1)))) return rejectControl(ControlError::Invalid);
        seen|=1<<(sequence[i]-1);
    }
    for(uint8_t i=0;i<5;++i) order[i]=sequence[i]-1;
    return true;
}
bool getTeachPoint(uint8_t index,TeachPoint& out) {
    if(index>=count) return false;
    out=points[order[index]]; return true;
}
