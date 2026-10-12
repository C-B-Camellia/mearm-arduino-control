#include "recording.h"
#include "motion.h"
#include "control_error.h"
#include <math.h>
namespace {
RecordedFrame frames[RECORD_CAPACITY];
RecordingStatus status{};
uint32_t startedAt=0,lastSample=0;
bool append(uint32_t now,const Pose& pose) {
    if(status.count==RECORD_CAPACITY || now-startedAt>65535) return false;
    RecordedFrame f{};
    for(uint8_t i=0;i<JointCount;++i) {
        if(!isfinite(pose.angles[i]) || pose.angles[i]<0 || pose.angles[i]>180) return false;
        f.angles[i]=static_cast<uint8_t>(pose.angles[i]);
    }
    f.timeMs=now-startedAt; frames[status.count++]=f; status.duration=f.timeMs; lastSample=now; return true;
}
bool finalize() {
    status.state=RecordState::Empty;
    if(status.duration<=10000) return rejectControl(ControlError::TooShort);
    bool moved=false;
    for(uint8_t joint=0;joint<JointCount;++joint) {
        uint8_t low=180,high=0;
        for(uint8_t i=0;i<status.count;++i) { low=min(low,frames[i].angles[joint]); high=max(high,frames[i].angles[joint]); }
        moved=moved || high-low>=5;
    }
    if(!moved) return rejectControl(ControlError::NoMovement);
    status.state=RecordState::Ready; clearControlError(); return true;
}
}
bool isRecording() { return status.state==RecordState::Recording; }
bool hasRecording() { return status.state==RecordState::Ready; }
RecordingStatus recordingStatus() { return status; }
void clearRecording() { status=RecordingStatus{}; }
bool getRecordedFrame(uint8_t index,RecordedFrame& out) {
    if(!hasRecording() || index>=status.count) return false;
    out=frames[index]; return true;
}
bool startRecording(uint32_t now) {
    if(isRecording() || isMotionBusy() || hasPendingPose()) return rejectControl(ControlError::Busy);
    if(isArmHeld()) return rejectControl(ControlError::Held);
    status=RecordingStatus{}; status.state=RecordState::Recording; startedAt=lastSample=now;
    if(!append(now,commandedPose())) { clearRecording(); return rejectControl(ControlError::Unsafe); }
    clearControlError(); return true;
}
bool stopRecording(uint32_t now) {
    if(!isRecording()) return rejectControl(ControlError::NoData);
    if(now!=lastSample && status.count<RECORD_CAPACITY) {
        if(!append(now,commandedPose())) { status.state=RecordState::Empty; return rejectControl(ControlError::Full); }
    }
    return finalize();
}
void updateRecording(uint32_t now,const Pose& pose) {
    if(!isRecording() || now-lastSample<RECORD_INTERVAL_MS) return;
    if(!append(now,pose)) { status.state=RecordState::Empty; rejectControl(ControlError::Full); printControlError(); return; }
    if(status.count==RECORD_CAPACITY) {
        Serial.println(F("REC FULL: stopped")); if(!finalize()) printControlError();
    }
}
