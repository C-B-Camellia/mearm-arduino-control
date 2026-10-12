#include "serial_format.h"
#include "recording.h"
#include "motion.h"
#include "control_error.h"
#include "pick_place.h"
#include <math.h>
namespace {
RecordedFrame frames[RECORD_CAPACITY];
RecordingStatus status{};
uint32_t startedAt=0,lastSample=0;
uint8_t reportedSecond=0;
// One bounded pass, at most 78*4 comparisons. No recursion or second buffer.
void compact() {
    uint8_t best=1,cost=181;
    for(uint8_t i=1;i<status.count-1;++i) {
        const RecordedFrame &a=frames[i-1],&b=frames[i],&c=frames[i+1];
        uint16_t span=c.timeMs-a.timeMs,offset=b.timeMs-a.timeMs;
        uint32_t error=0;
        for(uint8_t j=0;j<JointCount;++j) {
            int32_t residual=static_cast<int32_t>(static_cast<int>(a.angles[j])-b.angles[j])*span
                +static_cast<int32_t>(static_cast<int>(c.angles[j])-a.angles[j])*offset;
            uint32_t magnitude=residual<0?-residual:residual;
            error=max(error,magnitude);
        }
        uint8_t rounded=(error+span-1)/span;
        if(rounded<cost) { best=i; cost=rounded; }
    }
    // The removed vertex bounds the difference of the two linear paths.
    // Triangle inequality propagates an intentionally conservative bound.
    status.errorBound=min(180,static_cast<int>(status.errorBound)+static_cast<int>(cost));
    status.degraded=true;
    for(uint8_t i=best;i<status.count-1;++i) frames[i]=frames[i+1];
    --status.count;
}
bool observe(const Pose& pose,uint16_t time) {
    if(!isSafePose(pose)) { clearRecording(); return rejectControl(ControlError::Unsafe); }
    RecordedFrame next{}; next.timeMs=time;
    for(uint8_t j=0;j<JointCount;++j) next.angles[j]=static_cast<uint8_t>(pose.angles[j]);
    if(status.count && time==frames[status.count-1].timeMs) return true;
    bool replace=status.count>=2;
    if(replace) {
        const RecordedFrame &a=frames[status.count-2],&b=frames[status.count-1];
        uint16_t dt=time-a.timeMs,previous=b.timeMs-a.timeMs;
        for(uint8_t j=0;j<JointCount;++j) {
            // Exact temporal collinearity: suppress redundant hold/ramp samples
            // without accumulating approximation error or losing reversals.
            int32_t before=static_cast<int>(b.angles[j])-a.angles[j];
            int32_t delta=static_cast<int>(next.angles[j])-a.angles[j];
            if(before*dt!=delta*previous) replace=false;
        }
        // Keep both endpoints of every observed claw transition.
        if(next.angles[Gripper]!=b.angles[Gripper] || b.angles[Gripper]!=a.angles[Gripper]) replace=false;
    }
    if(replace) frames[status.count-1]=next;
    else {
        if(status.count==RECORD_CAPACITY) compact();
        frames[status.count++]=next;
    }
    status.duration=time; return true;
}
bool finalize() { status.state=RecordState::Ready; printRecordingStatus(); clearControlError(); return true; }
}
void printRecordingStatus() {
    printText(F("REC ")); printUnsigned(static_cast<uint8_t>(status.state));
    printChar(' '); printUnsigned(status.duration);
    printText(F("ms n=")); printUnsigned(status.count);
    printText(F(" err<=")); printUnsigned(status.errorBound);
    Serial.println(status.degraded?F(" DEGRADED"):F(" deg"));
}
bool isRecording() { return status.state==RecordState::Arming || status.state==RecordState::Recording; }
bool hasRecording() { return status.state==RecordState::Ready; }
const RecordingStatus& recordingStatus() { return status; }
void clearRecording() { status=RecordingStatus{}; }
bool getRecordedFrame(uint8_t index,RecordedFrame& out) {
    if(!hasRecording() || index>=status.count) return false;
    out=frames[index]; return true;
}
bool startRecording(uint32_t now) {
    if(isRecording() || isMotionBusy() || hasPendingPose() || isPickSaving()) return rejectControl(ControlError::Busy);
    if(isArmHeld()) return rejectControl(ControlError::Held);
    status=RecordingStatus{}; status.state=RecordState::Arming;
    startedAt=now; reportedSecond=0; printLine(F("REC ARMING 3")); clearControlError(); return true;
}
bool stopRecording(uint32_t now) {
    if(status.state==RecordState::Arming) { clearRecording(); printLine(F("REC CANCEL")); return true; }
    if(!isRecording()) return rejectControl(ControlError::NoData);
    if(!observe(commandedPose(),min(static_cast<uint32_t>(now-startedAt),static_cast<uint32_t>(UINT16_MAX)))) return false;
    return finalize();
}
__attribute__((noinline)) void updateRecording(uint32_t now,const Pose& pose) {
    if(!isRecording()) return;
    uint32_t elapsed=now-startedAt;
    if(status.state==RecordState::Arming) {
        if(elapsed<RECORD_ARMING_MS) {
            uint8_t second=elapsed/1000;
            if(second!=reportedSecond) { reportedSecond=second; printText(F("REC ARMING ")); printlnUnsigned(3-second); }
            return;
        }
        // Start at the actual observation, never backdate an unobserved pose.
        startedAt=lastSample=now; reportedSecond=0;
        status.state=RecordState::Recording; status.errorBound=0;
        if(!observe(pose,0)) { printControlError(); return; }
        printLine(F("REC START 0ms")); return;
    }
    if(now-lastSample<RECORD_INTERVAL_MS) return;
    lastSample=now;
    if(!observe(pose,min(elapsed,static_cast<uint32_t>(UINT16_MAX)))) { printControlError(); return; }
    uint8_t second=status.duration/1000;
    if(second!=reportedSecond) {
        if(second>=10 && reportedSecond<10) printLine(F("REC 10s; B2 SAVE"));
        reportedSecond=second; printRecordingStatus();
    }
    if(elapsed>=RECORD_MAX_MS) { printLine(F("REC MAX")); finalize(); }
}
