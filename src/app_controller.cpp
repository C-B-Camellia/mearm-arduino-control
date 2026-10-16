#include "app_controller.h"
#include "command_text.h"
#include "commands.h"
#include "motion.h"
#include "recording.h"
#include "control_error.h"
#include "teaching.h"
#include "path_planner.h"
#include "serial_format.h"
#include "pick_place.h"
#include "arm.h"
#include "manual_control.h"
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif
namespace { uint8_t nextObject=0; }
Workflow currentWorkflow() { return teachingActive()?(markingActive()?Workflow::TeachMark:Workflow::TeachSetup):Workflow::Normal; }
void initializeApplication() { nextObject=0; clearRecording(); initializeTeaching(); initializePickConfiguration(); }
namespace {
struct DrawPreset { TeachPoint from,to; };
const DrawPreset drawPresets[2] PROGMEM={{{125,30},{125,100}},{{115,-40},{155,10}}};
// Keep preset scratch off the HOLD/CANCEL/teaching call paths.
__attribute__((noinline)) bool startPreset(bool n) {
#ifdef __AVR__
    DrawPreset p; memcpy_P(&p,&drawPresets[n],sizeof(p));
#else
    const DrawPreset& p=drawPresets[n];
#endif
    return n?startBasicShape(ShapeId::N,p.from,p.to):startDrawLine(p.from,p.to);
}
// Heavy service scratch is only needed by the service itself. HOME
// forwarding must not carry this frame into cancellation's IK/FK call chain.
__attribute__((noinline)) bool dispatchServiceAction(const Action& action) {
    switch(action.type) {
    case AppAction::Pick:
        if(teachingActive() || isRecording() || isMotionBusy()) return rejectControl(ControlError::Busy);
        return startPickPlace(action.argument);
    case AppAction::RecordStart:
        if(currentWorkflow()!=Workflow::Normal) return rejectControl(ControlError::Busy);
        return startRecording(millis());
    case AppAction::RecordStop: return stopRecording(millis());
    case AppAction::RecordToggle:
        return dispatchAction({isRecording()?AppAction::RecordStop:AppAction::RecordStart,0});
    case AppAction::Play:
        if(teachingActive() || isRecording() || isMotionBusy()) return rejectControl(ControlError::Busy);
        return startPlayback();
    case AppAction::Home: case AppAction::Hold: case AppAction::Go: case AppAction::Cancel: break;
    case AppAction::PickNext:
        if(!dispatchAction({AppAction::Pick,nextObject})) return false;
        nextObject=(nextObject+1)%3; return true;
    case AppAction::TeachStart: return beginTeach();
    case AppAction::DrawHeight: return captureTeachHeight(true);
    case AppAction::TravelHeight: return captureTeachHeight(false);
    case AppAction::TeachAlignment: return captureTeachAlignment();
    case AppAction::Marking: return beginMarking();
    case AppAction::Mark: return markTeachPoint();
    case AppAction::Clear: return clearTeachPoints();
    case AppAction::Undo: return undoTeachPoint();
    case AppAction::TeachExit: return exitTeach();
    case AppAction::Polyline: return startPolyline();
    case AppAction::Curve: return startCurve();
    // Fixed paper-space presets, compiled as constants (not EEPROM data).
    case AppAction::DrawLine: return startPreset(false);
    case AppAction::DrawShape: return startPreset(true);
    }
    return rejectControl(ControlError::Invalid);
}
}
__attribute__((noinline)) bool dispatchAction(const Action& action) {
    if(action.type==AppAction::Hold) {
        if(!pauseMotion(millis())) holdArm();
        return true;
    }
    if(action.type==AppAction::Cancel) {
        if(isRecording()) clearRecording();
        if(drawingTaskActive()) return cancelAndReturnHome();
        cancelMotion(); return true;
    }
    if(isPickSaving()) return rejectControl(ControlError::Busy);
    if(action.type==AppAction::Go) {
        if(!resumeMotion(millis())) { resumeArm(); syncManualTarget(millis()); }
        return true;
    }
    if(action.type==AppAction::Home) {
        if(isRecording()) clearRecording();
        return cancelAndReturnHome();
    }
    return dispatchServiceAction(action);
}
bool executeAppCommand(char* const* t,uint8_t count,bool& success) {
    if(commandIs(t[0],"PICK")) {
        success=count==2 && commandIs(t[1],"NEXT") ? dispatchAction({AppAction::PickNext,0}) : executePickCommand(t,count);
        return true;
    }
    int op=commandIndex(t[0],commandNames("A\0B\0C\0REC\0PLAY\0HOME\0MARK\0CLEAR\0LIST\0ORDER\0TEACH\0DRAW\0UNDO\0HOLD\0GO\0CANCEL\0"));
    if(op<0) return false;
    success=false;
    Action action{};
    switch(op) {
    case 0: case 1: case 2:
        if(count!=1) break;
        action={AppAction::Pick,static_cast<uint8_t>(op)}; goto dispatch;
    case 3:
        if(count==1) { action={AppAction::RecordToggle,0}; goto dispatch; }
        if(count!=2) break;
        if(commandIs(t[1],"START")) action={AppAction::RecordStart,0};
        else if(commandIs(t[1],"STOP")) action={AppAction::RecordStop,0};
        else break;
        goto dispatch;
    case 4: case 5: case 6: case 7: case 12:
        if(count!=1) break;
        action={op==4?AppAction::Play:op==5?AppAction::Home:op==6?AppAction::Mark:op==7?AppAction::Clear:AppAction::Undo,0}; goto dispatch;
    case 8:
        if(count!=1) break;
        for(uint8_t i=0;i<teachPointCount();++i) {
            TeachPoint point; getTeachPoint(i,point); printUnsigned(i+1); printChar(' '); printDecimal(point.x); printChar(' '); printDecimal(point.y); printNewline();
        }
        success=true; return true;
    case 9:
        if(count!=6) break;
        {
            uint8_t order[5];
            for(uint8_t i=0;i<5;++i) {
                if(t[i+1][1]!=0 || t[i+1][0]<'1' || t[i+1][0]>'5') goto invalid;
                order[i]=t[i+1][0]-'0';
            }
            success=setTeachOrder(order); return true;
        }
    case 10:
        if(count!=2) break;
        {
            int sub=commandIndex(t[1],commandNames("START\0DRAWZ\0TRAVELZ\0MARKING\0EXIT\0TEACHZ\0"));
            if(sub<0) break;
            const AppAction teachActions[]={AppAction::TeachStart,AppAction::DrawHeight,AppAction::TravelHeight,AppAction::Marking,AppAction::TeachExit,AppAction::TeachAlignment};
            action={teachActions[sub],0}; goto dispatch;
        }
    case 11:
        if(count!=2) break;
        {
            int sub=commandIndex(t[1],commandNames("LINE\0N\0POLY\0CURVE\0"));
            if(sub<0) break;
            action={static_cast<AppAction>(static_cast<uint8_t>(AppAction::DrawLine)+sub),0}; goto dispatch;
        }
    case 13: case 14: case 15:
        if(count!=1) break;
        action={static_cast<AppAction>(static_cast<uint8_t>(AppAction::Hold)+op-13),0}; goto dispatch;
    }
invalid:
    rejectControl(ControlError::Invalid); return true;
dispatch:
    success=dispatchAction(action); return true;
}
void printApplicationStatus() {
    printText(F("pickNext=")); printChar('A'+nextObject); printNewline();
    printText(F("workflow=")); printlnUnsigned(static_cast<int>(currentWorkflow()));
    printRecordingStatus();
    printText(F("points=")); printUnsigned(teachPointCount()); printText(F(" paper=")); printlnUnsigned(drawSettingsValid());
    const TeachSettings& z=teachSettings();
    printText(F("Z=")); printDecimal(z.drawZ); printChar('/'); printDecimal(z.teachZ); printChar('/'); printDecimal(z.travelZ);
    printChar(' '); printlnUnsigned(z.drawValid | (z.teachValid<<1) | (z.travelValid<<2));
}
