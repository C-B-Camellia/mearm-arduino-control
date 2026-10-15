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
#include <string.h>
namespace { uint8_t nextObject=0; }
Workflow currentWorkflow() { return teachingActive()?(markingActive()?Workflow::TeachMark:Workflow::TeachSetup):Workflow::Normal; }
void initializeApplication() { nextObject=0; clearRecording(); initializeTeaching(); initializePickConfiguration(); }
namespace {
// Heavy service scratch is only needed by the service itself. Button/HOME
// forwarding must not carry this frame into cancellation's IK/FK call chain.
__attribute__((noinline)) bool dispatchServiceAction(const Action& action,const float* values) {
    switch(action.type) {
    case AppAction::Pick:
        if(teachingActive()) return rejectControl(ControlError::Busy);
        return startPickPlace(action.argument);
    case AppAction::RecordStart:
        if(currentWorkflow()!=Workflow::Normal) return rejectControl(ControlError::Busy);
        return startRecording(millis());
    case AppAction::RecordStop: return stopRecording(millis());
    case AppAction::RecordToggle:
        return dispatchAction({isRecording()?AppAction::RecordStop:AppAction::RecordStart,0});
    case AppAction::Play:
        if(teachingActive()) return rejectControl(ControlError::Busy);
        return startPlayback();
    case AppAction::Home:
    case AppAction::Button: break; // Handled by the public event entry below.
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
    case AppAction::DrawLine:
        if(!values) return rejectControl(ControlError::Invalid);
        return startDrawLine({values[0],values[1]},{values[2],values[3]});
    case AppAction::DrawShape:
        if(!values || action.argument>3) return rejectControl(ControlError::Invalid);
        return startBasicShape(static_cast<ShapeId>(action.argument),{values[0],values[1]},{values[2],values[3]});
    }
    return rejectControl(ControlError::Invalid);
}
}
__attribute__((noinline)) bool dispatchAction(const Action& action,const float* values) {
    if(isPickSaving()) return rejectControl(ControlError::Busy);
    if(action.type==AppAction::Button) return onButtonPressed(action.argument);
    if(action.type==AppAction::Home) {
        if(isRecording()) clearRecording();
        return cancelAndReturnHome();
    }
    return dispatchServiceAction(action,values);
}
bool onButtonPressed(uint8_t button) {
    if(isPickSaving()) return rejectControl(ControlError::Busy);
    if(button<1 || button>4) return rejectControl(ControlError::Invalid);
    if(drawingTaskActive()) {
        if(button==1) return pauseMotion(millis());
        if(button==2) return resumeMotion(millis());
        if(button==3) return cancelAndReturnHome();
        return true;
    }
    if(button==4 && isMotionBusy() && motionStatus().kind==TaskKind::TeachPosition) return exitTeach();
    if(isRecording() && button!=2 && button!=4) return rejectControl(ControlError::Busy);
    if(isMotionBusy() && button!=4) return rejectControl(ControlError::Busy);
    if(teachingActive()) {
        if(isMotionBusy()) return rejectControl(ControlError::Busy);
        if(markingActive()) {
            const AppAction actions[]={AppAction::Mark,AppAction::Polyline,AppAction::Curve,AppAction::Undo};
            return dispatchAction({actions[button-1],0});
        }
        const AppAction actions[]={AppAction::DrawHeight,AppAction::TravelHeight,AppAction::Marking,AppAction::TeachExit};
        return dispatchAction({actions[button-1],0});
    }
    switch(button) {
    case 1:
        if(!dispatchAction({AppAction::Pick,nextObject})) return false;
        nextObject=(nextObject+1)%3; return true;
    case 2: return dispatchAction({AppAction::RecordToggle,0});
    case 3: return dispatchAction({AppAction::Play,0});
    case 4: return dispatchAction({AppAction::Home,0});
    }
    return false;
}
namespace {
__attribute__((noinline)) bool executeDrawCommand(char* const* t,uint8_t count) {
    if(count==2) {
        if(commandIs(t[1],"POLY")) return startPolyline();
        if(commandIs(t[1],"CURVE")) return startCurve();
    }
    float values[4];
    if(count==6 && commandIs(t[1],"LINE")) {
        if(!parseNumbers(t+2,4,values)) return rejectControl(ControlError::Invalid);
        return dispatchAction({AppAction::DrawLine,0},values);
    }
    if((count==3 || count==7) && commandIs(t[1],"SHAPE")) {
        int shape=commandIndex(t[2],commandNames("N\0TRIANGLE\0Z\0V\0"));
        if(shape<0) return rejectControl(ControlError::Invalid);
        if(count==7) {
            if(!parseNumbers(t+3,4,values)) return rejectControl(ControlError::Invalid);
        } else {
            TeachPoint a,b;
            if(!getTeachPoint(0,a) || !getTeachPoint(1,b)) return rejectControl(ControlError::NoData);
            values[0]=min(a.x,b.x); values[1]=min(a.y,b.y); values[2]=max(a.x,b.x); values[3]=max(a.y,b.y);
        }
        return dispatchAction({AppAction::DrawShape,static_cast<uint8_t>(shape)},values);
    }
    return rejectControl(ControlError::Invalid);
}
}
bool executeAppCommand(char* const* t,uint8_t count,bool& success) {
    if(commandIs(t[0],"PICK")) { success=executePickCommand(t,count); return true; }
    int op=commandIndex(t[0],commandNames("A\0B\0C\0REC\0PLAY\0HOME\0BTN\0MARK\0CLEAR\0LIST\0ORDER\0TEACH\0DRAW\0UNDO\0"));
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
    case 4: case 5: case 7: case 8: case 13:
        if(count!=1) break;
        action={op==4?AppAction::Play:op==5?AppAction::Home:op==7?AppAction::Mark:op==8?AppAction::Clear:AppAction::Undo,0}; goto dispatch;
    case 6:
        if(count!=2 || t[1][1]!=0 || t[1][0]<'1' || t[1][0]>'4') break;
        action={AppAction::Button,static_cast<uint8_t>(t[1][0]-'0')}; goto dispatch;
    case 9:
        if(count!=1) break;
        for(uint8_t i=0;i<teachPointCount();++i) {
            TeachPoint point; getTeachPoint(i,point); printUnsigned(i+1); printChar(' '); printDecimal(point.x); printChar(' '); printDecimal(point.y); printNewline();
        }
        success=true; return true;
    case 10:
        if(count!=6) break;
        {
            uint8_t order[5];
            for(uint8_t i=0;i<5;++i) {
                if(t[i+1][1]!=0 || t[i+1][0]<'1' || t[i+1][0]>'5') goto invalid;
                order[i]=t[i+1][0]-'0';
            }
            success=setTeachOrder(order); return true;
        }
    case 11:
        if(count!=2) break;
        {
            int sub=commandIndex(t[1],commandNames("START\0DRAWZ\0TRAVELZ\0MARKING\0EXIT\0TEACHZ\0"));
            if(sub<0) break;
            action={sub<4?static_cast<AppAction>(static_cast<uint8_t>(AppAction::TeachStart)+sub):sub==4?AppAction::TeachExit:AppAction::TeachAlignment,0}; goto dispatch;
        }
    case 12: success=executeDrawCommand(t,count); return true;
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
