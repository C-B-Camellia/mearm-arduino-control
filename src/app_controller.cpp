#include "app_controller.h"
#include "command_text.h"
#include "commands.h"
#include "motion.h"
#include "recording.h"
#include "control_error.h"
#include "teaching.h"
#include "path_planner.h"
#include "serial_format.h"
#include <string.h>
namespace { uint8_t nextObject=0; }
Workflow currentWorkflow() { return teachingActive()?(markingActive()?Workflow::TeachMark:Workflow::TeachSetup):Workflow::Normal; }
void initializeApplication() { nextObject=0; clearRecording(); initializeTeaching(); }
bool dispatchAction(const Action& action,const float* values) {
    switch(action.type) {
    case AppAction::Pick:
        if(teachingActive()) return rejectControl(ControlError::Busy);
        return startPickPlace(action.argument);
    case AppAction::RecordStart:
        if(currentWorkflow()!=Workflow::Normal) return rejectControl(ControlError::Busy);
        Serial.println(F("REC replaces previous RAM recording")); return startRecording(millis());
    case AppAction::RecordStop: return stopRecording(millis());
    case AppAction::RecordToggle:
        return dispatchAction({isRecording()?AppAction::RecordStop:AppAction::RecordStart,0});
    case AppAction::Play:
        if(teachingActive()) return rejectControl(ControlError::Busy);
        return startPlayback();
    case AppAction::Home:
        if(isRecording()) return rejectControl(ControlError::Busy);
        return cancelAndReturnHome();
    case AppAction::Button: return onButtonPressed(action.argument);
    case AppAction::TeachStart: return beginTeach();
    case AppAction::DrawHeight: return captureTeachHeight(true);
    case AppAction::TravelHeight: return captureTeachHeight(false);
    case AppAction::Marking: return beginMarking();
    case AppAction::Mark: return markTeachPoint();
    case AppAction::Clear: return clearTeachPoints();
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
bool onButtonPressed(uint8_t button) {
    if(button<1 || button>4) return rejectControl(ControlError::Invalid);
    if(drawingTaskActive()) {
        if(button==1) return pauseMotion(millis());
        if(button==2) return resumeMotion(millis());
        if(button==3) return cancelAndReturnHome();
        return true;
    }
    if(isRecording() && button!=2) return rejectControl(ControlError::Busy);
    if(isMotionBusy() && button!=4) return rejectControl(ControlError::Busy);
    if(teachingActive()) {
        if(isMotionBusy()) return rejectControl(ControlError::Busy);
        if(markingActive()) {
            const AppAction actions[]={AppAction::Mark,AppAction::Polyline,AppAction::Curve,AppAction::Clear};
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
bool executeAppCommand(char* const* t,uint8_t count,bool& success) {
    const char* c=t[0];
    int op=commandIndex(c,commandNames("A\0B\0C\0REC\0PLAY\0HOME\0BTN\0MARK\0CLEAR\0LIST\0ORDER\0TEACH\0DRAW\0"));
    if(op<0) return false;
    success=false; Action action{};
    if(count==1 && (op==0 || op==1 || op==2)) action={AppAction::Pick,static_cast<uint8_t>(c[0]-'A')};
    else if(op==3 && count==1) action={AppAction::RecordToggle,0};
    else if(op==3 && count==2 && commandIs(t[1],"START")) action={AppAction::RecordStart,0};
    else if(op==3 && count==2 && commandIs(t[1],"STOP")) action={AppAction::RecordStop,0};
    else if(op==4 && count==1) action={AppAction::Play,0};
    else if(op==5 && count==1) action={AppAction::Home,0};
    else if(op==6 && count==2 && strlen(t[1])==1 && t[1][0]>='1' && t[1][0]<='4') action={AppAction::Button,static_cast<uint8_t>(t[1][0]-'0')};
    else if(op==7 && count==1) action={AppAction::Mark,0};
    else if(op==8 && count==1) action={AppAction::Clear,0};
    else if(op==9 && count==1) {
        for(uint8_t i=0;i<teachPointCount();++i) {
            TeachPoint point; getTeachPoint(i,point); Serial.print(i+1); Serial.print(' '); printDecimal(point.x); Serial.print(' '); printDecimal(point.y); Serial.println();
        }
        success=true; return true;
    }
    else if(op==10 && count==6) {
        uint8_t order[5];
        for(uint8_t i=0;i<5;++i) {
            if(strlen(t[i+1])!=1 || t[i+1][0]<'1' || t[i+1][0]>'5') { rejectControl(ControlError::Invalid); return true; }
            order[i]=t[i+1][0]-'0';
        }
        success=setTeachOrder(order); return true;
    }
    else if(op==11 && count==2) {
        const char* sub=t[1];
        if(commandIs(sub,"START")) action={AppAction::TeachStart,0};
        else if(commandIs(sub,"DRAWZ")) action={AppAction::DrawHeight,0};
        else if(commandIs(sub,"TRAVELZ")) action={AppAction::TravelHeight,0};
        else if(commandIs(sub,"MARKING")) action={AppAction::Marking,0};
        else if(commandIs(sub,"EXIT")) action={AppAction::TeachExit,0};
        else { rejectControl(ControlError::Invalid); return true; }
    }
    else if(op==12 && count==2 && commandIs(t[1],"POLY")) action={AppAction::Polyline,0};
    else if(op==12 && count==2 && commandIs(t[1],"CURVE")) action={AppAction::Curve,0};
    else if(op==12 && count==6 && commandIs(t[1],"LINE")) {
        float values[4];
        for(uint8_t i=0;i<4;++i) if(!parseNumber(t[i+2],values[i])) { rejectControl(ControlError::Invalid); return true; }
        success=dispatchAction({AppAction::DrawLine,0},values); return true;
    }
    else if(op==12 && (count==3 || count==7) && commandIs(t[1],"SHAPE")) {
        uint8_t shape=commandIs(t[2],"N")?0:commandIs(t[2],"TRIANGLE")?1:commandIs(t[2],"Z")?2:commandIs(t[2],"V")?3:4;
        if(shape==4) { rejectControl(ControlError::Invalid); return true; }
        float values[4];
        if(count==7) {
            for(uint8_t i=0;i<4;++i) if(!parseNumber(t[i+3],values[i])) { rejectControl(ControlError::Invalid); return true; }
        } else {
            TeachPoint a,b;
            if(!getTeachPoint(0,a) || !getTeachPoint(1,b)) { rejectControl(ControlError::NoData); return true; }
            values[0]=min(a.x,b.x); values[1]=min(a.y,b.y); values[2]=max(a.x,b.x); values[3]=max(a.y,b.y);
        }
        success=dispatchAction({AppAction::DrawShape,shape},values); return true;
    }
    else { rejectControl(ControlError::Invalid); return true; }
    success=dispatchAction(action); return true;
}
void printApplicationStatus() {
    RecordingStatus s=recordingStatus();
    Serial.print(F("workflow=")); Serial.print(static_cast<int>(currentWorkflow()));
    Serial.print(F(" rec=")); Serial.print(static_cast<int>(s.state));
    Serial.print(F(" frames=")); Serial.print(s.count); Serial.print(F(" ms=")); Serial.println(s.duration);
    Serial.print(F("points=")); Serial.print(teachPointCount()); Serial.print(F(" paper=")); Serial.println(drawSettingsValid());
}
