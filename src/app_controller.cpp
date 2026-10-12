#include "app_controller.h"
#include "commands.h"
#include "motion.h"
#include "recording.h"
#include "control_error.h"
#include <string.h>
namespace { Workflow workflow=Workflow::Normal; uint8_t nextObject=0; }
Workflow currentWorkflow() { return workflow; }
void initializeApplication() { workflow=Workflow::Normal; nextObject=0; clearRecording(); }
bool dispatchAction(const Action& action) {
    switch(action.type) {
    case AppAction::Pick: return startPickPlace(action.argument);
    case AppAction::RecordStart:
        if(workflow!=Workflow::Normal) return rejectControl(ControlError::Busy);
        Serial.println(F("REC replaces previous RAM recording")); return startRecording(millis());
    case AppAction::RecordStop: return stopRecording(millis());
    case AppAction::RecordToggle:
        return dispatchAction({isRecording()?AppAction::RecordStop:AppAction::RecordStart,0});
    case AppAction::Play: return startPlayback();
    case AppAction::Home:
        if(isRecording()) return rejectControl(ControlError::Busy);
        return cancelAndReturnHome();
    case AppAction::Button: return onButtonPressed(action.argument);
    }
    return rejectControl(ControlError::Invalid);
}
bool onButtonPressed(uint8_t button) {
    if(button<1 || button>4) return rejectControl(ControlError::Invalid);
    if(isRecording() && button!=2) return rejectControl(ControlError::Busy);
    if(isMotionBusy() && button!=4) return rejectControl(ControlError::Busy);
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
    bool known=!strcmp(c,"A") || !strcmp(c,"B") || !strcmp(c,"C") || !strcmp(c,"REC") || !strcmp(c,"PLAY") || !strcmp(c,"HOME") || !strcmp(c,"BTN");
    if(!known) return false;
    success=false; Action action{};
    if(count==1 && (!strcmp(c,"A") || !strcmp(c,"B") || !strcmp(c,"C"))) action={AppAction::Pick,static_cast<uint8_t>(c[0]-'A')};
    else if(!strcmp(c,"REC") && count==1) action={AppAction::RecordToggle,0};
    else if(!strcmp(c,"REC") && count==2 && !strcmp(t[1],"START")) action={AppAction::RecordStart,0};
    else if(!strcmp(c,"REC") && count==2 && !strcmp(t[1],"STOP")) action={AppAction::RecordStop,0};
    else if(!strcmp(c,"PLAY") && count==1) action={AppAction::Play,0};
    else if(!strcmp(c,"HOME") && count==1) action={AppAction::Home,0};
    else if(!strcmp(c,"BTN") && count==2 && strlen(t[1])==1 && t[1][0]>='1' && t[1][0]<='4') action={AppAction::Button,static_cast<uint8_t>(t[1][0]-'0')};
    else { rejectControl(ControlError::Invalid); return true; }
    success=dispatchAction(action); return true;
}
void printApplicationStatus() {
    RecordingStatus s=recordingStatus();
    Serial.print(F("workflow=")); Serial.print(static_cast<int>(workflow));
    Serial.print(F(" rec=")); Serial.print(static_cast<int>(s.state));
    Serial.print(F(" frames=")); Serial.print(s.count); Serial.print(F(" ms=")); Serial.println(s.duration);
}
