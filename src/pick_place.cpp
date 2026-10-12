#include "pick_place.h"
#include "hardware_config.h"
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif
namespace {
struct PickProgram { bool valid; Pose stages[PICK_STAGE_COUNT]; uint16_t dwell[PICK_STAGE_COUNT]; };
// A/B/C: measured approach, descend, grip, lift, transport, lower, release,
// retreat. Set valid only after checking real object/placement trajectories.
#ifdef MEARM_NATIVE_TEST
PickProgram programs[3]{};
#else
const PickProgram programs[3] PROGMEM={};
#endif
void readProgram(uint8_t object,uint8_t stage,Pose& p,uint16_t& dwell) {
#ifdef __AVR__
    memcpy_P(&p,&programs[object].stages[stage],sizeof(Pose));
    dwell=pgm_read_word(&programs[object].dwell[stage]);
#else
    p=programs[object].stages[stage]; dwell=programs[object].dwell[stage];
#endif
}
}
bool pickCalibrationValid(uint8_t object) {
    if(object>=3 || !gripperCalibrationValid() || !joints[Base].outputEnabled) return false;
#ifdef __AVR__
    return pgm_read_byte(&programs[object].valid);
#else
    return programs[object].valid;
#endif
}
bool getPickStage(uint8_t object,uint8_t stage,Pose& p,uint16_t& dwell) {
    if(stage>=PICK_STAGE_COUNT || !pickCalibrationValid(object)) return false;
    readProgram(object,stage,p,dwell); return true;
}
#ifdef MEARM_NATIVE_TEST
void setSyntheticPickProgram(uint8_t object,const Pose (&poses)[PICK_STAGE_COUNT]) {
    if(object>=3) return;
    programs[object].valid=true;
    for(uint8_t i=0;i<PICK_STAGE_COUNT;++i) { programs[object].stages[i]=poses[i]; programs[object].dwell[i]=100; }
}
void clearSyntheticPickPrograms() { memset(programs,0,sizeof(programs)); }
#endif
