#include "pick_place.h"
#include "hardware_config.h"
#include <string.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif
namespace {
// Servo output is integer degrees. Store those exact calibrated commands in
// flash; expand to the existing Pose API before all normal motion safety checks.
struct PickProgram { bool valid; uint8_t stages[PICK_STAGE_COUNT][JointCount]; uint16_t dwell[PICK_STAGE_COUNT]; };
#ifdef __AVR__
static_assert(sizeof(PickProgram)==1+PICK_STAGE_COUNT*(JointCount+sizeof(uint16_t)),"unexpected AVR pick table padding");
#endif
// A/B/C: measured approach, descend, grip, lift, transport, lower, release,
// retreat. Set valid only after checking real object/placement trajectories.
#ifdef MEARM_NATIVE_TEST
PickProgram programs[3]{};
#else
const PickProgram programs[3] PROGMEM={};
#endif
void readProgram(uint8_t object,uint8_t stage,Pose& p,uint16_t& dwell) {
#ifdef __AVR__
    for(uint8_t i=0;i<JointCount;++i) p.angles[i]=pgm_read_byte(&programs[object].stages[stage][i]);
    dwell=pgm_read_word(&programs[object].dwell[stage]);
#else
    for(uint8_t i=0;i<JointCount;++i) p.angles[i]=programs[object].stages[stage][i];
    dwell=programs[object].dwell[stage];
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
    programs[object].valid=false;
    for(uint8_t i=0;i<PICK_STAGE_COUNT;++i) {
        Pose quantized;
        if(!prepareOutputPose(poses[i],quantized)) return;
        for(uint8_t j=0;j<JointCount;++j) programs[object].stages[i][j]=static_cast<uint8_t>(quantized.angles[j]);
        programs[object].dwell[i]=100;
    }
    programs[object].valid=true;
}
void clearSyntheticPickPrograms() { memset(programs,0,sizeof(programs)); }
#endif
