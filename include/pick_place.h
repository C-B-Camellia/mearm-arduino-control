#pragma once
#include "arm.h"
constexpr uint8_t PICK_STAGE_COUNT=8;
bool pickCalibrationValid(uint8_t object);
bool getPickStage(uint8_t object,uint8_t stage,Pose& pose,uint16_t& dwellMs);
#ifdef MEARM_NATIVE_TEST
void setSyntheticPickProgram(uint8_t object,const Pose (&poses)[PICK_STAGE_COUNT]);
void clearSyntheticPickPrograms();
#endif
