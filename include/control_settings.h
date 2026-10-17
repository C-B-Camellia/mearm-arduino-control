#pragma once
#include <Arduino.h>
constexpr float kPenGripAngle = 85.0f;
float manualSpeedScale();
bool setManualSpeedScale(float scale);
void adjustManualSpeedScale(bool faster);
float automaticSpeed();
// Development comparisons use -DMEARM_DRAW_SPEED_MM_S=...; no runtime setter.
#ifndef MEARM_DRAW_SPEED_MM_S
#define MEARM_DRAW_SPEED_MM_S 80.0f
#endif
constexpr float kDrawSpeedMmS = MEARM_DRAW_SPEED_MM_S;
static_assert(kDrawSpeedMmS > 0.0f, "Drawing speed must be positive");
#ifndef MEARM_FIVE_POINT_SPEED_MM_S
#define MEARM_FIVE_POINT_SPEED_MM_S 80.0f
#endif
constexpr float kFivePointDrawSpeedMmS = MEARM_FIVE_POINT_SPEED_MM_S;
static_assert(kFivePointDrawSpeedMmS > 0.0f, "Five-point speed must be positive");
// Base transition speed; only pen-down applies the local scale below.
#ifndef MEARM_PEN_TRANSITION_SPEED_MM_S
#define MEARM_PEN_TRANSITION_SPEED_MM_S 25.0f
#endif
#ifndef MEARM_PEN_DOWN_SPEED_SCALE
#define MEARM_PEN_DOWN_SPEED_SCALE 1.50f
#endif
constexpr float kPenTransitionSpeedMmS = MEARM_PEN_TRANSITION_SPEED_MM_S;
constexpr float kPenDownSpeedScale = MEARM_PEN_DOWN_SPEED_SCALE;
bool setAutomaticSpeed(float speed);
void resetControlSettings();
