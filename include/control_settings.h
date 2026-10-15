#pragma once
#include <Arduino.h>
constexpr float kPenGripAngle = 85.0f;
float manualSpeedScale();
bool setManualSpeedScale(float scale);
void adjustManualSpeedScale(bool faster);
float automaticSpeed();
// Development comparisons use -DMEARM_DRAW_SPEED_MM_S=...; no runtime setter.
#ifndef MEARM_DRAW_SPEED_MM_S
#define MEARM_DRAW_SPEED_MM_S 35.0f
#endif
constexpr float kDrawSpeedMmS = MEARM_DRAW_SPEED_MM_S;
static_assert(kDrawSpeedMmS > 0.0f, "Drawing speed must be positive");
#ifndef MEARM_FIVE_POINT_SPEED_MM_S
#define MEARM_FIVE_POINT_SPEED_MM_S 70.0f
#endif
constexpr float kFivePointDrawSpeedMmS = MEARM_FIVE_POINT_SPEED_MM_S;
static_assert(kFivePointDrawSpeedMmS > 0.0f, "Five-point speed must be positive");
// Preserve the field-fixes pen-down/up timing independently of stroke speed.
constexpr float kPenTransitionSpeedMmS = 25.0f;
bool setAutomaticSpeed(float speed);
void resetControlSettings();
