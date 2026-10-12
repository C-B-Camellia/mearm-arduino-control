#pragma once
#include <Arduino.h>
float manualSpeedScale();
bool setManualSpeedScale(float scale);
void adjustManualSpeedScale(bool faster);
float automaticSpeed();
bool setAutomaticSpeed(float speed);
void resetControlSettings();
