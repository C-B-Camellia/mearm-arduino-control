#pragma once
#include <Arduino.h>
enum class ControlError : uint8_t { None, Busy, Held, Uncalibrated, Disabled, Invalid, Unsafe, Unreachable, Full, NoData, TooShort, NoMovement, LinkUnconfigured };
ControlError lastControlError();
bool rejectControl(ControlError error);
void clearControlError();
void printControlError();
