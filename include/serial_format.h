#pragma once
#include <Arduino.h>
// Bounded diagnostics only; never quantizes a motion/IK input.
void printDecimal(float value,uint8_t digits=2);
