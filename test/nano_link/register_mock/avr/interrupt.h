#pragma once
#include <Arduino.h>
inline void cli() { SREG&=0x7f; }
#define ISR(vector) void vector()
