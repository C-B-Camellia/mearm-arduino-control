#pragma once
#include <Arduino.h>
// Bounded diagnostics only; never quantizes a motion/IK input.
void printDecimal(float value,uint8_t digits=2);

void printText(const __FlashStringHelper*);
void printLine(const __FlashStringHelper*);
void printChar(char);
void printNewline();
void printUnsigned(uint16_t);
void printlnUnsigned(uint16_t);
