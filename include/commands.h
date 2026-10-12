#pragma once
#include <stdint.h>

void pollSerial();
bool parseNumber(const char* text,float& value);
bool parseNumbers(char* const* text,uint8_t count,float* values);

// Future external-button messages and joystick requests will use the same arm
// and motion APIs. Add their real input handling after wiring/protocols are
// confirmed; do not register placeholder commands that appear to succeed.
