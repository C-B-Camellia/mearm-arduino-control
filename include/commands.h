#pragma once

void pollSerial();
bool parseNumber(const char* text,float& value);

// Future external-button messages and joystick requests will use the same arm
// and motion APIs. Add their real input handling after wiring/protocols are
// confirmed; do not register placeholder commands that appear to succeed.
