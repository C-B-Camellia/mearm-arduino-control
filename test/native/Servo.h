#pragma once

#include "Arduino.h"

class Servo {
    int pin = -1;
public:
    void attach(int value)
    {
        pin = value;
        attachedPins.push_back(pin);
    }
    void write(int angle)
    {
        servoWrites.push_back({pin, angle, millis()});
    }
};
