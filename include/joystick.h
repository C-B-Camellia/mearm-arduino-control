#pragma once

#include <Arduino.h>

enum class JoystickAxis : uint8_t {
    LX,
    LY,
    RY,
    RX,
    Count
};
struct JoystickAxisConfig {
    uint8_t pin;
    int center;
    int deadzone;
    int direction;
};
struct JoystickInput {
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;
};

JoystickInput readJoysticks();