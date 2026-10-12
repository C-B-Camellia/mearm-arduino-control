#include "joystick.h"

namespace {
    JoystickAxisConfig joysticks[static_cast<uint8_t>(JoystickAxis::Count)] = {
        {A0, 506, 20, 1}, // Base
        {A1, 519, 20, -1},  // Shoulder / J1
        {A3, 519, 20, 1},  // Elbow / J2
        {A2, 515, 20, 1}  // Gripper
    };
}

int8_t quantizeAxis(int raw, const JoystickAxisConfig& config) {
    int delta = raw - config.center;
    if (abs(delta) <= config.deadzone) {
        return 0;
    }
    if (raw <= 8) {
        return -2 * config.direction;
    }
    if (raw >= 1015) {
        return 2 * config.direction;
    }
    if (delta < 0) return -1 * config.direction;
    return 1 * config.direction;
}

JoystickInput readJoysticks() {
    JoystickInput input{};

    input.lx = quantizeAxis(
        analogRead(joysticks[static_cast<uint8_t>(JoystickAxis::LX)].pin),
        joysticks[static_cast<uint8_t>(JoystickAxis::LX)]);
    input.ly = quantizeAxis(
        analogRead(joysticks[static_cast<uint8_t>(JoystickAxis::LY)].pin),
        joysticks[static_cast<uint8_t>(JoystickAxis::LY)]);
    input.rx = quantizeAxis(
        analogRead(joysticks[static_cast<uint8_t>(JoystickAxis::RX)].pin),
        joysticks[static_cast<uint8_t>(JoystickAxis::RX)]);
    input.ry = quantizeAxis(
        analogRead(joysticks[static_cast<uint8_t>(JoystickAxis::RY)].pin),
        joysticks[static_cast<uint8_t>(JoystickAxis::RY)]);
    return input;
}
