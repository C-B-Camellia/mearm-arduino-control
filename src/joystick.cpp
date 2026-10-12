#include "joystick.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif

namespace {
    const JoystickAxisConfig joysticks[static_cast<uint8_t>(JoystickAxis::Count)] PROGMEM = {
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
    int8_t axes[4];
    for(uint8_t i=0;i<4;++i) {
#ifdef __AVR__
        JoystickAxisConfig config;
        memcpy_P(&config,&joysticks[i],sizeof(config));
#else
        const JoystickAxisConfig& config=joysticks[i];
#endif
        axes[i]=quantizeAxis(analogRead(config.pin),config);
    }
    // JoystickAxis order is LX/LY/RY/RX; JoystickInput is LX/LY/RX/RY.
    return {axes[0],axes[1],axes[3],axes[2]};
}
bool joysticksCentered(const JoystickInput& input) { return input.lx==0 && input.ly==0 && input.rx==0 && input.ry==0; }
