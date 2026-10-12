#include "control_error.h"
namespace { ControlError error=ControlError::None; }
ControlError lastControlError() { return error; }
bool rejectControl(ControlError e) { error=e; return false; }
void clearControlError() { error=ControlError::None; }
void printControlError() {
    Serial.print(F("ERR "));
    switch(error) {
    case ControlError::None: Serial.println(F("INVALID")); break;
    case ControlError::Busy: Serial.println(F("BUSY")); break;
    case ControlError::Held: Serial.println(F("HELD")); break;
    case ControlError::Uncalibrated: Serial.println(F("UNCALIBRATED")); break;
    case ControlError::Disabled: Serial.println(F("DISABLED")); break;
    case ControlError::Invalid: Serial.println(F("INVALID")); break;
    case ControlError::Unsafe: Serial.println(F("UNSAFE")); break;
    case ControlError::Unreachable: Serial.println(F("UNREACHABLE")); break;
    case ControlError::Full: Serial.println(F("FULL")); break;
    case ControlError::NoData: Serial.println(F("NO_DATA")); break;
    case ControlError::TooShort: Serial.println(F("TOO_SHORT")); break;
    case ControlError::NoMovement: Serial.println(F("NO_MOVEMENT")); break;
    case ControlError::LinkUnconfigured: Serial.println(F("LINK_UNCONFIGURED")); break;
    }
}
