#include "serial_format.h"
#include "control_error.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif
namespace {
ControlError error=ControlError::None;
const char errorNames[] PROGMEM="INVALID\0BUSY\0HELD\0UNCALIBRATED\0DISABLED\0UNSAFE\0UNREACHABLE\0FULL\0NO_DATA\0GEOMETRY\0";
const uint8_t errorOffsets[] PROGMEM={0,8,13,18,31,0,40,47,59,64,72};
static_assert(sizeof(errorOffsets)==static_cast<uint8_t>(ControlError::Geometry)+1,"error table mismatch");
}
ControlError lastControlError() { return error; }
bool rejectControl(ControlError e) { error=e; return false; }
void clearControlError() { error=ControlError::None; }
void printControlError() {
    uint8_t index=static_cast<uint8_t>(error);
    if(index>=sizeof(errorOffsets)) index=0;
    printText(F("ERR "));
#ifdef __AVR__
    Serial.println(reinterpret_cast<const __FlashStringHelper*>(errorNames+pgm_read_byte(errorOffsets+index)));
#else
    Serial.println(errorNames+errorOffsets[index]);
#endif
}
