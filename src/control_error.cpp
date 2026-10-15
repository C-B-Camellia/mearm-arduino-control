#include "serial_format.h"
#include "control_error.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif
namespace {
ControlError error=ControlError::None;
// Fixed-width protocol codes; legacy ControlError values 0..10 stay stable.
const char errorCodes[] PROGMEM="E05E01E02E03E04E05E06E07E08E09E10T01T02T03D01P01P02";
static_assert(sizeof(errorCodes)==3*(static_cast<uint8_t>(ControlError::PickStage)+1)+1,"error table mismatch");
}
ControlError lastControlError() { return error; }
bool rejectControl(ControlError e) { error=e; return false; }
void clearControlError() { error=ControlError::None; }
void printControlError() {
    uint8_t index=static_cast<uint8_t>(error);
    if(index>static_cast<uint8_t>(ControlError::PickStage)) index=0;
    printText(F("ERR "));
    for(uint8_t i=0;i<3;++i) {
#ifdef __AVR__
        printChar(pgm_read_byte(errorCodes+3*index+i));
#else
        printChar(errorCodes[3*index+i]);
#endif
    }
    printNewline();
}
